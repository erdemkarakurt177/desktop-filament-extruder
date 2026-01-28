#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =====================================================
// =====================  LCD SECTION  =================
// =====================================================
#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

unsigned long programStartMs = 0;
unsigned long lastLcdUpdate  = 0;
const unsigned long LCD_PERIOD_MS = 250; // 4 Hz

unsigned long lastSerialPrint = 0;
const unsigned long SERIAL_PERIOD_MS = 500; // 2 Hz

// =====================================================
// =================== STATE MACHINE ===================
// =====================================================
enum SystemState {
  STATE_IDLE = 0,        // Waiting for UI command
  STATE_AUTOTUNING,      // AutoTune in progress
  STATE_HEATING,         // Heater ON
  STATE_READY,           // Temp reached, waiting for extrusion command
  STATE_EXTRUDING,       // E0 only, waiting for winding command
  STATE_FULL_AUTO,       // All motors + optical sensor feedback
  STATE_TEST_OPTIC,      // X+Y only test (heater/E0 forced OFF)
  STATE_EMERGENCY        // Emergency stop
};

SystemState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;

// =====================================================
// ================== RAMPS PIN DEFINES =================
// =====================================================
// X = spool
#define X_STEP_PIN    54
#define X_DIR_PIN     55
#define X_ENABLE_PIN  38

// Y = traverse
#define Y_STEP_PIN    60
#define Y_DIR_PIN     61
#define Y_ENABLE_PIN  56

// E0 = extrusion
#define E0_STEP_PIN   26
#define E0_DIR_PIN    28
#define E0_ENABLE_PIN 24

// Optical sensor (D44)
#define OPTIC_PIN 44

// Fans
#define FAN_MAIN_PIN  9

// Heaters / thermistor
#define TEMP_PIN          A13
#define HEATER_MAIN_PIN   10
#define HEATER_AUX_PIN     8

// =====================================================
// =================== OPTICAL SENSOR ==================
// =====================================================
const uint32_t DEBOUNCE_MS = 40;

static inline bool opticTriggered() {
  return digitalRead(OPTIC_PIN) == LOW; // LOW when triggered
}

static inline bool opticTriggeredDebounced() {
  static bool lastStable = false;
  static bool lastRead   = false;
  static uint32_t tChange = 0;

  bool now = opticTriggered();

  if (now != lastRead) {
    lastRead = now;
    tChange = millis();
  }
  if (millis() - tChange > DEBOUNCE_MS) {
    lastStable = now;
  }
  return lastStable;
}

// =====================================================
// ===================== SPEED SETTINGS =================
// =====================================================
// HALF-PERIOD values (smaller = faster)

// -------------------- X (spool) with optical feedback + START RAMP --------------------
// IMPORTANT: You asked to keep these exactly.
volatile uint16_t X_START_US   = 10000;
volatile uint16_t X_TARGET_US  = 8000;
volatile uint16_t X_RAMP_STEP  = 25;
volatile uint16_t X_CURRENT_US = X_START_US;
volatile bool     X_AT_TARGET  = false;

volatile uint16_t X_MIN_US     = 3000;  // trigger en hızlı
volatile uint16_t X_MAX_US     = 12000;  // en yavaş bile fena yavaş olmasın

const uint16_t TRIM_UP_STEP_US   = 80;   // trigger olunca daha agresif hızlansın
const uint16_t TRIM_BACK_STEP_US = 40;
const uint32_t CONTROL_PERIOD_MS = 200;


// -------------------- Y TRAVERSE (lead screw) --------------------
const float Y_TRAVEL_MM = 41.0;
const float Y_STEPS_PER_MM = 800.0f;

volatile uint32_t Y_STEPS_PER_TRAVEL =
  (uint32_t)(Y_TRAVEL_MM * Y_STEPS_PER_MM + 0.5f);

volatile uint16_t Y_STEP_US = 2500;
volatile uint32_t yStepCount = 0;
volatile bool     yDirForward = false;

// -------------------- E0 (extrusion) fixed speed --------------------
volatile uint16_t E0_STEP_US = 110;

// =====================================================
// ================= TEMPERATURE / PID ==================
// =====================================================
#define SERIES_RESISTOR     4700.0
#define NOMINAL_RESISTANCE  100000.0
#define NOMINAL_TEMPERATURE 25.0
#define B_COEFFICIENT       3950.0
#define ADC_MAX             1023.0

double targetTemp   = 200.0;
double currentTemp  = 0.0;
double heaterPower  = 0.0;

// PID parameters (will be updated by AutoTune)
double Kp = 5, Ki = 0.01, Kd = 0.0;

// PID and AutoTune objects
PID myPID(&currentTemp, &heaterPower, &targetTemp, Kp, Ki, Kd, DIRECT);
PID_ATune autoTune(&currentTemp, &heaterPower);

// AutoTune control
bool autoTuning = false;
unsigned long autoTuneStartTime = 0;
const unsigned long AUTOTUNE_TIMEOUT_MS = 5400000; // 90 minutes

#define MAX_TEMP 240.0
#define MIN_TEMP 5.0
const double READY_MARGIN_C = 2.0;   // 2°C kala READY kabul et

unsigned long lastTempUpdate = 0;

// =====================================================
// ===================== TIMER HELPERS ==================
// =====================================================
static inline uint16_t usToOcr(uint16_t us) {
  uint32_t ticks = (uint32_t)us * 2UL;   // prescaler 8 => 0.5us tick
  if (ticks < 20UL) ticks = 20UL;
  if (ticks > 65535UL) ticks = 65535UL;
  return (uint16_t)(ticks - 1UL);
}

// =====================================================
// ==================== TIMER1 (X) ======================
// =====================================================
volatile bool xStepHigh = false;
volatile bool xTimerActive = false;

ISR(TIMER1_COMPA_vect) {
  if (xStepHigh) {
    digitalWrite(X_STEP_PIN, LOW);
    xStepHigh = false;

    // -------- X START RAMP (ONLY HERE) --------
    if (!X_AT_TARGET) {
      if (X_CURRENT_US > X_TARGET_US) {
        uint16_t nextUs = (uint16_t)(X_CURRENT_US - X_RAMP_STEP);
        if (nextUs < X_TARGET_US) nextUs = X_TARGET_US;
        X_CURRENT_US = nextUs;
        OCR1A = usToOcr(X_CURRENT_US);
      } else {
        X_AT_TARGET = true;
      }
    }

  } else {
    digitalWrite(X_STEP_PIN, HIGH);
    xStepHigh = true;
  }
}

void timer1StartX() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  TCCR1B |= (1 << WGM12); // CTC
  TCCR1B |= (1 << CS11);  // prescaler 8

  OCR1A = usToOcr(X_CURRENT_US);

  TIFR1  |= (1 << OCF1A);
  TIMSK1 |= (1 << OCIE1A);
  xTimerActive = true;
  sei();
}

void timer1StopX() {
  TIMSK1 &= ~(1 << OCIE1A);
  digitalWrite(X_STEP_PIN, LOW);
  xTimerActive = false;
}

static inline void applyXSpeed(uint16_t halfPeriodUs) {
  X_CURRENT_US = halfPeriodUs;
  if (xTimerActive) {
    OCR1A = usToOcr(X_CURRENT_US);
  }
}

// =====================================================
// ==================== TIMER3 (Y) ======================
// =====================================================
volatile bool yStepHigh = false;
volatile bool yTimerActive = false;

ISR(TIMER3_COMPA_vect) {
  if (yStepHigh) {
    digitalWrite(Y_STEP_PIN, LOW);
    yStepHigh = false;

    yStepCount++;
    if (yStepCount >= Y_STEPS_PER_TRAVEL) {
      yStepCount = 0;
      yDirForward = !yDirForward;
      digitalWrite(Y_DIR_PIN, yDirForward ? HIGH : LOW);
    }

  } else {
    digitalWrite(Y_STEP_PIN, HIGH);
    yStepHigh = true;
  }
}

void timer3StartY() {
  cli();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;

  TCCR3B |= (1 << WGM32); // CTC
  TCCR3B |= (1 << CS31);  // prescaler 8

  OCR3A = usToOcr(Y_STEP_US);

  TIFR3  |= (1 << OCF3A);
  TIMSK3 |= (1 << OCIE3A);
  yTimerActive = true;
  sei();
}

void timer3StopY() {
  TIMSK3 &= ~(1 << OCIE3A);
  digitalWrite(Y_STEP_PIN, LOW);
  yTimerActive = false;
}

// =====================================================
// ==================== TIMER4 (E0) =====================
// =====================================================
volatile bool e0StepHigh = false;
volatile bool e0TimerActive = false;

ISR(TIMER4_COMPA_vect) {
  if (e0StepHigh) {
    digitalWrite(E0_STEP_PIN, LOW);
    e0StepHigh = false;
  } else {
    digitalWrite(E0_STEP_PIN, HIGH);
    e0StepHigh = true;
  }
}

void timer4StartE0() {
  cli();
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;

  TCCR4B |= (1 << WGM42); // CTC
  TCCR4B |= (1 << CS41);  // prescaler 8

  OCR4A = usToOcr(E0_STEP_US);

  TIFR4  |= (1 << OCF4A);
  TIMSK4 |= (1 << OCIE4A);
  e0TimerActive = true;
  sei();
}

void timer4StopE0() {
  TIMSK4 &= ~(1 << OCIE4A);
  digitalWrite(E0_STEP_PIN, LOW);
  e0TimerActive = false;
}

// =====================================================
// ================== MOTOR CONTROL ====================
// =====================================================
static inline void setDriversAll(bool enable) {
  digitalWrite(X_ENABLE_PIN,  enable ? LOW : HIGH);
  digitalWrite(Y_ENABLE_PIN,  enable ? LOW : HIGH);
  digitalWrite(E0_ENABLE_PIN, enable ? LOW : HIGH);
}

static inline void stopAllMotors() {
  timer1StopX();
  timer3StopY();
  timer4StopE0();
  setDriversAll(false);
}

// ---- TEST OPTIC helpers ----
static inline void startTestOptic() {
  // Force everything else OFF
  analogWrite(HEATER_MAIN_PIN, 0);
  analogWrite(HEATER_AUX_PIN,  0);

  timer4StopE0();
  digitalWrite(E0_ENABLE_PIN, HIGH);

  // Enable X + Y
  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);

  // X ramp init
  X_CURRENT_US = X_START_US;
  X_AT_TARGET  = false;

  // Start timers
  timer1StartX();
  timer3StartY();
}

static inline void stopTestOptic() {
  timer1StopX();
  timer3StopY();
  digitalWrite(X_ENABLE_PIN, HIGH);
  digitalWrite(Y_ENABLE_PIN, HIGH);
}

// =====================================================
// ===================== TEMP READ ======================
// =====================================================
double readTemperature() {
  double avg = analogRead(TEMP_PIN);
  if (avg < 1.0) avg = 1.0;
  if (avg > 1022.0) avg = 1022.0;

  double resistance = SERIES_RESISTOR / ((ADC_MAX / avg) - 1.0);

  double steinhart = resistance / NOMINAL_RESISTANCE;
  steinhart = log(steinhart);
  steinhart /= B_COEFFICIENT;
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;

  return steinhart;
}

// =====================================================
// =================== STATE FUNCTIONS =================
// =====================================================
void enterState(SystemState newState) {
  // Leaving test?
  if (currentState == STATE_TEST_OPTIC && newState != STATE_TEST_OPTIC) {
    stopTestOptic();
  }

  currentState = newState;
  stateStartTime = millis();

  Serial.print("STATE:");
  Serial.println(newState);

  // Entering test?
  if (newState == STATE_TEST_OPTIC) {
    startTestOptic();
  }
}

void handleStateIdle() {
  analogWrite(HEATER_MAIN_PIN, 0);
  analogWrite(HEATER_AUX_PIN, 0);
  stopAllMotors();

  lcd.setCursor(0, 0);
  lcd.print("IDLE - Ready    ");
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(currentTemp, 1);
  lcd.print("C          ");
}

void handleStateAutotuning() {
  byte val = autoTune.Runtime();

  if (val != 0) {
    autoTuning = false;

    Kp = autoTune.GetKp();
    Ki = autoTune.GetKi();
    Kd = autoTune.GetKd();

    myPID.SetTunings(Kp, Ki, Kd);

    Serial.print("TUNED:");
    Serial.print(Kp, 3); Serial.print(",");
    Serial.print(Ki, 3); Serial.print(",");
    Serial.println(Kd, 3);

    analogWrite(HEATER_MAIN_PIN, 0);
    analogWrite(HEATER_AUX_PIN, 0);

    enterState(STATE_IDLE);

  } else {
    int pwm = (int)heaterPower;
    if (pwm < 0) pwm = 0;
    if (pwm > 255) pwm = 255;
    analogWrite(HEATER_MAIN_PIN, pwm);
    analogWrite(HEATER_AUX_PIN, pwm);

    if (millis() - autoTuneStartTime > AUTOTUNE_TIMEOUT_MS) {
      autoTuning = false;
      analogWrite(HEATER_MAIN_PIN, 0);
      analogWrite(HEATER_AUX_PIN, 0);
      Serial.println("ERROR:AUTOTUNE_TIMEOUT");
      enterState(STATE_IDLE);
    }
  }

  lcd.setCursor(0, 0);
  lcd.print("AUTOTUNING...   ");
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(currentTemp, 1);
  lcd.print("C ");

  unsigned long elapsed = (millis() - stateStartTime) / 1000UL;
  unsigned int m = elapsed / 60UL;
  unsigned int s = elapsed % 60UL;
  if (m < 10) lcd.print('0');
  lcd.print(m);
  lcd.print(':');
  if (s < 10) lcd.print('0');
  lcd.print(s);
}

void handleStateHeating() {
  myPID.Compute();
  int pwm = (int)heaterPower;
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  analogWrite(HEATER_MAIN_PIN, pwm);
  analogWrite(HEATER_AUX_PIN, pwm);

  if (currentTemp >= (targetTemp - READY_MARGIN_C)) {
    enterState(STATE_READY);
  }

  lcd.setCursor(0, 0);
  lcd.print("HEATING...      ");
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(currentTemp, 1);
  lcd.print("C->");
  lcd.print(targetTemp, 0);
  lcd.print("C ");
}

void handleStateReady() {
  myPID.Compute();
  int pwm = (int)heaterPower;
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  analogWrite(HEATER_MAIN_PIN, pwm);
  analogWrite(HEATER_AUX_PIN, pwm);

  lcd.setCursor(0, 0);
  lcd.print("READY           ");
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(currentTemp, 1);
  lcd.print("C UI Cmd ");
}

void handleStateExtruding() {
  myPID.Compute();
  int pwm = (int)heaterPower;
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  analogWrite(HEATER_MAIN_PIN, pwm);
  analogWrite(HEATER_AUX_PIN, pwm);

  lcd.setCursor(0, 0);
  lcd.print("EXTRUDING       ");
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(currentTemp, 1);
  lcd.print("C Ready  ");
}

void handleStateFullAuto() {
  myPID.Compute();
  int pwm = (int)heaterPower;
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  analogWrite(HEATER_MAIN_PIN, pwm);
  analogWrite(HEATER_AUX_PIN, pwm);

  static uint32_t lastControl = 0;
  if (X_AT_TARGET && (millis() - lastControl >= CONTROL_PERIOD_MS)) {
    lastControl = millis();

    bool trig = opticTriggeredDebounced();
    uint16_t next = X_CURRENT_US;

    if (trig) {
      if (next > X_MIN_US + TRIM_UP_STEP_US) next -= TRIM_UP_STEP_US;
      else next = X_MIN_US;
    } else {
      if (next < X_TARGET_US) {
        uint16_t diff = X_TARGET_US - next;
        uint16_t step = (diff > TRIM_BACK_STEP_US) ? TRIM_BACK_STEP_US : diff;
        next += step;
      } else if (next > X_TARGET_US) {
        uint16_t diff = next - X_TARGET_US;
        uint16_t step = (diff > TRIM_BACK_STEP_US) ? TRIM_BACK_STEP_US : diff;
        next -= step;
      }
    }

    if (next < X_MIN_US) next = X_MIN_US;
    if (next > X_MAX_US) next = X_MAX_US;

    applyXSpeed(next);
  }

  lcd.setCursor(0, 0);
  lcd.print("AUTO ");
  lcd.print(opticTriggeredDebounced() ? "TRIG" : "OK  ");
  lcd.print("   ");

  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(currentTemp, 1);
  lcd.print("C ");

  unsigned long sec = (millis() - stateStartTime) / 1000UL;
  unsigned int m = sec / 60UL;
  unsigned int s = sec % 60UL;
  if (m < 10) lcd.print('0');
  lcd.print(m);
  lcd.print(':');
  if (s < 10) lcd.print('0');
  lcd.print(s);
}

void handleStateTestOptic() {
  // Hard safety: heater OFF + E0 OFF
  analogWrite(HEATER_MAIN_PIN, 0);
  analogWrite(HEATER_AUX_PIN,  0);
  timer4StopE0();
  digitalWrite(E0_ENABLE_PIN, HIGH);

  // Optical feedback for X speed (after ramp)
  static uint32_t lastControl = 0;
  if (X_AT_TARGET && (millis() - lastControl >= CONTROL_PERIOD_MS)) {
    lastControl = millis();

    bool trig = opticTriggeredDebounced();
    uint16_t next = X_CURRENT_US;

    if (trig) {
      if (next > X_MIN_US + TRIM_UP_STEP_US) next -= TRIM_UP_STEP_US;
      else next = X_MIN_US;
    } else {
      if (next < X_TARGET_US) {
        uint16_t diff = X_TARGET_US - next;
        uint16_t step = (diff > TRIM_BACK_STEP_US) ? TRIM_BACK_STEP_US : diff;
        next += step;
      } else if (next > X_TARGET_US) {
        uint16_t diff = next - X_TARGET_US;
        uint16_t step = (diff > TRIM_BACK_STEP_US) ? TRIM_BACK_STEP_US : diff;
        next -= step;
      }
    }

    if (next < X_MIN_US) next = X_MIN_US;
    if (next > X_MAX_US) next = X_MAX_US;

    applyXSpeed(next);
  }

  lcd.setCursor(0, 0);
  lcd.print("TEST OPTIC ");
  lcd.print(opticTriggeredDebounced() ? "TRIG" : "OK  ");

  lcd.setCursor(0, 1);
  lcd.print("X:");
  lcd.print(X_CURRENT_US);
  lcd.print("us      ");
}

void handleStateEmergency() {
  analogWrite(HEATER_MAIN_PIN, 0);
  analogWrite(HEATER_AUX_PIN, 0);
  stopAllMotors();

  static bool blinkState = false;
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink >= 500) {
    lastBlink = millis();
    blinkState = !blinkState;
  }

  if (blinkState) {
    lcd.setCursor(0, 0);
    lcd.print("!! EMERGENCY !! ");
    lcd.setCursor(0, 1);
    lcd.print("   ALL OFF      ");
  } else {
    lcd.clear();
  }
}

// =====================================================
// ================= SERIAL COMMANDS ===================
// =====================================================
String inputString = "";
bool stringComplete = false;

void sendStatus();

void processSerialCommand() {
  if (!stringComplete) return;

  inputString.trim();

  // ---- TEST OPTIC ----
  if (inputString == "START_TEST_OPTIC") {
    if (currentState != STATE_EMERGENCY) {
      enterState(STATE_TEST_OPTIC);
      Serial.println("OK:TEST_OPTIC_STARTED");
    } else {
      Serial.println("ERROR:EMERGENCY");
    }
  }
  else if (inputString == "STOP_TEST_OPTIC") {
    if (currentState == STATE_TEST_OPTIC) {
      enterState(STATE_IDLE);
      Serial.println("OK:TEST_OPTIC_STOPPED");
    } else {
      Serial.println("ERROR:NOT_IN_TEST");
    }
  }

  // START_AUTOTUNE
  else if (inputString == "START_AUTOTUNE") {
    if (currentState == STATE_IDLE) {
      autoTune.SetNoiseBand(2);
      autoTune.SetOutputStep(100);
      autoTune.SetLookbackSec(50);

      autoTuning = true;
      autoTuneStartTime = millis();

      enterState(STATE_AUTOTUNING);
      Serial.println("OK:AUTOTUNE_STARTED");
    } else {
      Serial.println("ERROR:NOT_IDLE");
    }
  }

  // CANCEL_AUTOTUNE
  else if (inputString == "CANCEL_AUTOTUNE") {
    if (currentState == STATE_AUTOTUNING) {
      autoTuning = false;
      analogWrite(HEATER_MAIN_PIN, 0);
      analogWrite(HEATER_AUX_PIN, 0);
      enterState(STATE_IDLE);
      Serial.println("OK:AUTOTUNE_CANCELLED");
    } else {
      Serial.println("ERROR:NOT_AUTOTUNING");
    }
  }

  // START_HEATER
  else if (inputString == "START_HEATER") {
    if (currentState == STATE_IDLE) {
      enterState(STATE_HEATING);
      Serial.println("OK:HEATER_STARTED");
    } else {
      Serial.println("ERROR:NOT_IDLE");
    }
  }

  // START_EXTRUSION
  else if (inputString == "START_EXTRUSION") {
    if (currentState == STATE_READY) {
      digitalWrite(E0_ENABLE_PIN, LOW);
      timer4StartE0();
      enterState(STATE_EXTRUDING);
      Serial.println("OK:EXTRUSION_STARTED");
    } else {
      Serial.println("ERROR:NOT_READY");
    }
  }

  // START_WINDING
  else if (inputString == "START_WINDING") {
    if (currentState == STATE_EXTRUDING) {
      digitalWrite(X_ENABLE_PIN, LOW);
      digitalWrite(Y_ENABLE_PIN, LOW);

      X_CURRENT_US = X_START_US;
      X_AT_TARGET  = false;

      timer1StartX();
      timer3StartY();

      enterState(STATE_FULL_AUTO);
      Serial.println("OK:WINDING_STARTED");
    } else {
      Serial.println("ERROR:NOT_EXTRUDING");
    }
  }

  // EMERGENCY_STOP
  else if (inputString == "EMERGENCY_STOP") {
    autoTuning = false;
    enterState(STATE_EMERGENCY);
    Serial.println("OK:EMERGENCY");
  }

  // RESET
  else if (inputString == "RESET") {
    if (currentState == STATE_EMERGENCY) {
      enterState(STATE_IDLE);
      Serial.println("OK:RESET");
    }
  }

  // GET_STATUS
  else if (inputString == "GET_STATUS") {
    sendStatus();
  }

  // SET_TARGET:XXX
  else if (inputString.startsWith("SET_TARGET:")) {
    double temp = inputString.substring(11).toFloat();
    if (temp >= 0 && temp <= MAX_TEMP) {
      targetTemp = temp;
      Serial.print("OK:TARGET:");
      Serial.println(targetTemp);
    }
  }

  // SET_PID:Kp,Ki,Kd
  else if (inputString.startsWith("SET_PID:")) {
    String params = inputString.substring(8);
    int c1 = params.indexOf(',');
    int c2 = params.indexOf(',', c1 + 1);

    if (c1 > 0 && c2 > 0) {
      Kp = params.substring(0, c1).toFloat();
      Ki = params.substring(c1 + 1, c2).toFloat();
      Kd = params.substring(c2 + 1).toFloat();
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.println("OK:PID_SET");
    }
  }

  inputString = "";
  stringComplete = false;
}

void sendStatus() {
  Serial.print("STATUS:");
  Serial.print(currentTemp, 1); Serial.print(",");
  Serial.print(targetTemp, 1); Serial.print(",");
  Serial.print((int)heaterPower); Serial.print(",");
  Serial.print(currentState); Serial.print(",");
  Serial.print(opticTriggeredDebounced() ? "1" : "0"); Serial.print(",");
  Serial.print(X_CURRENT_US); Serial.print(",");
  Serial.print(Kp, 3); Serial.print(",");
  Serial.print(Ki, 3); Serial.print(",");
  Serial.println(Kd, 3);
}

// =====================================================
// ======================= SETUP ========================
// =====================================================
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN,  OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  digitalWrite(X_DIR_PIN, HIGH);     // (you already flipped this)
  digitalWrite(X_ENABLE_PIN, HIGH);  // Disabled

  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN,  OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  yDirForward = false;
  digitalWrite(Y_DIR_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, HIGH); // Disabled

  pinMode(E0_STEP_PIN, OUTPUT);
  pinMode(E0_DIR_PIN,  OUTPUT);
  pinMode(E0_ENABLE_PIN, OUTPUT);
  digitalWrite(E0_DIR_PIN, LOW);
  digitalWrite(E0_ENABLE_PIN, HIGH); // Disabled

  // Optical sensor
  pinMode(OPTIC_PIN, INPUT_PULLUP);

  // Fans
  pinMode(FAN_MAIN_PIN, OUTPUT);
  
  digitalWrite(FAN_MAIN_PIN, HIGH);
  

  // Heaters
  pinMode(HEATER_MAIN_PIN, OUTPUT);
  pinMode(HEATER_AUX_PIN,  OUTPUT);
  analogWrite(HEATER_MAIN_PIN, 0);
  analogWrite(HEATER_AUX_PIN,  0);

  // LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("UI Controlled   ");
  lcd.setCursor(0, 1);
  lcd.print("with AutoTune   ");

  programStartMs = millis();

  // Temp check
  currentTemp = readTemperature();
  if (currentTemp < MIN_TEMP || currentTemp > 300.0) {
    enterState(STATE_EMERGENCY);
  }

  // PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(100);

  inputString.reserve(200);

  enterState(STATE_IDLE);

  Serial.println("READY");
  delay(500);
}

// =====================================================
// ======================== LOOP ========================
// =====================================================
void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      stringComplete = true;
    } else {
      inputString += c;
    }
  }

  processSerialCommand();

  if (millis() - lastTempUpdate >= 100) {
    lastTempUpdate = millis();
    currentTemp = readTemperature();

    if (currentTemp > MAX_TEMP || currentTemp < MIN_TEMP) {
      if (currentState != STATE_EMERGENCY) {
        enterState(STATE_EMERGENCY);
      }
    }
  }

  switch (currentState) {
    case STATE_IDLE:
      handleStateIdle();
      break;
    case STATE_AUTOTUNING:
      handleStateAutotuning();
      break;
    case STATE_HEATING:
      handleStateHeating();
      break;
    case STATE_READY:
      handleStateReady();
      break;
    case STATE_EXTRUDING:
      handleStateExtruding();
      break;
    case STATE_FULL_AUTO:
      handleStateFullAuto();
      break;
    case STATE_TEST_OPTIC:
      handleStateTestOptic();
      break;
    case STATE_EMERGENCY:
      handleStateEmergency();
      break;
  }

  if (millis() - lastSerialPrint >= SERIAL_PERIOD_MS) {
    lastSerialPrint = millis();
    sendStatus();
  }
}
