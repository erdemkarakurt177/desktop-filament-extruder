## Desktop Plastic Filament Recycling & Extrusion System

This repository contains the firmware and documentation of a desktop-scale plastic filament recycling and extrusion system developed as a Bachelor's graduation project at Middle East Technical University (METU).

The system converts waste thermoplastics into reusable 3D-printing filament using a single-screw extrusion mechanism with PID-controlled heating. The user interface is PC-based (commands and monitoring via USB Serial).

### Key Features
- Arduino-based embedded control system (Arduino Mega + RAMPS 1.4)
- PID temperature control (auto-tuning supported)
- Multi-stepper motor coordination (extrusion, spool winding, traverse)
- Emphasis on energy efficiency and operational safety

### Repository Structure
- `firmware/` : Arduino firmware (.ino)
- `documentation/` : System overview diagram and project poster

### Documentation
- System overview diagram: `documentation/system_overview.png`
- Technical poster: `documentation/Project_poster.pdf`
