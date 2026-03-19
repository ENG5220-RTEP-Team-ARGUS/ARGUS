<p align="center">
  <img src="https://github.com/user-attachments/assets/be912344-c201-4a3f-af12-ca498a7208d0" width="550"/>
</p>

<h1 align="center">A.R.G.U.S - "Detect. Decide. Stop."</h1>

## Table of Contents
- [Overview](#overview)
- [Real-World Use Case](#real-world-use-case)
- [System Architecture](#system-architecture)
- [Bill of Materials (BOM)](#bill-of-materials-bom)
- [Installation & Setup](#installation--setup)
- [Building the Project](#building-the-project)
- [Running the System](#running-the-system)
- [Testing](#testing)
- [Project Structure](#project-structure)
- [Core Components](#core-components)
- [Real-Time Design & Latency](#real-time-design--latency)
- [Documentation](#documentation)
- [Social Media & PR](#social-media--pr)
- [Authors & Contributions](#authors--contributions)
- [Acknowledgements](#acknowledgements)
- [License](#license)
- [Future Work](#future-work)

---

## Overview
<p align="justify"> A.R.G.U.S (Adaptive Real-Time Guardian for Unsafe Situations) is a real-time, vision-based safety system for robotic manipulators, designed for high-risk environments such as surgical robotics and industrial automation. It continuously monitors the workspace - particularly during critical operations like instrument exchange - where unexpected motion can cause damage or injury. By analysing visual input under strict latency constraints, A.R.G.U.S. detects deviations from expected conditions and immediately triggers fail-safe interventions (e.g. hard stops) using event-driven control. This ensures deterministic, reliable interruption of motion, preventing accidents before they occur.</p>

---

## Real-World Use Case
A.R.G.U.S is designed for **safety-critical environments**, including:

- Surgical robotics (instrument exchange safety)
- Industrial robotic arms (collision prevention)
- Human-robot collaboration systems  

During operations such as **tool exchange**, unexpected motion can cause injury or system failure. A.R.G.U.S ensures the robot only operates under valid conditions, stopping instantly when anomalies occur.

---

## System Architecture
> *(Insert diagram from `/docs/architecture`)*

- Camera input → event stream  
- Region of Interest (ROI) validation  
- Guardian state machine  
- Fail-safe controller (stop signal)

---

## Bill of Materials (BOM)

### Controller
| Component | Quantity | Cost (£) |
|----------|---------|----------|
| Raspberry Pi (Model TBD) | 1 | TBD |

### Sensors & Vision
| Component | Quantity | Cost (£) |
|----------|---------|----------|
| Camera Module | 1 | TBD |

### Additional Components
| Component | Quantity | Cost (£) |
|----------|---------|----------|
| Robotic Arm (Test Platform) | 1 | TBD |

**Total Cost:** TBD

---

## Installation & Setup

### Requirements
- Linux (Raspberry Pi OS)
- C++17+
- CMake ≥ 3.10
- OpenCV (if used)
- libgpiod

### Install Dependencies
```bash
sudo apt update
sudo apt install cmake libgpiod-dev
```
---

## Building the Project

```bash
git clone https://github.com/YOUR_REPO.git --recursive
cd ARGUS

cmake .
make
```

---

## Running the System

```bash
./argus_main
```

---

## Testing

```bash
make test
```

## Project Structure

```
config/              # Configuration files
docs/architecture/   # System diagrams
include/             # Header files
src/                 # Core implementation
tests/               # Unit tests
```

---

## Core Components

### Vision Processor
- Processes camera frames as events
- Extracts ROI and validates signal

### Guardian State Machine
- Encodes `SAFE` / `UNSAFE` states
- Handles transitions deterministically

### Motion Controller
- Issues stop signals
- Interfaces with robotic system

---

## Real-Time Design & Latency

- Event-driven architecture (no polling)
- Deterministic response times
- Frame-based processing pipeline

> ⚠️ *Add measured latency results here*

---

## Documentation

### Wiki
[https://github.com/ENG5220-RTEP-Team-ARGUS/ARGUS/wiki](https://github.com/ENG5220-RTEP-Team-ARGUS/ARGUS.wiki.git)

### Doxygen
All public classes and interfaces are documented using Doxygen.

---

## Social Media & PR

-  [Instagram](https://www.instagram.com/argus102026/)
-  [YouTube](https://www.youtube.com/@argus-w3g)
-  [LinkedIn](https://www.linkedin.com/company/a-r-g-u-s)

### Platform Performance Summary
---

## Authors & Contributions

| Name | Contribution |
|------|-------------|
| Member 1 |  |
| Member 2 |  |
| Member 3 |  |
| Member 4 |  |
| Member 5 |  |

---

## Acknowledgements

- Course lecturers
- Lab technicians
- Funding/support sources

---

## License

*Specify license here.*

---

## Future Work

- [ ] Multi-camera integration
- [ ] Advanced risk prediction
- [ ] Full robotic system integration
- [ ] Improved latency optimisation

https://www.instagram.com/argus102026/
https://www.youtube.com/@argus-w3g
https://www.linkedin.com/company/a-r-g-u-s

