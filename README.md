# noamnhFOC

This repository implements Field Oriented Control (FOC) using ESP-IDF in C++. The project leverages the ESP32’s MCPWM peripheral to enable high-performance motor control for robotics, automation, and other embedded applications.


## Branches

| Branch Name             | Description/Purpose           |
|-------------------------|-------------------------------|
| main                    | Main development branch       |
| roadmap/run_smooth_test | Roadmap and smooth test       |
| version/ledc            | LEDC version branch           |
| version/mcpwm           | MCPWM version branch          |


## Features

- Written in modern C++
- Uses ESP-IDF framework
- MCPWM-based FOC motor control
## Roadmap

- [x] Sensor Read
- [x] MCPWM Initialization
- [x] Open Loop Control
- [ ] Current Reading (In Progress)
- [ ] Calibration and Alignment (To Be Done)
- [ ] Closed Loop - Current (To Be Done)
- [ ] Closed Loop - Velocity (To Be Done)
- [ ] Closed Loop - Position (To Be Done)
- [ ] Implementing Observer (To Be Done)
- [ ] Interpolating Trajectory (To Be Done)
- [ ] Impedance Control (To Be Done)

## Getting Started

1. Clone this repository:
   ```bash
   git clone https://github.com/noamnh/noamnhFOC.git
