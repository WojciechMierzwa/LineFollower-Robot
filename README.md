# SWIM - Projekt 2024

## Project Overview

The SWIM project involves a Line Follower robot controlled by a custom application running on an STM32 microcontroller. The robot features four operational modes:

1. **Line Following:** Automatically tracks a line.
2. **Obstacle Detection:** Uses a distance sensor mounted on a servo to detect obstacles and navigate around them.
3. **Remote Control (Unrestricted):** Control the robot via Bluetooth without collision avoidance.
4. **Remote Control (Collision-Aware):** Control the robot via Bluetooth with automatic collision prevention.

## Project Requirements

### Hardware Requirements

- **STM32 Microcontroller NUCLEO-F302R8**
- **DC Motors**
- **Motor Driver Circuit**
- **Line Detection Sensors**
- **Bluetooth Module**
- **Servo Motor**
- **8-LED Display**
- **Battery Pack**

### Software Requirements

- **STM32CubeIDE** for project development.
- **STM32CubeMX** for microcontroller configuration.
- **Custom firmware** for robot control and sensor integration.



![Final Result]((https://github.com/WojciechMierzwa/LineFollower-Robot/tree/main/Photos/line_follower.jpg))

[Watch Video](https://youtu.be/trq2xTyngCU)

## Project Details

### Line Following Algorithm

The robot follows a line using infrared sensors placed at the front. The algorithm detects the line and adjusts motor speeds to keep the robot on track.


## Contribution Guidelines

1. **Fork the Repository** to create your own version of the project.
2. **Clone Your Fork** to your local machine.
3. **Create a New Branch** for your changes.
4. **Commit Your Changes** with clear messages.
5. **Push Your Branch** to your fork on GitHub.
6. **Open a Pull Request** with a description of your changes.

## Contact

For questions or contributions, please contact the project maintainer:

- **Wojciech Mierzwa**
- **Email:** [wojciech.piotr.mierzwa@gmail.com]


## Acknowledgements

- **STM32CubeIDE** and **STM32CubeMX** for development tools.
- **Contributors** and community members for their support and feedback.

