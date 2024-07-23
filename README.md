# servo_publisher

## Overview

The `servo_publisher` ROS 2 package controls a servo motor using a Raspberry Pi and publishes the servo's angle to a ROS 2 topic. The servo's rotation is controlled via PWM signals, and the package continuously rotates the servo smoothly between specified angles while publishing the angle values to a ROS 2 topic.

## Requirements

- **ROS 2** (any compatible version, e.g., Foxy, Galactic, or Humble)
- **Python 3**
- **gpiozero** library
- **pigpio** library
- **Raspberry Pi** with a compatible servo and GPIO pin setup

## Installation

1. **Set up your ROS 2 workspace** (if not already done):

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

2. **Clone the repository**:

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/FarrukhAijaz/servo_publisher.git
    ```

3. **Install dependencies**:

    Make sure you have `gpiozero` and `pigpio` installed on your Raspberry Pi:

    ```bash
    sudo apt-get update
    sudo apt-get install python3-gpiozero pigpio
    ```

4. **Build the workspace**:

    ```bash
    cd ~/ros2_ws
    colcon build
    ```

5. **Source the workspace**:

    ```bash
    source install/setup.bash
    ```

## Usage

1. **Start the pigpio daemon** (this is necessary for gpiozero to work with the PiGPIOFactory):

    ```bash
    sudo pigpiod
    ```

2. **Run the ROS 2 node**:

    ```bash
    ros2 run servo_publisher servo_publisher
    ```

## Configuration

The `servo_publisher` package is configured with the following parameters:

- **MIN_PULSE_WIDTH**: Minimum pulse width for the servo (500 µs).
- **MAX_PULSE_WIDTH**: Maximum pulse width for the servo (2500 µs).
- **SERVO_PIN**: GPIO pin connected to the servo (default is 12).
- **ROTATION_SPEED**: Delay in seconds between angle updates (default is 0.1).
- **ANGLE**: Reference value to adjust the servo at a planar surface (default should be 0 degrees).
- **OFFSET**: Offset angle for the servo to move between ± X degrees (default is 20 degrees).

Adjust these parameters in the `rotation.py` file as needed.

## Troubleshooting

- **No movement**: Ensure that the pigpio daemon is running (`sudo pigpiod`).
- **No ROS 2 messages**: Verify that your ROS 2 environment is properly sourced and that the `servo_publisher` node is running.

## License

This package is licensed under the GNU General Public License (GPL). See the [LICENSE](LICENSE) file for details.

## Contact

For any issues or questions, please contact Farrukh Aijaz at [farrukhajaz1@gmail.com](mailto:farrukhajaz1@gmail.com).
