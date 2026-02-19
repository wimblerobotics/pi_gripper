# pi_gripper

ROS 2 node for controlling a servo gripper via PWM on Raspberry Pi 5.

Three PWM back-ends are supported:

| Driver     | Interface | Notes                                               |
|------------|-----------|-----------------------------------------------------|
| `pca9685`  | I2C       | **Default.** 16-channel PCA9685 board, best compatibility. |
| `software` | GPIO      | Recommended fallback for Pi 5; uses libgpiod.       |
| `hardware` | sysfs     | Kernel PWM via `/sys/class/pwm`; does **not** work on Pi 5. |

See [docs/HARDWARE.md](docs/HARDWARE.md) for wiring diagrams and Pi 5 PWM notes.

---

## Prerequisites

| Dependency | Purpose |
|------------|---------|
| ROS 2 Jazzy | runtime |
| `python3-smbus2` or `pip install smbus2` | PCA9685 I2C communication |
| `python3-gpiod` or `pip install gpiod` | Software PWM (GPIO) |
| `i2c-tools` | verify I2C wiring (`i2cdetect`) |

Enable I2C on Raspberry Pi (add to `/boot/firmware/config.txt`):
```
dtparam=i2c_arm=on
dtparam=i2c1=on
```

Add your user to the required groups:
```bash
sudo usermod -aG i2c,gpio,dialout $USER
# log out and back in
```

---

## Building

```bash
mkdir -p ~/sigyn_vision_ws/src
cd ~/sigyn_vision_ws/src
git clone <repo-url> pi_gripper
cd ~/sigyn_vision_ws
colcon build --packages-select pi_gripper
source install/setup.bash
```

---

## Running

**PCA9685 (default):**
```bash
ros2 launch pi_gripper pi_gripper.launch.py
```

**PCA9685 with custom I2C address / channel:**
```bash
ros2 launch pi_gripper pi_gripper.launch.py i2c_address:=64 pwm_channel:=15
```

**Software PWM on GPIO 18 (Pi 5):**
```bash
sudo ros2 launch pi_gripper pi_gripper.launch.py pwm_driver_type:=software gpio_pin:=18
```

**Hardware PWM via sysfs (Pi 4 only):**
```bash
sudo ros2 launch pi_gripper pi_gripper.launch.py pwm_driver_type:=hardware pwm_chip:=1 pwm_channel:=0
```

---

## Launch Arguments

| Argument          | Default    | Description |
|-------------------|------------|-------------|
| `pwm_driver_type` | `pca9685`  | Driver back-end: `pca9685`, `software`, or `hardware` |
| `i2c_address`     | `64`       | I2C address of PCA9685 in decimal (0x40 = 64) |
| `i2c_bus`         | `1`        | I2C bus number |
| `pwm_frequency`   | `50`       | PWM frequency in Hz (50 Hz recommended for servos) |
| `pwm_channel`     | `15`       | PWM channel: 0–15 for PCA9685; 0 for hardware/software |
| `gpio_pin`        | `18`       | GPIO line for software PWM (physical Pin 12) |
| `pwm_chip`        | `1`        | Kernel PWM chip index for hardware PWM |
| `use_software_pwm`| `false`    | Legacy backward-compatibility flag |

---

## ROS Interface

### Subscribed Topics

| Topic                   | Type                          | Description |
|-------------------------|-------------------------------|-------------|
| `/cmd_vel_pi_gripper`   | `geometry_msgs/msg/Twist`     | Gripper command |

**`linear.x` mapping:**

| `linear.x` | Pulse width | Gripper position |
|------------|-------------|-----------------|
| `-1000`    | 1.1 ms      | Fully open       |
| `0`        | ~1.55 ms    | Centre           |
| `+1000`    | 2.0 ms      | Fully closed     |

Values outside ±1000 are clamped. Intermediate values are linearly interpolated.

---

## Testing Manually

```bash
# Open gripper
ros2 topic pub --once /cmd_vel_pi_gripper geometry_msgs/msg/Twist \
  "{linear: {x: -1000.0}}"

# Centre
ros2 topic pub --once /cmd_vel_pi_gripper geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}}"

# Close gripper
ros2 topic pub --once /cmd_vel_pi_gripper geometry_msgs/msg/Twist \
  "{linear: {x: 1000.0}}"
```

---

## Running as a systemd Service

A ready-to-use service file is provided (`systemd/pi-gripper.service`).

```bash
# Install and enable (once, at machine setup time)
sudo bash scripts/setup_service.sh

# Check status
sudo systemctl status pi-gripper.service
```

The service requires `rmw_cyclonedds_cpp` and a CycloneDDS configuration at
`/etc/cyclonedds.xml`. Adjust `systemd/pi-gripper.service` if your environment
differs.

---

## License

Apache-2.0 — see [LICENSE](LICENSE).

