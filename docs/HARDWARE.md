# Hardware Notes

## Raspberry Pi 5 and Ubuntu 24.04 PWM

- Hardware PWM on Pi 5 with Ubuntu 24.04 (kernel 6.8.0) and the RP1 chip does **not** work.
  All attempts to use device tree overlays (e.g., `dtoverlay=pwm`, `dtparam=audio=off` in
  `/boot/firmware/config.txt`) failed to provide a working hardware PWM device.
- Only software PWM using GPIO (via libgpiod) is supported and tested on this platform.
- Ignore any instructions for editing `/boot/firmware/config.txt` for PWM on Pi 5 — these
  are not required and do not work.

## PWM Driver Selection

| Driver      | Interface | Notes                                               |
|-------------|-----------|-----------------------------------------------------|
| `pca9685`   | I2C       | Default. 16-channel, best compatibility across Pis. |
| `software`  | GPIO      | Recommended fallback for Pi 5 (uses libgpiod).      |
| `hardware`  | sysfs     | Requires kernel PWM support; does not work on Pi 5. |

## PCA9685 Wiring (I2C)

| PCA9685 Pin | Raspberry Pi Pin | Notes        |
|-------------|------------------|--------------|
| VCC         | Pin 1 (3.3V)     | Power        |
| GND         | Pin 6 (GND)      | Ground       |
| SDA         | Pin 3 (GPIO 2)   | I2C Data     |
| SCL         | Pin 5 (GPIO 3)   | I2C Clock    |

Connect the servo signal wire to one of the PCA9685 PWM output channels (default: channel 15).
Power the servo from an external 5 V supply via the PCA9685 V+ rail.

Verify the device is visible on the bus:
```bash
sudo i2cdetect -y 1
# Expected: device at address 0x40
```

## Software PWM GPIO Wiring (Pi 5 fallback)

| GPIO Pin | Physical Pin | Description     |
|----------|-------------|-----------------|
| GPIO 18  | Pin 12      | PWM Signal (default) |
| GND      | Any GND pin | Ground          |

The software driver uses `/dev/gpiochip4` on Pi 5.  A real-time scheduling hint is attempted
automatically at startup to improve timing accuracy.
