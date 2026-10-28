# kvn_display

Shows the rover's status on a small monochrome display. Display controllers, fonts and drawing come
from [u8g2](https://github.com/olikraus/u8g2), so every display u8g2 supports over I2C or 4-wire SPI
works without new code: SSD1306, SH1106, SSD1309, ST7565, ST7920, UC1701 and many more. The rover's
display is the 0.49" 64x32 white SSD1306 at I2C address `0x3C`.

```bash
ros2 launch kvn_display kvn_display.launch.py           # uses config/kvn_display.yaml
ros2 run kvn_display kvn_display_node --ros-args -p driver:=sh1106_i2c_128x64_noname
```

On the rover it is started from `kvn_robot_bringup/launch/robot_base.launch.py` with
`use_display:=true` (systemd: `KVN_DISPLAY_ENABLED=true` in `/etc/kvn/kvn-ros.env`). The driver
and its bus then come from the `display` section of `kvn_robot_bringup/config/robot.yaml`.

## Choosing the driver

`driver` is the name of a u8g2 full-buffer setup function without `u8g2_Setup_` and `_f`, from
[u8g2's list](https://github.com/olikraus/u8g2/wiki/u8g2setupc): `ssd1306_i2c_64x32_1f` is
`u8g2_Setup_ssd1306_i2c_64x32_1f_f`. The panel size comes with the driver. Names with `_i2c_`
use I2C; all others use 4-wire SPI.

Several drivers can fit one controller and size (the `_noname`, `_1f`, ... variants differ in
column offset and initialisation). If the picture is shifted or garbled, try the other variants
of the same size.

| Bus | Wiring | Parameters |
|---|---|---|
| I2C | SDA, SCL | `i2c_device` (`/dev/i2c-1`), `i2c_address` (usually `0x3C` or `0x3D`) |
| SPI | MOSI, SCLK, CS (by the kernel), D/C and reset (GPIO) | `spi_device`, `spi_speed_hz` (0: the display's maximum), `gpio_chip`, `dc_line`, `reset_line` (-1: none) |

On the Raspberry Pi:

- I2C: enable it (`dtparam=i2c_arm=on`), add the service user to the `i2c` group, and check the
  address with `i2cdetect -y 1`.
- SPI: keep the display off SPI0, which the IMU (`/dev/spidev0.0`) uses, so frame transfers never
  delay IMU reads: enable SPI1 (`dtoverlay=spi1-1cs`, `/dev/spidev1.0`). Add the service user to
  the `spi` and `gpio` groups. `dc_line` and `reset_line` are line offsets on `gpio_chip`, which
  for the header pins are the BCM GPIO numbers (`gpioinfo` lists them).

## Screen

The screen has 4 lines; the font is the largest of the X11 fixed fonts (5x7 up to 10x20) that fits
a quarter of the panel height. On the 64x32 display that is 5x7, 12 characters per line.

| Line | Shows | Example |
|---|---|---|
| 1 | `/robot_status`, inverted (lit background) for `FLT`, `WRN` or no status for `status_timeout_s` (`NO STATUS`) | `FLT:BATT` |
| 2 | `/battery_state` voltage and percentage (when valid), `--.-V` when stale | `14.8V 72%` |
| 3 | `/orchestrator/mode` as a short label, empty without the orchestrator | `ASSIST` |
| 4 | `/cmd_vel_source`: who drives `/cmd_vel` | `CMD AUTO` |

Text longer than the line is cut at the panel edge. The screen is redrawn every
`update_period_s` and sent only when a line changes (the I2C bus is shared with the INA3221
power monitor).

If the display is missing or a write fails, the node logs a warning and reopens it on the next
update, so a loose connector does not stop it. An unknown driver or an invalid bus setting stops
the node at startup. When the node exits it puts the display into power save.

## Parameters

| Name | Default | Description |
|---|---|---|
| `driver` | `ssd1306_i2c_64x32_1f` | u8g2 driver, see above |
| `rotation` | `0` | 0, 90, 180 or 270 degrees |
| `contrast` | `-1` | 0..255, -1 keeps the driver's default |
| `i2c_device`, `i2c_address` | `/dev/i2c-1`, `60` (`0x3C`) | I2C drivers |
| `spi_device`, `spi_speed_hz` | `/dev/spidev1.0`, `0` | SPI drivers |
| `gpio_chip`, `dc_line`, `reset_line` | `/dev/gpiochip0`, `-1`, `-1` | SPI drivers; `dc_line` is required |
| `status_topic` | `/robot_status` | `std_msgs/String` from `kvn_status` |
| `battery_topic` | `/battery_state` | `sensor_msgs/BatteryState` |
| `mode_topic` | `/orchestrator/mode` | `std_msgs/String`, latched |
| `cmd_source_topic` | `/cmd_vel_source` | `std_msgs/String`, latched, no timeout |
| `status_timeout_s`, `battery_timeout_s`, `mode_timeout_s` | `3.0` | Older inputs are shown as missing |
| `update_period_s` | `0.5` | Redraw period |

## Build

CMake downloads u8g2 (pinned release of
[U8g2_Arduino](https://github.com/olikraus/U8g2_Arduino), the C library only, with a checksum) and
builds it into the node, with only the five fonts the screen uses (`cmake/extract_u8g2_fonts.py`)
instead of u8g2's 40 MB font file. The driver table is generated from `u8g2.h`, so updating u8g2
brings its new displays. For an offline build, pass
`--cmake-args -DFETCHCONTENT_SOURCE_DIR_U8G2=<U8g2_Arduino checkout>` to colcon. SPI needs
libgpiod 2 (`libgpiod-dev`, installed by rosdep).

## Code

| File | Role |
|---|---|
| [screen.hpp](include/kvn_display/screen.hpp) | What the lines say, and drawing them with u8g2 |
| [display.hpp](include/kvn_display/display.hpp) | u8g2 display: driver lookup, initialisation, power save |
| [transport.hpp](include/kvn_display/transport.hpp) | Byte transport for u8g2: Linux i2c-dev, spidev + libgpiod |
| [display_node.hpp](include/kvn_display/display_node.hpp) | ROS node: inputs, staleness, reconnecting |

The tests run real u8g2 drivers against a recording transport (`test/recording_transport.hpp`):
the bytes sent over I2C, the D/C and reset lines over SPI, and the pixels drawn. None need the
hardware.
