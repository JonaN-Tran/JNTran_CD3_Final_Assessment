| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

# _Programming 3 Project_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

## _Purpose_

This project focuses on the functionality of a keypad, MPU6050, and LCD screen connected to the ESP32-S3 and communicating through the I2C bus of the ESP32-S3. It is expected that the data read from the MPU6050 sensor will be displayed on the LCD screen when the keycode is correct.
This is the simplest buildable example. The example is used by command `idf.py create-project`
that copies the project to user specified path and set it's name. For more information follow the [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project)

## _Component Registries used_

- [esp-idf-lib/i2cdev v.2.0.8](https://components.espressif.com/components/esp-idf-lib/i2cdev/versions/2.0.8/readme)
- [iamflinks/i2c_lcd_pcf8574 v1.0.1](https://components.espressif.com/components/iamflinks/i2c_lcd_pcf8574/versions/1.0.1/api?language=en#function-lcd_set_backlight)

## _Components used_

- ESP32-S3
- 4x4 Keypad
- LCD1602A
- MPU6050
- 2x 220 Ohm resistors
- 4x 10k Ohm resistors
- 1x Red LED
- 1x Green LED
- 2x 2N7000 MOSFET

## _Folder contents_

The project **Programming 3 Project** [main](main) foldier contains:
- One source file in C language [main.c](main/main.c).
- One header file in C language [keypad_mpu6050.h](main/keypad_mpu6050.h).
- One implementation file in C language [keypad_mpu6050.h](main/keypad_mpu6050.c)

It also contains the component registry folders:
- [esp-idf-lib__i2cdev](managed_components/esp-idf-lib__i2cdev/)
- [iamflinks__i2c_lcd_pcf8574](managed_components/iamflinks__i2c_lcd_pcf8574/)

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both).