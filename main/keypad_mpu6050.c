#include "keypad_mpu6050.h"  // My header file

#include <inttypes.h>
#include <stdio.h>
#include <string.h>  // String library

#include "driver/gpio.h"  // GPIO library
#include "driver/i2c.h"   // I2C library
#include "esp_log.h"      // ESP_LOG library
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_lcd_pcf8574.h"  // LCD1602A LCD Display library

// Declaring GPIO pins and access code for keypad
char accessCode[] = "1234";  // Keypad access code
char userCode[4];            // User's input access code
int rowPins[ROWS] = {36, 35, 0, 45};
int colPins[COLS] = {48, 47, 21, 20};
// Declaring values for keypad buttons in 2D array
char keys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

/*
 * Function: setup_gpio
 * Purpose: Sets up the GPIO pins for all connected components
 * Parameters: none
 * Return: void
 */
void setup_gpio() {
    // Setting LED pins to OUTPUT
    gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);

    // Setting up pins for keypad
    // Setting row pins as output on constant active
    for (int i = 0; i < ROWS; i++) {
        gpio_set_direction(rowPins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(rowPins[i], 1);
    }
    // Setting column pins as input in pullup mode
    for (int i = 0; i < COLS; i++) {
        gpio_set_direction(colPins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(colPins[i], GPIO_PULLUP_ONLY);
    }
}

/*
 * Function: readKeyPadPress
 * Purpose: Scans which button was pressed and returns the value of the button using blocking polling
 * Parameters: array of keypad values, array of column pins of keypad, array of row pins of keypad
 * Return: char
 */
char readKeyPadPress(char _keys[4][4], int _colPins[], int _rowPins[]) {
    int scanVal;
    // Setting default return value when no key pressed
    char retCode = '\0';

    for (int i = 0; i < ROWS; i++) {
        // One's complements the row specified ROW (sets 1 to 0)
        scanVal = ~(1 << i);

        for (int j = 0; j < ROWS; j++) {
            // Sets level of ROW to the value of the read bit
            gpio_set_level(_rowPins[j], (scanVal >> j) & 1);
        }
        vTaskDelay(pdMS_TO_TICKS(5));

        for (int k = 0; k < COLS; k++) {
            // Checks if pin on specified COL is 0 upon button press
            if (gpio_get_level(_colPins[k]) == 0) {
                while (gpio_get_level(_colPins[k]) == 0) {
                    // Prevents additional input when a user holds down a keypad button
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                // Assigns value of button pressed to retCode
                retCode = _keys[i][k];
            }
        }
    }
    return retCode;
}

/**
 * Function: turn_on_LED
 * Purpose: Turns on specified LED for X amount of milliseconds
 * Paramters: LED pin, milliseconds
 * Return: void
 */
void turn_on_LED(int led, int ms) {
    gpio_set_level(led, 1);
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(led, 0);
}

/**
 * Function: i2c_master_init
 * Purpose: Sets up I2C master
 * Paramters: none
 * Return: esp_err_t (Success or Failure)
 */
esp_err_t I2C_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;

    return i2c_driver_install(I2C_MASTER_NUM,
                              conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE,
                              0);
}

/**
 * Function: i2c_send_data_block
 * Purpose: Sends a block of data over the I2C bus to a peripheral
 * Paramters: Address of device, data to send, length of data
 * Return: esp_err_t (Success or Failure)
 */
esp_err_t i2c_send_data_block(uint8_t deviceAddr, uint8_t regAddr, uint8_t* data) {
    uint8_t buf[2] = {regAddr, data};
    size_t length = sizeof(buf);
    return i2c_master_write_to_device(I2C_MASTER_NUM, deviceAddr, buf, length, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/**
 * Function: i2c_read_bytes
 * Purpose: Reads a block of data over the I2C bus to a peripheral
 * Paramters: Address of device, Start of register, buffer, length of data
 * Return: esp_err_t (Success or Failure)
 */
esp_err_t i2c_read_bytes(uint8_t deviceAddr, uint8_t startReg, uint8_t* buffer, size_t length) {
    return i2c_master_write_read_device(I2C_MASTER_NUM,
                                        deviceAddr,
                                        &startReg,
                                        1,
                                        buffer,
                                        length,
                                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/**
 * Function: display_mpu6050_data
 * Purpose: Displays the data read from MPU6050
 * Paramters: LCD pointer, number of iterations (for 'for' loop), register type (0 = Accel, 1 = Temp, 2 = Gyro)
 * Return: void
 */
void display_mpu6050_data(i2c_lcd_pcf8574_handle_t lcd, int iterations, int regist_type) {
    // Wake up MPU6050 (clear sleep bit)
    esp_err_t ret_send = i2c_send_data_block(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    if (ret_send == ESP_OK) {
        ESP_LOGI(TAG_MPU, "SEND SUCCESSFUL.");
        for (int i = 0; i < iterations; i++) {
            uint8_t data[14];
            esp_err_t ret_read = i2c_read_bytes(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, sizeof(data));

            // Display X, Y, Z axis of Acceleration and Gyroscope
            if (ret_read == ESP_OK) {
            ESP_LOGI(TAG_MPU, "READ SUCCESSFUL.");
                // Declaring buffer array to to combine int16_t and float numbers with string in 'sniprintf()'
                char buffer1[16];
                char buffer2[16];
                char buffer3[16];

                // Display data for acceleration if register type == 0
                if (regist_type == 0) {
                    int16_t ax = (data[0] << 8) | data[1];  // Acceleration X-axis
                    int16_t ay = (data[2] << 8) | data[3];  // Acceleration Y-axis
                    int16_t az = (data[4] << 8) | data[5];  // Acceleration Z-axis

                    print_to_lcd(lcd, 0, 0, "Accel");

                    // Print acceleration X-axis to LCD
                    sniprintf(buffer1, sizeof(buffer1), "X=%d", ax);
                    print_to_lcd(lcd, 8, 0, buffer1);

                    // Print acceleration Y-axis to LCD
                    sniprintf(buffer2, sizeof(buffer2), "Y=%d", ay);
                    print_to_lcd(lcd, 0, 1, buffer2);

                    // Print acceleration Z-axis to LCD
                    sniprintf(buffer3, sizeof(buffer3), "Z=%d", az);
                    print_to_lcd(lcd, 8, 1, buffer3);
                }
                // Display data for temperature if register type == 1
                else if (regist_type == 1) {
                    int16_t temp_out = (data[6]) << 8 | data[7];
                    float temp_in_C = (float)temp_out / 340 + 36.53;  // Conversion of temp_out to temperature in Celsius

                    // Store float temperature data in buffer array
                    char buffer4[50];
                    print_to_lcd(lcd, 0, 0, "Temp");

                    // Print temperature in Celsius to LCD
                    sniprintf(buffer4, sizeof(buffer4), "%.2f C", temp_in_C);
                    print_to_lcd(lcd, 8, 0, buffer4);
                }
                // Display data for gyroscope if register type == 2
                else if (regist_type == 2) {
                    int16_t gx = (data[8] << 8) | data[9];    // Gyroscope X-axis
                    int16_t gy = (data[10] << 8) | data[11];  // Gyroscope Y-axis
                    int16_t gz = (data[12] << 8) | data[13];  // Gyroscope Z-axis

                    print_to_lcd(lcd, 0, 0, "Gyro");

                    // Print gyroscope X-axis to LCD
                    sniprintf(buffer1, sizeof(buffer1), "X=%d", gx);
                    print_to_lcd(lcd, 8, 0, buffer1);

                    // Print gyroscope Y-axis to LCD
                    sniprintf(buffer2, sizeof(buffer2), "Y=%d", gy);
                    print_to_lcd(lcd, 0, 1, buffer2);

                    // Print gyroscope Z-axis to LCD
                    sniprintf(buffer3, sizeof(buffer3), "Z=%d", gz);
                    print_to_lcd(lcd, 8, 1, buffer3);
                }
            } else {
                ESP_LOGE(TAG_MPU, "Failed to read data: %s", esp_err_to_name(ret_read));
            }
            vTaskDelay(pdMS_TO_TICKS(3000 / iterations));
        }
    } else {
        ESP_LOGE(TAG_MPU, "Failed to send data: %s", esp_err_to_name(ret_send));
    }
}

/**
 * Function: print_to_lcd
 * Purpose: Sets the cursor position then prints message on LCD
 * Paramters: LCD pointer, cursor column, cursor row, lcd message
 * Return: none
 */
void print_to_lcd(i2c_lcd_pcf8574_handle_t lcd, int col, int row, const char* msg) {
    lcd_set_cursor(&lcd, col, row);
    lcd_print(&lcd, msg);
}