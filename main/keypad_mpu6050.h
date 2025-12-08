#ifndef KEYPAD_MPU6050_H
#define KEYPAD_MPU6050_H

#include "driver/i2c.h"
#include <inttypes.h>
#include "i2c_lcd_pcf8574.h"

// Declaring size for appropriate arrays
#define ROWS 4
#define COLS 4

// Declaring GPIO pins for LEDs
#define RED_LED 1
#define GREEN_LED 2

// Declaring arrays for keypad
extern char accessCode[];
extern char userCode[4];
extern int rowPins[ROWS];
extern int colPins[COLS];
extern char keys[ROWS][COLS];

// Declaring variables for I2C
#define I2C_MASTER_SCL_IO 37  // SCL pin for ESP32-S3
#define I2C_MASTER_SDA_IO 38  // SDA pin for ESP32-S3
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000  // 400 kHz (Fast mode)
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_TIMEOUT_MS 1000

// Declaring variables for MPU6050 addresses
#define MPU6050_ADDR 0x68  // AD0 = GND
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B  // Register address to begin with
static const char* TAG_I2C = "I2C";
static const char* TAG_MPU = "MPU6050";
static const char* TAG_KEYPAD = "KEYPAD";
static const char* TAG_LCD = "LCD1602";

// Declaring variables for LCD1602
#define LCD1602_ADDR 0x27  // I2C Address of LCD
#define LCD_COLS 16        // Number of columns in the LCD
#define LCD_ROWS 2         // Number of rows in the LCD

// Declaring functions
void setup_gpio();
char readKeyPadPress(char _keys[4][4], int _colPins[], int _rowPins[]);
void turn_on_LED(int pin, int ms);
esp_err_t I2C_master_init();
esp_err_t i2c_send_data_block(uint8_t deviceAddr, uint8_t regAddr, uint8_t* data);
esp_err_t i2c_read_bytes(uint8_t deviceAddr, uint8_t startReg, uint8_t* buffer, size_t length);
void display_mpu6050_data(i2c_lcd_pcf8574_handle_t lcd, int iterations, int regist_type);
void print_to_lcd(i2c_lcd_pcf8574_handle_t lcd, int col, int row, const char* msg);
#endif