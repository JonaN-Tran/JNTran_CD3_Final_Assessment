#include <string.h>  // String library

#include "esp_log.h"
#include "keypad_mpu6050.h"  // My header file

// Declaring pointer for LCD
i2c_lcd_pcf8574_handle_t lcd;

void app_main(void) {
    // Setup GPIO
    setup_gpio();
    // Setup I2C master
    ESP_ERROR_CHECK(I2C_master_init());
    ESP_LOGI(TAG_I2C, "I2C initialized");
    // Setup LCD
    lcd_init(&lcd, LCD1602_ADDR, I2C_MASTER_NUM);
    lcd_begin(&lcd, LCD_COLS, LCD_ROWS);

    // Turn on the backlight at 100% brightness (0-255)
    lcd_set_backlight(&lcd, 255);

    while (1) {
        // CLear LCD from start
        lcd_clear(&lcd);

        // Initialize loop counter
        int i = 0;

        // Print to LCD on first row
        print_to_lcd(lcd, 0, 0, "Enter Code: ");
        while (i < 5) {
            ESP_LOGI(TAG_KEYPAD, "System running...");
            // If 'i' is the last index of the 'userCode' string, add '\0' to complete the full string
            if (i == 4) {
                userCode[i] = '\0';
                i++;
            } else {
                // Read user input from keypad
                char key = readKeyPadPress(keys, colPins, rowPins);
                // if user input is detected
                if (key != '\0') {
                    // Assign pressed keypad value to appropriate index of string 'userCode'
                    userCode[i] = key;
                    // Set cursor after "Enter Code:" text to add "*" when user presses a keypad
                    size_t length = strlen("Enter Code: ");
                    // Print user's input to LCD
                    print_to_lcd(lcd, i + length, 0, "*");
                    i++;
                }
            }
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        lcd_clear(&lcd);

        // Compare user's input (userCode[]) to access code (accessCode[])
        // Grant access
        if (strcmp(userCode, accessCode) == 0) {
            // Print confirmation of correct code
            print_to_lcd(lcd, 0, 0, "ACCESS GRANTED!");

            // Blinks green LED on granted access for 5s
            turn_on_LED(GREEN_LED, 5000);

            // Declaring num of iterations to display data
            int num_iterations = 6;

            // Set which register to display first: 0 = Accel, 1 = Temp, 2 = Gyro
            int register_type = 0;

            // Loop X times to display acceleration, gyroscope, then temperature
            // data_iterations = (num of data types) * (num of times to iterate 'num of data types')
            int data_iterations = 3 * 2;

            for (int j = 0; j < data_iterations; j++) {
                lcd_clear(&lcd);

                // Display MPU6050 data to LCD
                display_mpu6050_data(lcd, num_iterations, register_type);

                // Reset data type to first one if it reaches the last one
                if (register_type >= 2) {
                    register_type = 0;
                }
                // Set the data type to the next one if not at the last data type
                else {
                    register_type++;
                }
            }

            // Display messsage for program reset to "Power-Up Sequence" on LCD
            lcd_clear(&lcd);
            print_to_lcd(lcd, 0, 0, "Returning to");
            print_to_lcd(lcd, 0, 1, "PWR-up sequence");

            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        // Deny access
        else {
            // Print confirmation of correct code
            print_to_lcd(lcd, 0, 0, "ACCESS DENIED!");

            // Blinks red LED on granted access for 2s
            turn_on_LED(RED_LED, 2000);
        }
        // clear_lcd();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}