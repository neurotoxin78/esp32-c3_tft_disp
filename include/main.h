#include <Arduino.h>
#define LED GPIO_NUM_12

#define GPIO_UP_PIN 8
#define GPIO_RT_PIN 9
#define GPIO_DN_PIN 13
#define GPIO_LT_PIN 5
#define GPIO_CR_PIN 4

#define VRX_PIN GPIO_NUM_0 // ESP32 pin GIOP36 (ADC0) connected to VRX pin
#define VRY_PIN GPIO_NUM_1 // ESP32 pin GIOP39 (ADC0) connected to VRY pin
#define LEFT_THRESHOLD 1000
#define RIGHT_THRESHOLD 3000
#define UP_THRESHOLD 1000
#define DOWN_THRESHOLD 3000

#define COMMAND_NO 0x00
#define COMMAND_LEFT 0x01
#define COMMAND_RIGHT 0x02
#define COMMAND_UP 0x04
#define COMMAND_DOWN 0x08