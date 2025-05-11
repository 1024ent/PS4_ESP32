/**
 * @file main.cpp
 * @brief Main Program
 * @copyright UMPSA ROBOTICS
 * @license Loo Hui Kie [ELPROG]
 */
/*
JOYSTICK TABLE
==================================
|BUTTONS | VALUE |  MECHANISM    |
==================================
|  RIGHT |   1   |     TBC       |
|  DOWN  |   2   |     TBC       |
|  UP    |   3   |     TBC       |
| LEFT   |   4   |     TBC       |
| SQUARE |   5   |     TBC       |
| CROSS  |   6   |     TBC       |
| CIRCLE |   7   |     TBC       |
|TRIANGLE|   8   |     TBC       |
|   L1   |   9   |     TBC       |
|   R1   |   10  |     TBC       |
|   L2   | pwm-- |  ROBOT BASE   |
|   R2   | pwm++ |  ROBOT BASE   |
|   L3   |  TBC  |     TBC       |
|   R3   |  TBC  |     TBC       |
|  L JOY | dir   |  ROBOT BASE   |
|  R JOY | dir   |  ROBOT BASE   |
|  SHARE |ACTIVE | MOTOR ACTIVATE|
| OPTIONS|  TBC  |     TBC       |
|TOUCHPAD|  TBC  |     TBC       |
==================================
*/
#include <Arduino.h>
#include "PS4Controller.h"
#include "CytronMotorDriver.h"

// --- Controller MAC Address (Use your own controller MAC) ---
#define CONTROLLER_2 "d8:bc:38:fc:9f:ba"

// --- Relay and LED Pins ---
#define RELAY_PIN 5
#define BLINK_GPIO1 GPIO_NUM_2

// --- Motor Pins ---
#define MOTOR1_PWM 18
#define MOTOR1_DIR 22
#define MOTOR2_PWM 23
#define MOTOR2_DIR 19

// --- Task Handles ---
TaskHandle_t myThreadIndicator = NULL;
TaskHandle_t myControllerLed = NULL;
TaskHandle_t task1 = NULL;

// --- RGB LED State ---
int r = 255, g = 0, b = 0;

// --- Function Prototypes ---
void nextRainbowColor();
void controller_led_sequence(void *pvParameters);
void thread_indicator(void *pvParameters);
void task_1(void *pvParameters);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting PS4 Controller + Motor Control RTOS App...");

  // --- PS4 Controller Init ---
  if (!PS4.begin(CONTROLLER_2)) {
    Serial.println("PS4 Controller 2 not found!");
    while (true);  // Halt if not found
  }
  Serial.println("✅ PS4 Controller connected.");

  // --- GPIO Init ---
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  gpio_pad_select_gpio(BLINK_GPIO1);
  gpio_set_direction(BLINK_GPIO1, GPIO_MODE_OUTPUT);

  // --- Task: Motor + Relay Sequence ---
  xTaskCreatePinnedToCore(
    task_1,
    "Motor Control Task",
    10000,
    NULL,
    5,
    &task1,
    1
  );

  // --- Task: Controller LED Effects ---
  xTaskCreatePinnedToCore(
    controller_led_sequence,
    "Controller LED",
    2048,
    NULL,
    1,
    &myControllerLed,
    0
  );

  // --- Task: Onboard LED Blinker ---
  xTaskCreatePinnedToCore(
    thread_indicator,
    "Thread LED",
    1024,
    NULL,
    1,
    &myThreadIndicator,
    0
  );
}

void loop() {
  // Nothing to do here — everything is handled by FreeRTOS tasks
}

void task_1(void *pvParameters) {
  bool isRunning = false;
  bool resetRequired = false;

  CytronMD motor1(PWM_DIR, 18, 22);  // LEFT motor
  CytronMD motor2(PWM_DIR, 23, 19);  // RIGHT motor

  motor1.setSpeed(0);
  motor2.setSpeed(0);

  for(;;) {
    if (PS4.isConnected()) {
      if (PS4.Circle() && !isRunning && !resetRequired) {
        Serial.println("✅ Circle button detected!");
        isRunning = true;

        vTaskDelay(pdMS_TO_TICKS(500));

        Serial.println("⚙️ Motors 50%...");
        motor1.setSpeed(-125);
        motor2.setSpeed(127);
        vTaskDelay(pdMS_TO_TICKS(1500));

        Serial.println("⚙️ Motors 100% and relay ON");
        motor1.setSpeed(-250);
        motor2.setSpeed(255);
        digitalWrite(RELAY_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(3000));

        digitalWrite(RELAY_PIN, LOW);
        Serial.println("Relay OFF");

        motor1.setSpeed(-125);
        motor2.setSpeed(127);
        vTaskDelay(pdMS_TO_TICKS(1500));

        motor1.setSpeed(-75);
        motor2.setSpeed(76);
        vTaskDelay(pdMS_TO_TICKS(1500));

        motor1.setSpeed(0);
        motor2.setSpeed(0);
        Serial.println("✅ Motors stopped");

        resetRequired = true;
        isRunning = false;
      }

      if (PS4.L1() && !isRunning) {
        resetRequired = false;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void controller_led_sequence(void *pvParameters) {
  for (;;) {
    if (PS4.isConnected()) {
      PS4.setLed(r, g, b);
      PS4.setFlashRate(0, 0);
      PS4.sendToController();
      nextRainbowColor();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void nextRainbowColor() {
  const int increment = 5;

  if (r > 0 && b == 0) {
    r -= increment;
    g += increment;
  } else if (g > 0 && r == 0) {
    g -= increment;
    b += increment;
  } else if (b > 0 && g == 0) {
    r += increment;
    b -= increment;
  }
}

void thread_indicator(void *pvParameters) {
  for (;;) {
    gpio_set_level(BLINK_GPIO1, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(BLINK_GPIO1, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
