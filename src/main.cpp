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

#define CONTROLLER_1 "14:2b:2f:c0:28:fe"
#define CONTROLLER_2 ""

TaskHandle_t task1 = NULL;
TaskHandle_t task2 = NULL;

void task_1(void *pvParameters);
void task_2(void *pvParameters);

void setup() {
  Serial.begin(115200);
  Serial.println("PS4 Controller Test");

  if (!PS4.begin(CONTROLLER_1)) {
    Serial.println("PS4 Controller 1 not found");
    while (1);  // halt if controller not found
  }
  Serial.println("PS4 Controller found");

  xTaskCreatePinnedToCore(task_1, 
                          "Running Task 1", 
                          4096,  // 75000 is excessive; 4KB is typically sufficient
                          NULL, 
                          2, 
                          &task1, 
                          1);

  xTaskCreatePinnedToCore(task_2, 
                          "Running Task 2", 
                          4096, 
                          NULL, 
                          2, 
                          &task2, 
                          1);  
}

void loop() {
  // Empty as tasks handle everything
}

void task_1(void *pvParameters) {
  bool isRunning = false;
  bool resetRequired = false;

  for(;;) {
    if(PS4.isConnected()) {
      if(PS4.Right() && !isRunning && !resetRequired) {
        isRunning = true; 

        // Action: e.g., rotate motor CW
        Serial.println("Task 1: Motor CW");

        resetRequired = true;
        isRunning = false;
      }
      if(PS4.L1() && !isRunning) {
        resetRequired = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Use proper FreeRTOS macro
  }
}

void task_2(void *pvParameters) {
  bool isRunning = false;
  bool resetRequired = false;

  for(;;) {
    if(PS4.isConnected()) {
      if(PS4.Down() && !isRunning && !resetRequired) {
        isRunning = true; 

        // Action: e.g., rotate motor CCW
        Serial.println("Task 2: Motor CCW");

        resetRequired = true;
        isRunning = false;
      }
      if(PS4.L1() && !isRunning) {
        resetRequired = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Consistent timing macro
  }
}

