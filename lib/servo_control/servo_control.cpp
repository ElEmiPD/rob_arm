#include "servo_control.hpp"

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

// Handles de las tareas
TaskHandle_t taskServo1Handle = NULL;
TaskHandle_t taskServo2Handle = NULL;
TaskHandle_t taskServo3Handle = NULL;
TaskHandle_t taskServo4Handle = NULL;
TaskHandle_t taskServo5Handle = NULL;
TaskHandle_t taskServo6Handle = NULL;

void initServos() {
    // Adjuntar pines
    servo1.attach(pinSM1);
    servo2.attach(pinSM2);
    servo3.attach(pinSM3);
    servo4.attach(pinSM4);
    servo5.attach(pinSM5);
    servo6.attach(pinSM6);

    xTaskCreate(TaskServo1, "TaskServo1", 2048, NULL, 1, NULL);
    xTaskCreate(TaskServo2, "TaskServo2", 2048, NULL, 1, NULL);
    xTaskCreate(TaskServo3, "TaskServo3", 2048, NULL, 1, NULL);
    xTaskCreate(TaskServo4, "TaskServo4", 2048, NULL, 1, NULL);
    xTaskCreate(TaskServo5, "TaskServo5", 2048, NULL, 1, NULL);
    xTaskCreate(TaskServo6, "TaskServo6", 2048, NULL, 1, NULL);
}

void TaskServo1(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo1 = 90;
    int dirServo1 = 1;

    // Base
    // Rango de movimiento del servo 1: 0 a 180 grados

    while (1) {
        posServo1 += dirServo1;
        if (posServo1 >= 180 || posServo1 <= 0) {  
            if (posServo1 <= 0)
                Serial.println("[TaskServo1] Inicio Servo 1");
            dirServo1 = -dirServo1;
        }
        servo1.write(posServo1);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


void TaskServo2(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(15);

  int posServo2 = 90;
  int dirServo2 = 1;

  while (1) {
    posServo2 += dirServo2;

    if (posServo2 >= 150 || posServo2 <= 20) {
      if (posServo2 <= 90)
        Serial.println("[TaskServo2] Inicio Servo 2");

      dirServo2 = -dirServo2;
    }

    servo2.write(posServo2);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


void TaskServo3(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(15);

  int posServo3 = 90;
  int dirServo3 = 1;

  while (1) {
    posServo3 += dirServo3;

    if (posServo3 >= 180 || posServo3 <= 0) { //180 0
      if (posServo3 <= 0)
        Serial.println("[TaskServo3] Inicio Servo 3");

      dirServo3 = -dirServo3;
    }

    servo3.write(posServo3);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskServo4(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(15);

  int posServo4 = 90;
  int dirServo4 = 1;

  while (1) {
    posServo4 += dirServo4;

    if (posServo4 >= 181 || posServo4 <= 10) { 
      if (posServo4 <= 10)
        Serial.println("[TaskServo4] Inicio Servo 4");

      dirServo4 = -dirServo4;
    }

    servo4.write(posServo4);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskServo5(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(15);

  int posServo5 = 90;
  int dirServo5 = 1;

  while (1) {
    posServo5 += dirServo5;

    if (posServo5 >= 180 || posServo5 <= 0) { 
      if (posServo5 <= 0)
        Serial.println("[TaskServo5] Inicio Servo 5");

      dirServo5 = -dirServo5;
    }

    servo5.write(posServo5);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskServo6(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(15);

  int posServo6 = 90;
  int dirServo6 = 1;

  while (1) {
    posServo6 += dirServo6;

    if (posServo6 >= 150 || posServo6 <= 20) { //20 150
      if (posServo6 <= 20)
        Serial.println("[TaskServo6] Inicio Servo 6");

      dirServo6 = -dirServo6;
    }

    servo6.write(posServo6);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

