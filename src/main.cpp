#define VERSION 1

#if VERSION == 0

#include <Arduino.h>
#include <ESP32Servo.h>

typedef enum {pinSM1 = 23, pinSM2 = 19, pinSM3 = 18, pinSM4 = 17, pinSM5 = 16, pinSM6 = 14} Servos_t;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

TaskHandle_t taskServo1Handle = NULL;
TaskHandle_t taskServo2Handle = NULL;
TaskHandle_t taskServo3Handle = NULL;
TaskHandle_t taskServo4Handle = NULL;
TaskHandle_t taskServo5Handle = NULL;
TaskHandle_t taskServo6Handle = NULL;

void TaskServo1(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(15);

  int posServo1 = 90;
  int dirServo1 = 1;

  while (1) {
    posServo1 += dirServo1;

    if (posServo1 >= 90 || posServo1 <= 90) {  //150 25
      if (posServo1 <= 90) // 25
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

    if (posServo2 >= 90 || posServo2 <= 90) {
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

    if (posServo3 >= 90 || posServo3 <= 90) { //180 0
      if (posServo3 <= 90)
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

    if (posServo6 >= 150 || posServo6 <= 20) { //20 149
      if (posServo6 <= 20)
        Serial.println("[TaskServo6] Inicio Servo 6");

      dirServo6 = -dirServo6;
    }

    servo6.write(posServo6);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup()
{
  Serial.begin(115200);

  servo1.attach(pinSM1);
  servo2.attach(pinSM2);
  servo3.attach(pinSM3);
  servo4.attach(pinSM4);
  servo5.attach(pinSM5);
  servo6.attach(pinSM6);

  xTaskCreate(
    TaskServo1,
    "TaskServo1",
    2048,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    TaskServo2,
    "TaskServo2",
    2048,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    TaskServo3,
    "TaskServo3",
    2048,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    TaskServo4,
    "TaskServo4",
    2048,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    TaskServo5,
    "TaskServo5",
    2048,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    TaskServo6,
    "TaskServo6",
    2048,
    NULL,
    1,
    NULL
  );

}

void loop(){}

#endif

#if VERSION == 1
// Version 1: prototipos y definiciones en cabecera
#include <Arduino.h>
#include "servo_control.hpp"

void setup() {
    Serial.begin(115200);

    initServos();
}

void loop() {}

#endif

#define VERSION 2

#if VERSION == 2
// Version 2: control de servos por puerto serial (semaforo + tareas FreeRTOS)
#include <Arduino.h>
#include "servo_control.hpp"

void setup() {
    Serial.begin(115200);

    initServos();   // Crea el semáforo, las 6 tareas de servo y la tarea serial
}

void loop() {}

#endif