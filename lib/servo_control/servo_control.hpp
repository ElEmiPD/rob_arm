#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <ESP32Servo.h>

// Definición de pines
typedef enum {
    pinSM1 = 23, 
    pinSM2 = 19, 
    pinSM3 = 18, 
    pinSM4 = 17, 
    pinSM5 = 16, 
    pinSM6 = 14
} Servos_t;

// Función principal para inicializar los servos y lanzar las tareas de FreeRTOS
void initServos();

// Declaración de las tareas
void TaskServo1(void *pvParameters);
void TaskServo2(void *pvParameters);
void TaskServo3(void *pvParameters);
void TaskServo4(void *pvParameters);
void TaskServo5(void *pvParameters);
void TaskServo6(void *pvParameters);

#endif // SERVO_CONTROL_H