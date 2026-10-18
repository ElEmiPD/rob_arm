#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#define VERSION 2

#define NUM_SERVOS 6

#include <Arduino.h>
#include <ESP32Servo.h>

// Definición de pines
typedef enum {
    pinSM1 = 19,    // Base
    pinSM2 = 23,    // Hombro
    pinSM3 = 18,    // Codo
    pinSM4 = 17,    // Muñeca
    pinSM5 = 16,    // Giro
    pinSM6 = 14     // Pinza
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

#if VERSION == 2
// Tarea que lee los comandos del puerto serial (ej. "base50", "codo60")
void TaskSerial(void *pvParameters);

#endif // VERSION == 2

#endif // SERVO_CONTROL_H