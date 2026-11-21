// Version 2: control de servos por puerto serial (semaforo + tareas FreeRTOS)
#include <Arduino.h>
#include "servo_control.hpp"

void setup() {
    Serial.begin(115200);

    initServos(); 
    wifi_init();
    mqtt_init();
}

void loop() {
    vTaskDelete(NULL);   // todo corre en tareas FreeRTOS; loopTask ya no es necesaria
}