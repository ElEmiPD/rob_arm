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

#if VERSION == 2
// ===============================================================
// VERSION 2: control por puerto serial
// ===============================================================

/*
  Nombre del comando y rango de cada servo (indice 0 = servo 1).
  Comando: <nombre><angulo>  ->  base50, codo60, pinza120 ...
  Cambia los nombres aquí si tu brazo usa otros.
*/
struct ServoConfig {
    const char *name;
    int minAngle;
    int maxAngle;
};

static const ServoConfig servoCfg[NUM_SERVOS] = {
    {"base",   0,   180},  // Servo 1
    {"hombro", 20,  150},  // Servo 2
    {"codo",   0,   180},  // Servo 3
    {"muneca", 10,  180},  // Servo 4
    {"giro",   0,   180},  // Servo 5
    {"pinza",  20,  150}   // Servo 6
};

static const int POS_INICIAL = 90;

/*
  Posición a la que debe llegar cada servo.
  La escribe TaskSerial y la lee la tarea de cada servo
  (enteros de 32 bits: acceso atómico en ESP32).
*/
static volatile int targetPos[NUM_SERVOS];

// Avanza 1 grado hacia el objetivo (solo escribe si hay cambio)
static inline void stepServo(Servo &servo, int idx, int &pos) {
    int target = targetPos[idx];

    if (pos < target) {
        pos++;
        servo.write(pos);
    } else if (pos > target) {
        pos--;
        servo.write(pos);
    }
}

// Interpreta un comando como "base50" o "codo 60"
static void processCommand(char *cmd) {
    // Pasar a minúsculas y quitar espacios
    char clean[32];
    size_t n = 0;
    for (char *p = cmd; *p && n < sizeof(clean) - 1; p++) {
        if (!isspace((unsigned char)*p))
            clean[n++] = (char)tolower((unsigned char)*p);
    }
    clean[n] = '\0';

    if (n == 0) return;

    // Buscar dónde empieza el número
    char *num = clean;
    while (*num && !isdigit((unsigned char)*num) && *num != '-') num++;

    if (*num == '\0' || num == clean) {
        Serial.println("[Serial] Formato invalido. Ejemplo: base50");
        return;
    }

    char *end;
    long angle = strtol(num, &end, 10);
    if (*end != '\0') {
        Serial.println("[Serial] Formato invalido. Ejemplo: base50");
        return;
    }

    *num = '\0';  // ahora 'clean' contiene solo el nombre

    for (int i = 0; i < NUM_SERVOS; i++) {
        if (strcmp(clean, servoCfg[i].name) == 0) {
            if (angle < servoCfg[i].minAngle || angle > servoCfg[i].maxAngle) {
                Serial.printf("[Serial] %s fuera de rango (%d a %d)\n",
                              servoCfg[i].name, servoCfg[i].minAngle, servoCfg[i].maxAngle);
                return;
            }
            targetPos[i] = (int)angle;
            Serial.printf("[Serial] %s -> %ld\n", servoCfg[i].name, angle);
            return;
        }
    }

    Serial.printf("[Serial] Servo desconocido: %s\n", clean);
}

// Tarea que acumula caracteres hasta recibir fin de línea
void TaskSerial(void *pvParameters) {
    char buf[32];
    size_t len = 0;

    while (1) {
        while (Serial.available()) {
            char c = (char)Serial.read();

            if (c == '\n' || c == '\r') {
                if (len > 0) {
                    buf[len] = '\0';
                    processCommand(buf);
                    len = 0;
                }
            } else if (len < sizeof(buf) - 1) {
                buf[len++] = c;
                Serial.write(c);  // echo
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#endif // VERSION == 2

// ===============================================================
// Inicialización (común a ambas versiones)
// ===============================================================
void initServos() {
#if VERSION == 2
    for (int i = 0; i < NUM_SERVOS; i++)
        targetPos[i] = POS_INICIAL;
#endif

    // Adjuntar pines
    servo1.attach(pinSM1);
    servo2.attach(pinSM2);
    servo3.attach(pinSM3);
    servo4.attach(pinSM4);
    servo5.attach(pinSM5);
    servo6.attach(pinSM6);

    xTaskCreate(TaskServo1, "TaskServo1", 3072, NULL, 1, NULL);
    xTaskCreate(TaskServo2, "TaskServo2", 3072, NULL, 1, NULL);
    xTaskCreate(TaskServo3, "TaskServo3", 3072, NULL, 1, NULL);
    xTaskCreate(TaskServo4, "TaskServo4", 3072, NULL, 1, NULL);
    xTaskCreate(TaskServo5, "TaskServo5", 3072, NULL, 1, NULL);
    xTaskCreate(TaskServo6, "TaskServo6", 3072, NULL, 1, NULL);

#if VERSION == 2
    xTaskCreate(TaskSerial, "TaskSerial", 3072, NULL, 1, NULL);

    Serial.println("[Servos] Comandos: <nombre><angulo>, ej. base50, codo60");
    for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.printf("  %s (%d a %d)\n", servoCfg[i].name, servoCfg[i].minAngle, servoCfg[i].maxAngle);
    }
#endif
}

// ===============================================================
// Tareas de los servos
// ===============================================================
#if VERSION == 2
// ---- VERSION 2: cada servo va hacia su posición objetivo ----

void TaskServo1(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo1 = POS_INICIAL;

    // Base
    servo1.write(posServo1);

    while (1) {
        stepServo(servo1, 0, posServo1);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskServo2(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo2 = POS_INICIAL;

    servo2.write(posServo2);

    while (1) {
        stepServo(servo2, 1, posServo2);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskServo3(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo3 = POS_INICIAL;

    servo3.write(posServo3);

    while (1) {
        stepServo(servo3, 2, posServo3);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskServo4(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo4 = POS_INICIAL;

    servo4.write(posServo4);

    while (1) {
        stepServo(servo4, 3, posServo4);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskServo5(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo5 = POS_INICIAL;

    servo5.write(posServo5);

    while (1) {
        stepServo(servo5, 4, posServo5);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskServo6(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo6 = POS_INICIAL;

    servo6.write(posServo6);

    while (1) {
        stepServo(servo6, 5, posServo6);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

#else
// ---- VERSIONES ANTERIORES: barrido automático ----

void TaskServo1(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo1 = 90;
    int dirServo1 = 1;

    // Base
    // Rango de movimiento del servo 1: 0 a 180 grados

    while (1) {
        posServo1 += dirServo1;

        if (posServo1 >= 180 || posServo1 <= 0)
            dirServo1 = -dirServo1;

        servo1.write(posServo1);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskServo2(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);

    int posServo2 = 90;
    int dirServo2 = 1;

    while (1) {
        posServo2 += dirServo2;

        if (posServo2 >= 150 || posServo2 <= 20)
            dirServo2 = -dirServo2;

        servo2.write(posServo2);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskServo3(void *pvParameters) {
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

void TaskServo4(void *pvParameters) {
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

void TaskServo5(void *pvParameters) {
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

void TaskServo6(void *pvParameters) {
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

#endif // VERSION == 2