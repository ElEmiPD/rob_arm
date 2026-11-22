#include "servo_control.hpp"

#if VERSION == 2
#include <WiFi.h>
#include <PubSubClient.h>
#endif

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

// Handles de las tareas
/*
TaskHandle_t taskServo1Handle = NULL;
TaskHandle_t taskServo2Handle = NULL;
TaskHandle_t taskServo3Handle = NULL;
TaskHandle_t taskServo4Handle = NULL;
TaskHandle_t taskServo5Handle = NULL;
TaskHandle_t taskServo6Handle = NULL;
*/


// Feedback de movimiento
SemaphoreHandle_t motionDoneSemaphore;
QueueHandle_t     motionDoneQueue;      // NUEVO: generación terminada → TaskMQTT


static portMUX_TYPE motionMux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint8_t pendingServos = 0;
static volatile uint32_t motionGeneration = 0;
static volatile uint32_t completedGeneration = 0;

//Mutex para evitar condiciones de carrera
SemaphoreHandle_t commandMutex;

// wifi y mqtt
String stMac;

WiFiClient espClient;
PubSubClient client(espClient);

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
static const TickType_t SETTLE_TICKS = pdMS_TO_TICKS(300);

//-----------------------------------------------------
// Wifi
const char* ssid = MY_WIFI_SSID;
const char* password = MY_WIFI_PSWD;

// MQTT
/*
const char* mqtt_server = "mqtt-dashboard.com"; // 192.168.1.50
const int mqtt_port = 1883;
const char* mqtt_topic = "rob_arm/control"; // "stust/rob_arm/control"
*/

const char* mqtt_server       = "192.168.0.226";   
const int   mqtt_port         = 1883;
const char* mqtt_user         = MQTT_USER;
const char* mqtt_pass         = MQTT_PSWD;
const char* mqtt_topic        = "rob_arm/control";   // el brazo lee aquí
const char* mqtt_status_topic = "rob_arm/status";    // el brazo escribe aquí
const char* mqtt_conn_topic   = "rob_arm/conexion";  // online/offline
//-----------------------------------------------------

/*
  Posición a la que debe llegar cada servo.
  La escriben TaskSerial y mqttCallback, y la lee la tarea de cada servo
  (enteros de 32 bits: acceso atómico en ESP32).
*/
static volatile int targetPos[NUM_SERVOS];
static volatile int currentPos[NUM_SERVOS];

// Avanza 1 grado hacia el objetivo (solo escribe si hay cambio)
static inline bool stepServo(Servo &servo, int idx, int &pos) {
    int target = targetPos[idx];

    if (pos == target)
        return false;

    pos += (pos < target) ? 1 : -1;
    servo.write(pos);
    currentPos[idx] = pos;
    return true;
}

// Interpreta un comando como "base50" o "codo 60"

static void processCommand(char *cmd) {

    if (commandMutex != NULL)
        xSemaphoreTake(commandMutex, portMAX_DELAY);

    int newTargets[NUM_SERVOS];

    portENTER_CRITICAL(&motionMux);

    for (int i = 0; i < NUM_SERVOS; i++)
        newTargets[i] = targetPos[i];

    portEXIT_CRITICAL(&motionMux);

    bool validCommand = false;

    char *saveptr = NULL;
    char *token = strtok_r(cmd, ",", &saveptr);

    while (token != NULL) {

        char clean[50];
        size_t n = 0;

        for (char *p = token; *p && n < sizeof(clean) - 1; p++) {
            if (!isspace((unsigned char)*p))
                clean[n++] = (char)tolower((unsigned char)*p);
        }

        clean[n] = '\0';

        if (n == 0) {
            token = strtok_r(NULL, ",", &saveptr);
            continue;
        }

        if (strcmp(clean, "paro") == 0) {
            portENTER_CRITICAL(&motionMux);
            for (int i = 0; i < NUM_SERVOS; i++)
                newTargets[i] = currentPos[i];      // congelar donde están
            portEXIT_CRITICAL(&motionMux);

            validCommand = true;
            Serial.println("[Command] PARO DE EMERGENCIA");
            break;                                  // ignora el resto de tokens
        }

        char *num = clean;

        while (*num && !isdigit((unsigned char)*num) && *num != '-')
            num++;

        if (*num == '\0' || num == clean) {
            Serial.printf("[Command] Formato invalido: %s\n", clean);
            token = strtok_r(NULL, ",", &saveptr);
            continue;
        }

        char *end;
        long angle = strtol(num, &end, 10);

        if (*end != '\0') {
            Serial.printf("[Command] Angulo invalido: %s\n", clean);
            token = strtok_r(NULL, ",", &saveptr);
            continue;
        }

        *num = '\0';

        bool found = false;

        for (int i = 0; i < NUM_SERVOS; i++) {

            if (strcmp(clean, servoCfg[i].name) == 0) {

                found = true;

                if (angle < servoCfg[i].minAngle ||
                    angle > servoCfg[i].maxAngle) {

                    Serial.printf(
                        "[Command] %s fuera de rango (%d a %d)\n",
                        servoCfg[i].name,
                        servoCfg[i].minAngle,
                        servoCfg[i].maxAngle
                    );

                } else {

                    newTargets[i] = (int)angle;
                    validCommand = true;

                    Serial.printf(
                        "[Command] %s -> %ld\n",
                        servoCfg[i].name,
                        angle
                    );
                }

                break;
            }
        }

        if (!found)
            Serial.printf("[Command] Servo desconocido: %s\n", clean);

        token = strtok_r(NULL, ",", &saveptr);
    }

    if (validCommand) {

        portENTER_CRITICAL(&motionMux);

        for (int i = 0; i < NUM_SERVOS; i++)
            targetPos[i] = newTargets[i];

        pendingServos = NUM_SERVOS;
        motionGeneration++;

        portEXIT_CRITICAL(&motionMux);

        Serial.println("[ARM] Nuevo movimiento iniciado");
    }

    if (commandMutex != NULL)
        xSemaphoreGive(commandMutex);
}

// Tarea que acumula caracteres hasta recibir fin de línea
void TaskSerial(void *pvParameters) {
    char buf[100];
    size_t len = 0;
    bool overflow = false;                 

    while (1) {
        while (Serial.available()) {
            char c = (char)Serial.read();

            if (c == '\n' || c == '\r') {
                if (overflow) {
                    Serial.println("\n[Serial] Linea demasiado larga, descartada");
                    overflow = false;
                    len = 0;
                } else if (len > 0) {
                    buf[len] = '\0';
                    processCommand(buf);
                    len = 0;
                }
            } else if (len < sizeof(buf) - 1) {
                buf[len++] = c;
                Serial.write(c);  // echo
            } else {
                overflow = true;           
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void TaskMotionMonitor(void *pvParameters) {

    uint32_t lastReportedGeneration = 0;

    while (1) {

        if (xSemaphoreTake(motionDoneSemaphore, portMAX_DELAY) == pdTRUE) {

            bool completed = false;
            uint32_t generation = 0;

            portENTER_CRITICAL(&motionMux);

            if (pendingServos == 0 &&
                completedGeneration != lastReportedGeneration) {

                generation = completedGeneration;
                lastReportedGeneration = generation;
                completed = true;
            }

            portEXIT_CRITICAL(&motionMux);

            if (completed) {
                Serial.printf(
                    "[ARM] MOVEMENT_DONE - Generation: %lu\n",
                    (unsigned long)generation
                );
                // Sobrescribe: si hay una generación previa sin publicar, gana la más reciente
                xQueueOverwrite(motionDoneQueue, &generation);
            }
        }
    }
}

// ---------------------------------------------------------------
// wifi
// ---------------------------------------------------------------
void wifi_init() {
    WiFi.mode(WIFI_STA);
    stMac = WiFi.macAddress();

    Serial.print("[WiFi] MAC: ");
    Serial.println(stMac);

    xTaskCreatePinnedToCore(TaskWIFI, "WiFi", 3072, NULL, 2, NULL, 0);
}

void TaskWIFI(void *pvParameters)
{
    const uint32_t WIFI_TIMEOUT_MS = 15000;

    while (1) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[WiFi] Conectando...");
            WiFi.disconnect();
            WiFi.begin(ssid, password);

            uint32_t t0 = millis();
            while (WiFi.status() != WL_CONNECTED &&
                   millis() - t0 < WIFI_TIMEOUT_MS) {
                vTaskDelay(pdMS_TO_TICKS(500));
            }

            if (WiFi.status() == WL_CONNECTED) {
                Serial.print("[WiFi] Conectado. IP: ");
                Serial.println(WiFi.localIP());
            } else {
                Serial.println("[WiFi] Timeout, reintentando en 5 s");
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ---------------------------------------------------------------
// mqtt
// ---------------------------------------------------------------

void mqtt_init() {
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback);
    client.setKeepAlive(15);
    client.setSocketTimeout(3);

    xTaskCreatePinnedToCore(TaskMQTT, "MQTT", 4096, NULL, 1, NULL, 1);
}

// Cada mensaje recibido se interpreta como un comando (ej. "base50")
void mqttCallback(char* topic, byte* payload, unsigned int length)
{
    char buf[100];

    if (length >= sizeof(buf))
        length = sizeof(buf) - 1;

    Serial.print("[MQTT] Recibido en ");
    Serial.print(topic);
    Serial.print(": ");

    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
        buf[i] = (char)payload[i];
    }
    buf[length] = '\0';

    Serial.println();

    processCommand(buf);
}

void TaskMQTT(void *pvParameters)
{
    unsigned long lastAttempt = 0;
    unsigned long lastPublish = 0;
    uint32_t counter = 0;

    String clientId = "RobArm-" + stMac;

    while (1) {
        if (WiFi.status() == WL_CONNECTED) {

            if (!client.connected()) {
                if (millis() - lastAttempt >= 5000) {
                    lastAttempt = millis();

                    Serial.print("[MQTT] Conectando con ID: ");
                    Serial.println(clientId);

                    //if (client.connect(clientId.c_str())) {
                    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass,
                    mqtt_conn_topic, 0, true, "offline")) 
                    {   
                        Serial.println("[MQTT] Conectado");
                        client.publish(mqtt_conn_topic, "online", true);         // retenido

                        if (client.subscribe(mqtt_topic)) {
                            Serial.print("[MQTT] Suscrito a: ");
                            Serial.println(mqtt_topic);
                        }
                    } else {
                        Serial.printf(
                            "[MQTT] Error de conexion: %d\n",
                            client.state()
                        );
                    }
                }
            } else {
                client.loop();

                uint32_t doneGen;
                if (xQueueReceive(motionDoneQueue, &doneGen, 0) == pdTRUE) {
                    char msg[48];
                    snprintf(msg, sizeof(msg), "MOVEMENT_DONE gen=%lu", (unsigned long)doneGen);

                    if (client.publish(mqtt_status_topic, msg))
                        Serial.printf("[MQTT] Publicado: %s\n", msg);
                    else
                        Serial.println("[MQTT] Error al publicar MOVEMENT_DONE");
                }
                /*
                if (millis() - lastPublish >= 60000) {
                    lastPublish = millis();

                    char mensaje[100];

                    snprintf(
                        mensaje, sizeof(mensaje),
                        "Hola desde ESP32. Mensaje: %lu",
                        (unsigned long)++counter
                    );

                    if (client.publish(mqtt_status_topic, mensaje)) {
                        Serial.printf(
                            "[MQTT] Publicado: %s\n", mensaje
                        );
                    } else {
                        Serial.println("[MQTT] Error al publicar");
                    }
                }
                */
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ---------------------------------------------------------------
// Servos
// ---------------------------------------------------------------

void initServos() {

    // 1) Sincronización
    motionDoneSemaphore = xSemaphoreCreateBinary();
    commandMutex        = xSemaphoreCreateMutex();
    motionDoneQueue     = xQueueCreate(1, sizeof(uint32_t));   // ver cambio 4

    if (motionDoneSemaphore == NULL || commandMutex == NULL || motionDoneQueue == NULL) {
        Serial.println("[ARM] Error creando mecanismos de sincronizacion");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));   // no continuar sin sincronización
    }

    // 2) Estado inicial
    for (int i = 0; i < NUM_SERVOS; i++) {
        targetPos[i]  = POS_INICIAL;
        currentPos[i] = POS_INICIAL;                 // ver cambio 5
    }

    // 3) Pines
    servo1.attach(pinSM1);
    servo2.attach(pinSM2);
    servo3.attach(pinSM3);
    servo4.attach(pinSM4);
    servo5.attach(pinSM5);
    servo6.attach(pinSM6);

    // 4) Supervisor antes que los servos
    xTaskCreate(TaskMotionMonitor, "MotionMonitor", 2048, NULL, 2, NULL);

    // 5) Servos (arranque escalonado para no saturar la fuente)
    xTaskCreate(TaskServo1, "TaskServo1", 2048, NULL, 1, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));
    xTaskCreate(TaskServo2, "TaskServo2", 2048, NULL, 1, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));
    xTaskCreate(TaskServo3, "TaskServo3", 2048, NULL, 1, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));
    xTaskCreate(TaskServo4, "TaskServo4", 2048, NULL, 1, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));
    xTaskCreate(TaskServo5, "TaskServo5", 2048, NULL, 1, NULL);
    vTaskDelay(pdMS_TO_TICKS(200));
    xTaskCreate(TaskServo6, "TaskServo6", 2048, NULL, 1, NULL);

    // 6) Entrada de comandos AL FINAL
    xTaskCreate(TaskSerial, "TaskSerial", 3072, NULL, 1, NULL);

    Serial.println("[Servos] Comandos: <nombre><angulo>, ej. base50, codo60");
    for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.printf("  %s (%d a %d)\n", servoCfg[i].name, servoCfg[i].minAngle, servoCfg[i].maxAngle);
    }
}

void checkServoCompletion(int idx, int currentPos, TickType_t lastMoveTick,
                          uint32_t &lastGeneration, bool &reported)
{
    bool allCompleted = false;

    // Fuera de la sección crítica
    bool settled = (xTaskGetTickCount() - lastMoveTick) >= SETTLE_TICKS;

    portENTER_CRITICAL(&motionMux);

    uint32_t generation = motionGeneration;

    if (generation != lastGeneration) {
        lastGeneration = generation;
        reported = false;
    }

    if (!reported &&
        settled &&
        currentPos == targetPos[idx] &&
        pendingServos > 0) {

        pendingServos--;
        reported = true;

        if (pendingServos == 0) {
            completedGeneration = generation;
            allCompleted = true;
        }
    }

    portEXIT_CRITICAL(&motionMux);

    if (allCompleted) {
        xSemaphoreGive(motionDoneSemaphore);
    }
}

// Tareas de los servos
#if VERSION == 2
// ---- VERSION 2: cada servo va hacia su posición objetivo ----

void TaskServo1(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo1 = POS_INICIAL;

    uint32_t lastGeneration = 0;
    bool reported = false;
    TickType_t lastMoveTick = 0;


    // Base
    servo1.write(posServo1);

    while (1) {
        if (stepServo(servo1, 0, posServo1))
        lastMoveTick = xTaskGetTickCount();

        checkServoCompletion(0, posServo1, lastMoveTick, lastGeneration, reported);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskServo2(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo2 = POS_INICIAL;

    uint32_t lastGeneration = 0;
    bool reported = false;
    TickType_t lastMoveTick = 0;

    servo2.write(posServo2);

    while (1) {
        
        if (stepServo(servo2, 1, posServo2))
            lastMoveTick = xTaskGetTickCount();
        checkServoCompletion(1, posServo2, lastMoveTick, lastGeneration, reported);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskServo3(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo3 = POS_INICIAL;

    uint32_t lastGeneration = 0;
    bool reported = false;
    TickType_t lastMoveTick = 0;
    servo3.write(posServo3);

    while (1) {
        if (stepServo(servo3, 2, posServo3))
            lastMoveTick = xTaskGetTickCount();
        checkServoCompletion(2, posServo3, lastMoveTick, lastGeneration, reported);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskServo4(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo4 = POS_INICIAL;

    uint32_t lastGeneration = 0;
    bool reported = false;
    TickType_t lastMoveTick = 0;
    servo4.write(posServo4);

    while (1) {
            
        if (stepServo(servo4, 3, posServo4))
            lastMoveTick = xTaskGetTickCount();
        checkServoCompletion(3, posServo4, lastMoveTick, lastGeneration, reported);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskServo5(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo5 = POS_INICIAL;

    uint32_t lastGeneration = 0;
    bool reported = false;
    TickType_t lastMoveTick = 0;
    servo5.write(posServo5);

    while (1) {

        if (stepServo(servo5, 4, posServo5))
            lastMoveTick = xTaskGetTickCount();
        checkServoCompletion(4, posServo5, lastMoveTick, lastGeneration, reported);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void TaskServo6(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(15);
    int posServo6 = POS_INICIAL;

    uint32_t lastGeneration = 0;
    bool reported = false;
    TickType_t lastMoveTick = 0;
    servo6.write(posServo6);

    while (1) {
        if (stepServo(servo6, 5, posServo6))
            lastMoveTick = xTaskGetTickCount();
        checkServoCompletion(5, posServo6, lastMoveTick, lastGeneration, reported);
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