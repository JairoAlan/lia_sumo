#include <Arduino.h>

// Pines de los motores
#define LEFT_MOTOR_FORWARD 25
#define LEFT_MOTOR_BACKWARD 33
#define RIGHT_MOTOR_FORWARD 27
#define RIGHT_MOTOR_BACKWARD 26

// Pines de los sensores de seguimiento
#define LEFT_TRACKER_PIN 39
#define RIGHT_TRACKER_PIN 36

// Pines de los sensores infrarrojos
const int IR_Sensor_Centro = 14;
const int IR_Sensor_Derecho = 13;
const int IR_Sensor_Izquierdo = 2;

// Definir variables para las distancias medidas
float distances[3];

// Mutex para sincronizar el acceso a los sensores infrarrojos
SemaphoreHandle_t irSensorMutex;

// Funciones para controlar los motores
void moveForward() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
}

void moveBackward() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void turnLeft(int degrees = 5) {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  delay(degrees * 10);  // Ajusta este valor según la velocidad de tu robot
}

void turnRight(int degrees = 5) {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  delay(degrees * 10);  // Ajusta este valor según la velocidad de tu robot
}

void stop() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void backupAndTurn() {
  moveBackward();
  delay(200);  // Retrocede por 0.2 segundos
  stop();
  delay(100);  // Pequeña pausa
  turnLeft(45);  // Gira a la izquierda por 45 grados
  stop();
}

// Función para manejar los sensores infrarrojos
void irSensorThread(void *param) {
  while (true) {
    // Tomar el mutex para acceder a los sensores infrarrojos
    xSemaphoreTake(irSensorMutex, portMAX_DELAY);

    float SIR_Centro = analogRead(IR_Sensor_Centro);
    float SIR_Derecho = analogRead(IR_Sensor_Derecho);
    float SIR_Izquierdo = analogRead(IR_Sensor_Izquierdo);

    float voltageCentro = SIR_Centro * (5.0 / 4095.0);
    float voltageDerecho = SIR_Derecho * (5.0 / 4095.0);
    float voltageIzquierdo = SIR_Izquierdo * (5.0 / 4095.0);

    distances[0] = 27.728 * pow(voltageCentro, -1.2045);  // Distancia Centro
    distances[1] = 27.728 * pow(voltageIzquierdo, -1.2045);  // Distancia Izquierdo
    distances[2] = 27.728 * pow(voltageDerecho, -1.2045);  // Distancia Derecho

    // Liberar el mutex
    xSemaphoreGive(irSensorMutex);

    delay(50);  // Tiempo de espera reducido para mayor velocidad
  }
}

// Nueva función para detectar un oponente
bool detectOpponent() {
  // Tomar el mutex para acceder a los sensores infrarrojos
  xSemaphoreTake(irSensorMutex, portMAX_DELAY);

  // Verificar si alguna distancia es menor que un umbral, indicando un oponente
  bool opponentDetected = (distances[0] < 55 || distances[1] < 55 || distances[2] < 55);

  // Liberar el mutex
  xSemaphoreGive(irSensorMutex);

  return opponentDetected;
}

// Función para manejar el movimiento de los motores y los sensores de seguimiento
void motorThread(void *param) {
  while (true) {
    int leftTrackerValue = analogRead(LEFT_TRACKER_PIN);
    int rightTrackerValue = analogRead(RIGHT_TRACKER_PIN);

    // Verificar si el valor del sensor izquierdo o derecho es >= 100
    if (leftTrackerValue >= 100 || rightTrackerValue >= 100) {
      moveForward();
    } else {
      backupAndTurn();
    }

    // Usar la nueva función para detectar un oponente
    if (detectOpponent()) {
      if (distances[1] < distances[0] && distances[1] < distances[2]) {
        turnLeft();
        moveForward();
      } else if (distances[2] < distances[0] && distances[2] < distances[1]) {
        turnRight();
        moveForward();
      } else {
        moveForward();
      }
    } else {
      moveForward();
    }

    if (distances[0] < 9 ) {
      moveForward();
    } else if( distances[1] < 9 ){
      turnLeft();
      moveForward();
    } else if( distances[2] < 9 ){
      turnRight();
      moveForward();
    } else {
      moveForward();
    }

    delay(50);  // Tiempo de espera reducido para mayor velocidad
  }
}

void setup() {
  Serial.begin(115200);

  // Configurar los pines de los motores como salida
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Configurar los pines de los sensores de seguimiento como entrada
  pinMode(LEFT_TRACKER_PIN, INPUT);
  pinMode(RIGHT_TRACKER_PIN, INPUT);

  // Configurar los pines de los sensores infrarrojos como entrada
  pinMode(IR_Sensor_Centro, INPUT);
  pinMode(IR_Sensor_Derecho, INPUT);
  pinMode(IR_Sensor_Izquierdo, INPUT);

  // Crear el mutex para sincronizar el acceso a los sensores infrarrojos
  irSensorMutex = xSemaphoreCreateMutex();

  // Espera de 3 segundos antes de comenzar
  Serial.println("Esperando 3 segundos antes de comenzar...");
  delay(3000);
  Serial.println("¡Comenzando!");

  // Iniciar los hilos (simulados con tareas en Arduino)
  xTaskCreatePinnedToCore(irSensorThread, "IrSensorThread", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(motorThread, "MotorThread", 1024, NULL, 1, NULL, 1);
}

void loop() {
  // Mantener el bucle principal corriendo
  delay(1000);
}
