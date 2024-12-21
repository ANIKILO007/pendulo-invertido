#include <Wire.h>
#include <MPU6050.h>

// Instancia del MPU6050
MPU6050 mpu;

// Pines del L298N
const int motorSpeedA = 5;
const int motorDir1A = 6;
const int motorDir2A = 7;
const int motorSpeedB = 10;
const int motorDir1B = 8;
const int motorDir2B = 9;

// Variables del PID
float setPoint = -1.0; // Punto de equilibrio
float input, output;
float Kp = 20, Ki = 8, Kd = 0.1; // Ajusta estos valores
float previousError = 0, integral = 0;

// Variables de tiempo
unsigned long lastTime = 0;
const unsigned long sampleTime = 10; // Tiempo de muestreo en ms

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Inicializar el MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Error al conectar con el MPU6050");
    while (1);
  }

  // Configurar pines del motor
  pinMode(motorSpeedA, OUTPUT);
  pinMode(motorDir1A, OUTPUT);
  pinMode(motorDir2A, OUTPUT);
  pinMode(motorSpeedB, OUTPUT);
  pinMode(motorDir1B, OUTPUT);
  pinMode(motorDir2B, OUTPUT);

  Serial.println("Sistema iniciado");
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= sampleTime) {
    lastTime = now;

    // Leer ángulo del MPU6050
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Calcular el ángulo para el eje Y
    float angle = atan2(ay, az) * 180 / PI;

    // Controlador PID
    float error = setPoint - angle;
    integral += error * sampleTime / 1000.0;
    float derivative = (error - previousError) / (sampleTime / 1000.0);
    output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    // Aplicar la salida a los motores
    setMotorSpeed(output);

    // Mostrar datos para debug
    Serial.print("Ángulo: "); Serial.print(angle);
    Serial.print(" | Salida PID: "); Serial.println(output);
  }
}

void setMotorSpeed(float speed) {
  if (speed > 0) {
    analogWrite(motorSpeedA, constrain(speed, 0, 100));
    analogWrite(motorSpeedB, constrain(speed, 0, 100));
    digitalWrite(motorDir1A, HIGH);
    digitalWrite(motorDir2A, LOW);
    digitalWrite(motorDir1B, HIGH);
    digitalWrite(motorDir2B, LOW);
  } else {
    analogWrite(motorSpeedA, constrain(-speed, 0, 100));
    analogWrite(motorSpeedB, constrain(-speed, 0, 100));
    digitalWrite(motorDir1A, LOW);
    digitalWrite(motorDir2A, HIGH);
    digitalWrite(motorDir1B, LOW);
    digitalWrite(motorDir2B, HIGH);
  }
}
