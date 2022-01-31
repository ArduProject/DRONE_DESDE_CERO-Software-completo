/*
   Reducir vibraciones de hélices y motores haciendo uso de las lecturas del ecelerómetro del sensor MPU6050
   Más información en https://arduproject.es/calibracion-de-helices-y-motores-sketch/
*/


#define usCiclo 6000    // Ciclo de ejecucion de software en microsegundos (PWM)
#define pin_motor1 3    // Pin motor 1
#define pin_motor2 4    // Pin motor 2
#define pin_motor3 5    // Pin motor 3
#define pin_motor4 6    // Pin motor 4
#define pin_Throttle 8  // Pin throttle del mando RC
#define pin_LED_rojo 13 // Pin LED rojo

#include <EnableInterrupt.h>
#include <Wire.h>

// MPU6050
#define MPU6050_adress 0x68
int gx, gy, gz;
float ax, ay, az, temperature;

// PWM
float ESC1_us, ESC2_us, ESC3_us, ESC4_us, loop_timer;
long tiempo_motores_start;

// AJUSTE MANDO RC - THROTLLE
float RC_Throttle_consigna;
const int us_max_Throttle_adj = 2000;
const int us_min_Throttle_adj = 950;
const float us_max_Throttle_raw = 2004; // <-- Si teneis la entrada Throttle invertida sustituid este valor
const float us_min_Throttle_raw = 1116; // <-- por este y viceversa

// INTERRUPCIÓN MANDO RC --> THROTTLE
volatile long Throttle_HIGH_us;
volatile int RC_Throttle_raw;
void INT_Throttle() {
  if (digitalRead(pin_Throttle) == HIGH) Throttle_HIGH_us = micros();
  if (digitalRead(pin_Throttle) == LOW)  RC_Throttle_raw  = micros() - Throttle_HIGH_us;
}

void setup() {

  // Decalaración de LEDs
  pinMode(pin_LED_rojo, OUTPUT);
  digitalWrite(pin_LED_rojo, HIGH);

  // Serial.begin()
  Serial.begin(115200);
  Wire.begin();

  // Interrupción para canal Thrtottle
  pinMode(pin_Throttle, INPUT_PULLUP);
  enableInterrupt(pin_Throttle, INT_Throttle, CHANGE);

  // Decalaración de motores
  pinMode(pin_motor1, OUTPUT);   // Declarar motor 1
  pinMode(pin_motor2, OUTPUT);   // Declarar motor 2
  pinMode(pin_motor3, OUTPUT);   // Declarar motor 3
  pinMode(pin_motor4, OUTPUT);   // Declarar motor 4
  digitalWrite(pin_motor1, LOW); // Motor 1 LOW por seguridad
  digitalWrite(pin_motor2, LOW); // Motor 2 LOW por seguridad
  digitalWrite(pin_motor3, LOW); // Motor 3 LOW por seguridad
  digitalWrite(pin_motor4, LOW); // Motor 4 LOW por seguridad

  // Iniciar sensor MPU6050
  MPU6050_iniciar();

  Serial.println("Encender mando RC y bajar Throttle al mínimo");
  // Hasta no bajar Throttle al mínimo no salimos de este bucle while
  while (RC_Throttle_consigna > 1100 || RC_Throttle_consigna < 900) {
    RC_Throttle_consigna = map(RC_Throttle_raw, us_min_Throttle_raw, us_max_Throttle_raw, us_min_Throttle_adj, us_max_Throttle_adj);
  }
  Serial.println("Encendido detectado");
  digitalWrite(pin_LED_rojo, LOW);
}

void loop() {
  //  Nuevo ciclo
  while (micros() - loop_timer < usCiclo);
  loop_timer = micros();

  // Leer sensor MPU6050
  MPU6050_leer();

  //  Ecuaciones de procesamiento de la señal Throttle
  RC_Throttle_consigna  = map(RC_Throttle_raw, us_min_Throttle_raw, us_max_Throttle_raw, us_min_Throttle_adj, us_max_Throttle_adj);

  // La señal RC_Throttle_consigna define el tiempo que la señal PWM está en estado HIGH
  ESC1_us = RC_Throttle_consigna;
  ESC2_us = RC_Throttle_consigna;
  ESC3_us = RC_Throttle_consigna;
  ESC4_us = RC_Throttle_consigna;

  // Para generar las 4 señales PWM, el primer paso es poner estas señales a 1 (HIGH).
  digitalWrite(pin_motor1, HIGH);  // MOTOR 1
  digitalWrite(pin_motor2, HIGH);  // MOTOR 2
  digitalWrite(pin_motor3, HIGH);  // MOTOR 3
  digitalWrite(pin_motor4, HIGH);  // MOTOR 4
  tiempo_motores_start = micros();

  // Cuando se cumpa el tiempo de PWM definido en ESCx_us, se pasa cada señal a 0 (LOW) para terminar el ciclo PWM.
  while (digitalRead(pin_motor1) == HIGH || digitalRead(pin_motor2) == HIGH || digitalRead(pin_motor3) == HIGH || digitalRead(pin_motor4) == HIGH) {
    if (tiempo_motores_start + ESC1_us <= micros()) digitalWrite(pin_motor1, LOW); // MOTOR 1
    if (tiempo_motores_start + ESC2_us <= micros()) digitalWrite(pin_motor2, LOW); // MOTOR 2
    if (tiempo_motores_start + ESC3_us <= micros()) digitalWrite(pin_motor3, LOW); // MOTOR 3
    if (tiempo_motores_start + ESC4_us <= micros()) digitalWrite(pin_motor4, LOW); // MOTOR 4
  }

  // Monitor Serie
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.println(az);
}

// Iniciar sensor MPU6050
void MPU6050_iniciar() {
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x6B);                          // PWR_MGMT_1 registro 6B hex
  Wire.write(0x00);                          // 00000000 para activar
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1B);                          // GYRO_CONFIG registro 1B hex
  Wire.write(0x08);                          // 00001000: 500dps
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1C);                          // ACCEL_CONFIG registro 1C hex
  Wire.write(0x10);                          // 00010000: +/- 8g
  Wire.endTransmission();
}

// Leer sensor MPU6050
void MPU6050_leer() {
  // Los datos del giroscopio y el acelerómetro se encuentran de la dirección 3B a la 14
  Wire.beginTransmission(MPU6050_adress);       // Empezamos comunicación
  Wire.write(0x3B);                             // Pedir el registro 0x3B (AcX)
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_adress, 14);         // Solicitar un total de 14 registros
  while (Wire.available() < 14);                // Esperamos hasta recibir los 14 bytes

  ax = Wire.read() << 8 | Wire.read();          // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay = Wire.read() << 8 | Wire.read();          // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = Wire.read() << 8 | Wire.read();          // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx = Wire.read() << 8 | Wire.read();          // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy = Wire.read() << 8 | Wire.read();          // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz = Wire.read() << 8 | Wire.read();          // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
