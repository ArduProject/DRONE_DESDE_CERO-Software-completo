/*
   Leer sensor MPU6050 (datos raw o sin procesar)
   Más información en https://arduproject.es/mpu6050-y-su-programacion/
*/

#define usCiclo 5000  // Ciclo de ejecución de software en microsegundos
#include <Wire.h>

// MPU6050
#define MPU6050_adress 0x68
float gyro_Z, gyro_X, gyro_Y, temperature, gyro_X_cal, gyro_Y_cal, gyro_Z_cal;
int gx, gy, gz, cal_int;
float acc_total_vector, ax, ay, az;
bool set_gyro_angles, accCalibOK  = false;
float acc_X_cal, acc_Y_cal, acc_Z_cal, angulo_pitch_acc, angulo_roll_acc, angulo_pitch, angulo_roll;

long tiempo_ejecucion, loop_timer;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Iniciar sensor MPU6050
  MPU6050_iniciar();

  // Calibrar giroscopio y acelerómetro. El sensor tiene que estar inmovil y en una supercifie plana.
  // Leer los datos del MPU6050 3000 veces y calcular el valor medio
  for (cal_int = 0; cal_int < 3000 ; cal_int ++) {
    MPU6050_leer();   // Leer sensor MPU6050
    gyro_X_cal += gx;
    gyro_Y_cal += gy;
    gyro_Z_cal += gz;
    acc_X_cal  += ax;
    acc_Y_cal  += ay;
    acc_Z_cal  += az;
    delayMicroseconds(50);
  }
  gyro_X_cal = gyro_X_cal / 3000;
  gyro_Y_cal = gyro_Y_cal / 3000;
  gyro_Z_cal = gyro_Z_cal / 3000;
  acc_X_cal  = acc_X_cal  / 3000;
  acc_Y_cal  = acc_Y_cal  / 3000;
  acc_Z_cal  = acc_Z_cal  / 3000;
  accCalibOK = true;

  loop_timer = micros();
}

void loop() {

  // Nuevo ciclo
  while (micros() - loop_timer < usCiclo);
  tiempo_ejecucion = (micros() - loop_timer) / 1000;
  loop_timer = micros();

  MPU6050_leer();     // Leer sensor MPU6050
  MPU6050_procesar(); // Procesar datos del sensor MPU6050

  // Monitor Serie
  Serial.print(angulo_pitch);
  Serial.print("\t");
  Serial.println(angulo_roll);
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

// Cálculo de velocidad angular (º/s) y ángulo (º)
void MPU6050_procesar() {

  // Restar valores de calibración del acelerómetro
  ax -= acc_X_cal;
  ay -= acc_Y_cal;
  az -= acc_Z_cal;
  az  = az + 4096;

  // Restar valores de calibración del giroscopio y calcular
  // velocidad angular en º/s. Leer 65.5 en raw equivale a 1º/s
  gyro_X = (gx - gyro_X_cal) / 65.5;
  gyro_Y = (gy - gyro_Y_cal) / 65.5;
  gyro_Z = (gz - gyro_Z_cal) / 65.5;

  // Calcular ángulo de inclinación con datos del giroscopio
  // 0.000000266 = tiempo_ejecucion / 1000 / 65.5 * PI / 180
  angulo_pitch += gyro_X * tiempo_ejecucion / 1000;
  angulo_roll  += gyro_Y * tiempo_ejecucion / 1000;
  angulo_pitch += angulo_roll  * sin((gz - gyro_Z_cal) * tiempo_ejecucion  * 0.000000266);
  angulo_roll  -= angulo_pitch * sin((gz - gyro_Z_cal) * tiempo_ejecucion  * 0.000000266);

  // Calcular vector de aceleración
  // 57.2958 = Conversion de radianes a grados 180/PI
  acc_total_vector  = sqrt(pow(ay, 2) + pow(ax, 2) + pow(az, 2));
  angulo_pitch_acc  = asin((float)ay / acc_total_vector) * 57.2958;
  angulo_roll_acc   = asin((float)ax / acc_total_vector) * -57.2958;

  // Filtro complementario
  if (set_gyro_angles) {
    angulo_pitch = angulo_pitch * 0.99 + angulo_pitch_acc * 0.01;
    angulo_roll  = angulo_roll  * 0.99 + angulo_roll_acc  * 0.01;
  }
  else {
    angulo_pitch = angulo_pitch_acc;
    angulo_roll  = angulo_roll_acc;
    set_gyro_angles = true;
  }
}
