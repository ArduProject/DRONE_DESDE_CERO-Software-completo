/*
           v1.02
   https://arduproject.es/
  dronedesdecero@gmail.com
    Actualizado 02/2022

  Más información en los siguientes enlaces:
  - Lectura de mando radio control: https://arduproject.es/mando-rc-y-receptor-programacion-en-arduino-sketch/
  - Lectura de sensor MPU6050:      https://arduproject.es/mpu6050-y-su-programacion/
  - Control de establidad:          https://arduproject.es/control-de-estabilidad-y-pid/
  - Motores y ESC:                  https://arduproject.es/motores-esc-y-su-programacion-en-arduino/
*/

// --------------------------------------------------------------------------------
bool visu = 0;         // Visualizar variables por canal serie. En vuelo a 0!!
int visu_select = 4;   // 0: mando RC, 1: giro, 2: acc, 3: ang, 4: esc
bool MODO_vuelo = 1;   // 0: Modo acrobatico, 1: Modo estable (por defecto MODO_vuelo = 1)

#define usCiclo 6000   // Ciclo de ejecucion del software en microsegundos

#define pin_motor1 3        // Pin motor 1
#define pin_motor2 4        // Pin motor 2
#define pin_motor3 5        // Pin motor 3
#define pin_motor4 6        // Pin motor 4
#define pin_INT_Throttle 8  // Pin Throttle del mando RC
#define pin_INT_Yaw 7       // Pin Yaw del mando RC
#define pin_INT_Pitch 12    // Pin Pitch del mando RC
#define pin_INT_Roll 9      // Pin Roll del mando RC
#define pin_LED_rojo1 10    // Pin LED rojo 1      
#define pin_LED_rojo2 13    // Pin LED rojo 2    
#define pin_LED_azul 11     // Pin LED azul    
#define pin_LED_naranja A2  // Pin LED naranja    
// --------------------------------------------------------------------------------

#include <EnableInterrupt.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

// AJUSTE DE PIDs
// Modificar estos parámetros apara ajustar los PID
float Roll_ang_Kp  = 0.5, Roll_ang_Ki  = 0.05, Roll_ang_Kd  = 10;
float Pitch_ang_Kp = 0.5, Pitch_ang_Ki = 0.05, Pitch_ang_Kd = 10;
float Pitch_W_Kp   = 2,   Pitch_W_Ki   = 0.02, Pitch_W_Kd   = 0;
float Roll_W_Kp    = 2,   Roll_W_Ki    = 0.02, Roll_W_Kd    = 0;
float Yaw_W_Kp     = 1,   Yaw_W_Ki     = 0.05, Yaw_W_Kd     = 0;

int PID_W_sat1   = 380;  // Limitar parte integral PID velocidad
int PID_W_sat2   = 380;  // Limitar salida del PID velocidad
int PID_ang_sat1 = 130;  // Limitar parte integral PID ángulo
int PID_ang_sat2 = 130;  // Limitar salida del PID ángulo

float PID_ang_Pitch_error, PID_ang_Pitch_P, PID_ang_Pitch_I, PID_ang_Pitch_D, PID_ang_Pitch_OUT;
float PID_ang_Roll_error, PID_ang_Roll_P, PID_ang_Roll_I, PID_ang_Roll_D, PID_ang_Roll_OUT;
float PID_ang_Yaw_error, PID_ang_Yaw_P, PID_ang_Yaw_I, PID_ang_Yaw_D, PID_ang_Yaw_OUT;
float PID_W_Pitch_error, PID_W_Pitch_P, PID_W_Pitch_I, PID_W_Pitch_D, PID_W_Pitch_OUT;
float PID_W_Roll_error, PID_W_Roll_P, PID_W_Roll_I, PID_W_Roll_D, PID_W_Roll_OUT;
float PID_W_Yaw_error, PID_W_Yaw_P, PID_W_Yaw_I, PID_W_Yaw_D, PID_W_Yaw_OUT;
float PID_W_Pitch_consigna, PID_W_Roll_consigna;

// AJUSTE MANDO RC - THROTLLE
const int us_max_Throttle_adj = 1800;
const int us_min_Throttle_adj = 970;
const float us_max_Throttle_raw = 2004;  // <-- Si la entrada Throttle está invertida sustituir este valor
const float us_min_Throttle_raw = 1116;  // <-- por este y viceversa

// AJUSTE MANDO RC - PITCH
const float us_max_Pitch_raw = 1952;
const float us_min_Pitch_raw = 992;
const int us_max_Pitch_adj = -30;  // <-- Si la entrada Pitch está invertida sustituir este valor
const int us_min_Pitch_adj = 30;   // <-- por este y viceversa

// AJUSTE MANDO RC - ROLL
const float us_max_Roll_raw = 1960;
const float us_min_Roll_raw = 992;
const int us_max_Roll_adj = 30;    // <-- Si la entrada Roll está invertida sustituir este valor
const int us_min_Roll_adj = -30;   // <-- por este y viceversa

// AJUSTE MANDO RC - YAW
const float us_max_Yaw_raw = 1928;
const float us_min_Yaw_raw = 972;
const int us_max_Yaw_adj = 30;     // <-- Si la entrada Yaw está invertida sustituir este valor
const int us_min_Yaw_adj = -30;    // <-- por este y viceversa

// MPU6050
#define MPU6050_adress 0x68
float angulo_pitch, angulo_roll, angulo_yaw, angulo_pitch_acc, angulo_roll_acc, temperature;
float angulo_pitch_ant, angulo_roll_ant, angulo_yaw_ant;
int gx, gy, gz, gyro_Z, gyro_X, gyro_Y, gyro_X_ant, gyro_Y_ant, gyro_Z_ant;
float gyro_X_cal, gyro_Y_cal, gyro_Z_cal;
float ax, ay, az, acc_X_cal, acc_Y_cal, acc_Z_cal, acc_total_vector;
bool set_gyro_angles, accCalibOK = false;
float tiempo_ejecucion_MPU6050, tiempo_MPU6050_1;

// TIEMPOS
long loop_timer, loop_timer1, tiempo_motores_start, tiempo_ON, tiempo_1, tiempo_2;

// LECTURA TENSIÓN DE BATERÍA
bool LOW_BAT_WARING = false;
float tension_bateria, lectura_ADC;
int LOW_BAT_WARING_cont;

// OTROS
int LED_contador;

/// SEÑALES PWM
float ESC1_us, ESC2_us, ESC3_us, ESC4_us;

// MANDO RC
float RC_Throttle_filt, RC_Pitch_filt, RC_Yaw_filt, RC_Roll_filt;
float RC_Throttle_consigna, RC_Pitch_consigna, RC_Roll_consigna, RC_Yaw_consigna;

// LEER MANDO RC - TROTTLE
volatile long Throttle_HIGH_us;
volatile int RC_Throttle_raw;
void INT_Throttle() {
  if (digitalRead(pin_INT_Throttle) == HIGH)Throttle_HIGH_us = micros();
  if (digitalRead(pin_INT_Throttle) == LOW) RC_Throttle_raw  = micros() - Throttle_HIGH_us;
}

// INTERRUPCIÓN MANDO RC - PITCH
volatile long Pitch_HIGH_us;
volatile int RC_Pitch_raw;
void INT_Pitch() {
  if (digitalRead(pin_INT_Pitch) == HIGH)Pitch_HIGH_us = micros();
  if (digitalRead(pin_INT_Pitch) == LOW) RC_Pitch_raw  = micros() - Pitch_HIGH_us;
}

// INTERRUPCIÓN MANDO RC - ROLL
volatile long Pitch_Roll_us;
volatile int RC_Roll_raw;
void INT_Roll() {
  if (digitalRead(pin_INT_Roll) == HIGH)Pitch_Roll_us = micros();
  if (digitalRead(pin_INT_Roll) == LOW) RC_Roll_raw   = micros() - Pitch_Roll_us;
}

// INTERRUPCIÓN MANDO RC - YAW
volatile long Pitch_Yaw_us;
volatile int RC_Yaw_raw;
void INT_Yaw() {
  if (digitalRead(pin_INT_Yaw) == HIGH)Pitch_Yaw_us = micros();
  if (digitalRead(pin_INT_Yaw) == LOW) RC_Yaw_raw   = micros() - Pitch_Yaw_us;
}

void setup() {
  // Iniciar I2C y LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();

  if (visu == 1) {
    // Solo iniciamos el monitor serie y aguno de los modos de visualización está activo
    Serial.begin(115200);
    // Avisar de que el modo visualización está activo encendiendo el LED naranja
    analogWrite(pin_LED_naranja, 255);
  }

  // Declaración de LEDs
  pinMode(pin_LED_naranja, OUTPUT); // Led naranja --> Batería baja
  pinMode(pin_LED_azul, OUTPUT);    // Led azul    --> Ciclo (parpadeo)
  pinMode(pin_LED_rojo1, OUTPUT);   // Led rojo 1  --> Error MPU6050
  pinMode(pin_LED_rojo2, OUTPUT);   // Led rojo 2  --> Tiempo de ciclo o de 1ms excedido

  // MandoRC: declaración de interrupciones
  pinMode(pin_INT_Yaw, INPUT_PULLUP);                
  enableInterrupt(pin_INT_Yaw, INT_Yaw, CHANGE);
  pinMode(pin_INT_Throttle, INPUT_PULLUP);           
  enableInterrupt(pin_INT_Throttle, INT_Throttle, CHANGE);
  pinMode(pin_INT_Pitch, INPUT_PULLUP);              
  enableInterrupt(12, INT_Pitch, CHANGE);
  pinMode(pin_INT_Roll, INPUT_PULLUP);                 
  enableInterrupt(pin_INT_Roll, INT_Roll, CHANGE);

  // Declaración de los pines de los motores
  pinMode(pin_motor1, OUTPUT);  //Motor 1
  pinMode(pin_motor2, OUTPUT);  //Motor 2
  pinMode(pin_motor3, OUTPUT);  //Motor 3
  pinMode(pin_motor4, OUTPUT);  //Motor 4
  // Forzar los pines a estado LOW
  digitalWrite(pin_motor1, LOW);
  digitalWrite(pin_motor2, LOW);
  digitalWrite(pin_motor3, LOW);
  digitalWrite(pin_motor4, LOW);

  // Para poder avanzar hay que encender el mando y bajar Throttle al mínimo
  digitalWrite(pin_LED_rojo2, HIGH);
  lcd.setCursor(0, 0);
  lcd.print("Encender mando");
  lcd.setCursor(0, 1);
  if (MODO_vuelo == 1) lcd.print("-MODO Estable-");
  else lcd.print("-MODO Acro-");
  while (RC_Throttle_consigna < 950 || RC_Throttle_consigna > 1050) RC_procesar();
  digitalWrite(pin_LED_rojo2, LOW);

  MPU6050_iniciar();         // Iniciar sensor MPU6050
  MPU6050_calibrar();        // Calibrar sensor MPU6050
  Lectura_tension_bateria(); // Leer tension de batería

  // Para entrar el loop principal hay que mover el stick de Roll a la derecha
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ROLL -->");
  while (RC_Roll_consigna < 10)RC_procesar();

  lcd.clear();
  loop_timer = micros();
}

void loop() {
  // Si se supera el tiempo de ciclo, se activa el LED rojo 2
  if (micros() - loop_timer > usCiclo + 50) digitalWrite(pin_LED_rojo2, HIGH);
  // Comienzo de un nuevo ciclo
  while (micros() - loop_timer < usCiclo);
  // Registrar instante de comienzo del ciclo
  loop_timer = micros();

  PWM();                           // Generar señales PWM para los motores
  MPU6050_leer();                  // Leer sensor MPU6050
  MPU6050_procesar();              // Procesar datos del sensor MPU6050
  if (MODO_vuelo == 1)PID_ang();   // Obtener salida de los PID de inclinación
  PID_w();                         // Obtener salida de los PID de velocidad
  Modulador();                     // Modulador o generador de señales para PWM

  // Guardamos las lecturas del sensor MPU6050 para el siguiente ciclo (necesario para los PID)
  angulo_pitch_ant = angulo_pitch;
  angulo_roll_ant  = angulo_roll;
  angulo_yaw_ant   = angulo_yaw;
  gyro_X_ant = gyro_X; // Pitch
  gyro_Y_ant = gyro_Y; // Roll
  gyro_Z_ant = gyro_Z; // Yaw

  // Visualización de variables
  if (visu == 1)Visualizaciones();
}

// Iniciar sensor MPU6050
void MPU6050_iniciar() {
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x6B);                                                          //Registro 6B hex)
  Wire.write(0x00);                                                          //00000000 para activar giroscopio
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1B);                                                          //Register 1B hex
  Wire.write(0x08);                                                          //Girscopio a 500dps (full scale)
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1C);                                                          //Register (1A hex)
  Wire.write(0x10);                                                          //Acelerometro a  +/- 8g (full scale range)
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_adress, 1);
  while (Wire.available() < 1);

  // Si hay un error en el sensor MPU6050 avisamos y enclavamos el programa
  if (Wire.read() != 0x08) {
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("MPU6050 error");
    while (1) {
      digitalWrite(pin_LED_rojo1, LOW);
      delay(500);
      digitalWrite(pin_LED_rojo1, HIGH);
      delay(500);
    }
  }

  // Activar y configurar filtro pasa bajos LPF que incorpora el sensor
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1A);
  Wire.write(0x04);
  Wire.endTransmission();

  /*
    Frecuencia de corte del filtro pasa bajos:
    256Hz(0ms):0x00
    188Hz(2ms):0x01
    98Hz(3ms):0x02
    42Hz(4.9ms):0x03
    20Hz(8.5ms):0x04
    10Hz(13.8ms):0x05
    5Hz(19ms):0x06
  */
}

// Calibrar sensor MPU6050
void MPU6050_calibrar() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calib. MPU6050");

  // Calibrar giroscopio tomando 3000 muestras
  for (int cal_int = 0; cal_int < 3000 ; cal_int ++) {
    MPU6050_leer();
    gyro_X_cal += gx;
    gyro_Y_cal += gy;
    gyro_Z_cal += gz;
    acc_X_cal  += ax;
    acc_Y_cal  += ay;
    acc_Z_cal  += az;
    delayMicroseconds(20);
  }
  // Calcular el valor medio de las 3000 muestras
  gyro_X_cal = gyro_X_cal / 3000;
  gyro_Y_cal = gyro_Y_cal / 3000;
  gyro_Z_cal = gyro_Z_cal / 3000;
  acc_X_cal  = acc_X_cal  / 3000;
  acc_Y_cal  = acc_Y_cal  / 3000;
  acc_Z_cal  = acc_Z_cal  / 3000;
  accCalibOK = true;
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
  temperature = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  gx = Wire.read() << 8 | Wire.read();          // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  gy = Wire.read() << 8 | Wire.read();          // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  gz = Wire.read() << 8 | Wire.read();          // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)

  // Restar valores de calibracion del acelerómetro
  if (accCalibOK == true) {
    ax -= acc_X_cal;
    ay -= acc_Y_cal;
    az -= acc_Z_cal;
    az = az + 4096;
  }
}

// Cálculo de velocidad angular (º/s) y ángulo (º)
void MPU6050_procesar() {
  // Restar valores de calibración del acelerómetro y calcular
  // velocidad angular en º/s. Leer 65.5 en raw equivale a 1º/s
  gyro_X = (gx - gyro_X_cal) / 65.5;
  gyro_Y = (gy - gyro_Y_cal) / 65.5;
  gyro_Z = (gz - gyro_Z_cal) / 65.5;

  // Calculamos exactamente cuánto tiempo ha pasado desde que se ha ejecutado el cálculo del ángulo.
  // Al tener señales PWM variables entre 1 y 2ms, este cálculo del ángulo no se ejecuta siempre
  // con un periodo constante.
  tiempo_ejecucion_MPU6050 = (micros() - tiempo_MPU6050_1) / 1000;

  // Calcular ángulo de inclinación con datos de giroscopio:
  // velocidad (º/s) * tiempo (s) = grados de inclinación (º)
  angulo_pitch += gyro_X * tiempo_ejecucion_MPU6050 / 1000;
  angulo_roll  += gyro_Y * tiempo_ejecucion_MPU6050 / 1000;
  // 0.000000266 = tiempo_ejecucion / 1000 / 65.5 * PI / 180
  angulo_pitch += angulo_roll  * sin((gz - gyro_Z_cal) * tiempo_ejecucion_MPU6050 * 0.000000266);
  angulo_roll  -= angulo_pitch * sin((gz - gyro_Z_cal) * tiempo_ejecucion_MPU6050 * 0.000000266);
  tiempo_MPU6050_1 = micros();

  // Calcular vector de aceleración
  // 57.2958 = Conversion de radianes a grados 180/PI
  acc_total_vector = sqrt(pow(ay, 2) + pow(ax, 2) + pow(az, 2));
  angulo_pitch_acc = asin((float)ay / acc_total_vector) * 57.2958;
  angulo_roll_acc  = asin((float)ax / acc_total_vector) * -57.2958;

  if (set_gyro_angles) {
    // Filtro complementario
    angulo_pitch = angulo_pitch * 0.995 + angulo_pitch_acc * 0.005;   // Angulo Pitch de inclinacion
    angulo_roll  = angulo_roll  * 0.995 + angulo_roll_acc  * 0.005;   // Angulo Roll de inclinacion
  }
  else {
    angulo_pitch = angulo_pitch_acc;
    angulo_roll  = angulo_roll_acc;
    set_gyro_angles = true;
  }
}

void RC_procesar() {
  //  Filtrado de lecturas raw del mando RC
  RC_Throttle_filt = RC_Throttle_filt * 0.9 + RC_Throttle_raw * 0.1;
  RC_Pitch_filt    = RC_Pitch_filt * 0.9 + RC_Pitch_raw * 0.1;
  RC_Roll_filt     = RC_Roll_filt  * 0.9 + RC_Roll_raw  * 0.1;
  RC_Yaw_filt      = RC_Yaw_filt   * 0.9 + RC_Yaw_raw   * 0.1;

  // Mapeo de señales del mando RC
  RC_Throttle_consigna = map(RC_Throttle_filt, us_min_Throttle_raw, us_max_Throttle_raw, us_min_Throttle_adj, us_max_Throttle_adj);
  RC_Pitch_consigna    = map(RC_Pitch_filt, us_min_Pitch_raw, us_max_Pitch_raw, us_min_Pitch_adj, us_max_Pitch_adj);
  RC_Roll_consigna     = map(RC_Roll_filt, us_min_Roll_raw, us_max_Roll_raw, us_min_Roll_adj, us_max_Roll_adj);
  RC_Yaw_consigna      = map(RC_Yaw_filt, us_min_Yaw_raw, us_max_Yaw_raw, us_min_Yaw_adj, us_max_Yaw_adj);

  // Si las lecturas son cercanas a 0, las forzamos a 0 para evitar inclinar el drone por error
  if (RC_Pitch_consigna < 3 && RC_Pitch_consigna > -3)RC_Pitch_consigna = 0;
  if (RC_Roll_consigna  < 3 && RC_Roll_consigna  > -3)RC_Roll_consigna  = 0;
  if (RC_Yaw_consigna   < 3 && RC_Yaw_consigna   > -3)RC_Yaw_consigna   = 0;
}

void Modulador() {
  // Si el Throttle es menos a 1300us, el control de estabilidad se desactiva. La parte integral
  // de los controladores PID se fuerza a 0.
  if (RC_Throttle_consigna <= 1300) {
    PID_W_Pitch_I = 0;
    PID_W_Roll_I = 0;
    PID_W_Yaw_I  = 0;
    PID_ang_Pitch_I = 0;
    PID_ang_Roll_I = 0;

    ESC1_us = RC_Throttle_consigna;
    ESC2_us = RC_Throttle_consigna;
    ESC3_us = RC_Throttle_consigna;
    ESC4_us = RC_Throttle_consigna;

    // Si lo motores giran con el stick de Throttle al mínimo, recudir el valor de 950us
    if (ESC1_us < 1000) ESC1_us = 950;
    if (ESC2_us < 1000) ESC2_us = 950;
    if (ESC3_us < 1000) ESC3_us = 950;
    if (ESC4_us < 1000) ESC4_us = 950;
  }

  // Si el throttle es mayor a 1300us, el control de estabilidad se activa.
  else {
    // Limitar throttle a 1800 para dejar margen a los PID
    if (RC_Throttle_consigna > 1800)RC_Throttle_consigna = 1800;

    // Modulador
    ESC1_us = RC_Throttle_consigna + PID_W_Pitch_OUT - PID_W_Roll_OUT - PID_W_Yaw_OUT; // Motor 1
    ESC2_us = RC_Throttle_consigna + PID_W_Pitch_OUT + PID_W_Roll_OUT + PID_W_Yaw_OUT; // Motor 2
    ESC3_us = RC_Throttle_consigna - PID_W_Pitch_OUT + PID_W_Roll_OUT - PID_W_Yaw_OUT; // Motor 3
    ESC4_us = RC_Throttle_consigna - PID_W_Pitch_OUT - PID_W_Roll_OUT + PID_W_Yaw_OUT; // Motor 4
    //    ESC1_us = RC_Throttle_filt; // Solo para testeos
    //    ESC2_us = RC_Throttle_filt;
    //    ESC3_us = RC_Throttle_filt;
    //    ESC4_us = RC_Throttle_filt;

    // Evitamos que alguno de los motores de detenga completamente en pleno vuelo
    if (ESC1_us < 1100) ESC1_us = 1100;
    if (ESC2_us < 1100) ESC2_us = 1100;
    if (ESC3_us < 1100) ESC3_us = 1100;
    if (ESC4_us < 1100) ESC4_us = 1100;
    // Evitamos mandar consignas mayores a 2000us a los motores
    if (ESC1_us > 2000) ESC1_us = 2000;
    if (ESC2_us > 2000) ESC2_us = 2000;
    if (ESC3_us > 2000) ESC3_us = 2000;
    if (ESC4_us > 2000) ESC4_us = 2000;
  }
}

void PWM() {
  // Para generar las 4 señales PWM, el primer paso es poner estas señales a 1 (HIGH).
  digitalWrite(pin_motor1, HIGH);
  digitalWrite(pin_motor2, HIGH);
  digitalWrite(pin_motor3, HIGH);
  digitalWrite(pin_motor4, HIGH);
  tiempo_motores_start = micros();

  // ------------------ ¡¡1ms max!! ------------------
  tiempo_1 = micros();

  RC_procesar();             // Leer mando RC
  LED_blink();               // LED parpadeo
  Lectura_tension_bateria(); // Leer Vbat

  // Si la duracion entre tiempo_1 y tiempo_2 ha sido mayor de 900us, encender LED de aviso.
  // Nunca hay que sobrepasar 1ms de tiempo en estado HIGH.
  tiempo_2 = micros();
  tiempo_ON = tiempo_2 - tiempo_1;
  if (tiempo_ON > 900) digitalWrite(pin_LED_rojo2, HIGH);
  // ------------------ ¡¡1ms max!! ------------------

  // Pasamos las señales PWM a estado LOW cuando haya transcurrido el tiempo definido en las variables ESCx_us
  while (digitalRead(pin_motor1) == HIGH || digitalRead(pin_motor2) == HIGH || digitalRead(pin_motor3) == HIGH || digitalRead(pin_motor4) == HIGH) {
    if (tiempo_motores_start + ESC1_us <= micros()) digitalWrite(pin_motor1, LOW);
    if (tiempo_motores_start + ESC2_us <= micros()) digitalWrite(pin_motor2, LOW);
    if (tiempo_motores_start + ESC3_us <= micros()) digitalWrite(pin_motor3, LOW);
    if (tiempo_motores_start + ESC4_us <= micros()) digitalWrite(pin_motor4, LOW);
  }
}

// PID ángulo
void PID_ang() {
  // PID ángulo - PITCH
  PID_ang_Pitch_error = RC_Pitch_consigna - angulo_pitch;                        // Error entre lectura y consigna
  PID_ang_Pitch_P  = Pitch_ang_Kp  * PID_ang_Pitch_error;                        // Parte proporcional
  PID_ang_Pitch_I += (Pitch_ang_Ki * PID_ang_Pitch_error);                       // Parte integral (sumatorio del error en el tiempo)
  PID_ang_Pitch_I  = constrain(PID_ang_Pitch_I, -PID_ang_sat1, PID_ang_sat1);    // Limitar parte integral
  PID_ang_Pitch_D  = Pitch_ang_Kd * (angulo_pitch - angulo_pitch_ant);           // Parte derivativa (diferencia entre el error actual y el anterior)

  PID_ang_Pitch_OUT =  PID_ang_Pitch_P + PID_ang_Pitch_I + PID_ang_Pitch_D;      // Salida PID
  PID_ang_Pitch_OUT = constrain(PID_ang_Pitch_OUT, -PID_ang_sat2, PID_ang_sat2); // Limitar salida del PID

  // PID ángulo - ROLL
  PID_ang_Roll_error = RC_Roll_consigna - angulo_roll;                           // Error entre lectura y consigna
  PID_ang_Roll_P  = Roll_ang_Kp  * PID_ang_Roll_error;                           // Parte proporcional
  PID_ang_Roll_I += (Roll_ang_Ki * PID_ang_Roll_error);                          // Parte integral (sumatorio del error en el tiempo)
  PID_ang_Roll_I  = constrain(PID_ang_Roll_I, -PID_ang_sat1, PID_ang_sat1);      // Limitar parte integral
  PID_ang_Roll_D  = Roll_ang_Kd * (angulo_roll - angulo_roll_ant);               // Parte derivativa (diferencia entre el error actual y el anterior)

  PID_ang_Roll_OUT = PID_ang_Roll_P + PID_ang_Roll_I + PID_ang_Roll_D;           // Salida PID
  PID_ang_Roll_OUT = constrain(PID_ang_Roll_OUT, -PID_ang_sat2, PID_ang_sat2);   // Limitar salida del PID
}

// PID velocidad angular
void PID_w() {
  // En funcion del modo de vuelo que hayamos seleccionado, las consignas de los PID serán diferentes
  if (MODO_vuelo == 0) {
    // En modo acrobático solo controlamos la velocidad de cada eje (un PID por eje). La consigna del PID se da en º/s
    // y viene directamente del mando RC
    PID_W_Pitch_consigna = RC_Pitch_consigna;
    PID_W_Roll_consigna  = RC_Roll_consigna;
  }
  else {
    // En modo estable las consignas de los PID de velocidad vienen de las salidas de los PID de ángulo
    PID_W_Pitch_consigna = PID_ang_Pitch_OUT;
    PID_W_Roll_consigna  = PID_ang_Roll_OUT;
  }

  // PID velocidad - PITCH
  PID_W_Pitch_error = PID_W_Pitch_consigna - gyro_X;                       // Error entre lectura y consigna
  PID_W_Pitch_P  = Pitch_W_Kp  * PID_W_Pitch_error;                        // Parte proporcional
  PID_W_Pitch_I += (Pitch_W_Ki * PID_W_Pitch_error);                       // Parte integral (sumatorio del error en el tiempo)
  PID_W_Pitch_I  = constrain(PID_W_Pitch_I, -PID_W_sat1, PID_W_sat1);      // Limitar parte integral
  PID_W_Pitch_D  = Pitch_W_Kd * (gyro_X - gyro_X_ant);                     // Parte derivativa (diferencia entre el error actual y el anterior)

  PID_W_Pitch_OUT = PID_W_Pitch_P + PID_W_Pitch_I + PID_W_Pitch_D;         // Salida PID
  PID_W_Pitch_OUT = constrain(PID_W_Pitch_OUT, -PID_W_sat2, PID_W_sat2);   // Limitar salida del PID

  // PID velocidad - ROLL
  PID_W_Roll_error = PID_W_Roll_consigna - gyro_Y;                         // Error entre lectura y consigna
  PID_W_Roll_P  = Roll_W_Kp  * PID_W_Roll_error;                           // Parte proporcional
  PID_W_Roll_I += (Roll_W_Ki * PID_W_Roll_error);                          // Parte integral (sumatorio del error en el tiempo)
  PID_W_Roll_I  = constrain(PID_W_Roll_I, -PID_W_sat1, PID_W_sat1);        // Limitar parte integral
  PID_W_Roll_D  = Roll_W_Kd * (gyro_Y - gyro_Y_ant);                       // Parte derivativa (diferencia entre el error actual y el anterior)

  PID_W_Roll_OUT = PID_W_Roll_P + PID_W_Roll_I + PID_W_Roll_D;             // Salida PID
  PID_W_Roll_OUT = constrain(PID_W_Roll_OUT, -PID_W_sat2, PID_W_sat2);     // Limitar salida del PID

  // PID velocidad - YAW
  PID_W_Yaw_error = RC_Yaw_consigna - gyro_Z;                              // Error entre lectura y consigna
  PID_W_Yaw_P  = Yaw_W_Kp  * PID_W_Yaw_error;                              // Parte proporcional
  PID_W_Yaw_I += (Yaw_W_Ki * PID_W_Yaw_error);                             // Parte integral (sumatorio del error en el tiempo)
  PID_W_Yaw_I  = constrain(PID_W_Yaw_I, -PID_W_sat1, PID_W_sat1);          // Limitar parte integral
  PID_W_Yaw_D  = Yaw_W_Kd * (gyro_Z - gyro_Z_ant);                         // Parte derivativa (diferencia entre el error actual y el anterior)

  PID_W_Yaw_OUT = PID_W_Yaw_P + PID_W_Yaw_I + PID_W_Yaw_D;                 // Salida PID
  PID_W_Yaw_OUT = constrain(PID_W_Yaw_OUT, -PID_W_sat2, PID_W_sat2);       // Limitar salida del PID
}

void Lectura_tension_bateria() {
  // La tensión de batería solo se lee con Throttle inferior a 1100us
  if (RC_Throttle_consigna < 1100) {
    // Leer entrada analógica
    lectura_ADC = analogRead(A6);
    tension_bateria = 2.95 * (lectura_ADC * 4.92  / 1023);

    // Si la tension de batería es inferior a 10.5V durante un número de ciclos consecutivos
    // se activa la señal de batería baja
    if (tension_bateria < 10.5 && LOW_BAT_WARING == false) {
      LOW_BAT_WARING_cont++;
      if (LOW_BAT_WARING_cont > 30)LOW_BAT_WARING = true;
    }
    else LOW_BAT_WARING_cont = 0;
  }
}

// Función para hacer parpadear el LED al entrar en el loop() principal
void LED_blink() {
  if (LED_contador % 20 == 0) {
    if (digitalRead(pin_LED_azul) == LOW) digitalWrite(pin_LED_azul, HIGH);
    else digitalWrite(pin_LED_azul, LOW);

    if (digitalRead(pin_LED_azul) == HIGH && LOW_BAT_WARING == 1)analogWrite(pin_LED_naranja, 0);
    else analogWrite(pin_LED_naranja, 255);
  }
  LED_contador++;
}

// Visualizar variables por Monitor Serie
// NOTA: al visualizar variables por el Monitor Serie es posible que se sobrepase el tiempo de ciclo establecido
// debido al tiempo extra que conlleva visualizar las variables. Esta es la razón de desactivar esta opción en
// vuelo.
void Visualizaciones() {

  // Visualizar variables por canal serie
  // Hay que seleccionar qué variable visualizar con visu_select
  if (visu == 1) {
    if (visu_select == 0) {
      Serial.print(RC_Pitch_consigna);
      Serial.print("\t");
      Serial.print(RC_Roll_consigna);
      Serial.print("\t");
      Serial.print(RC_Yaw_consigna);
      Serial.print("\t");
      Serial.println(RC_Throttle_consigna);
    }

    if (visu_select == 1) {
      Serial.print((gx - gyro_X_cal) / 16.4);
      Serial.print("\t");
      Serial.print((gy - gyro_Y_cal) / 16.4);
      Serial.print("\t");
      Serial.println((gz - gyro_Z_cal) / 16.4);
    }

    if (visu_select == 2) {
      Serial.print(ax / 4096);
      Serial.print("\t");
      Serial.print(ay / 4096);
      Serial.print("\t");
      Serial.println(az / 4096);
    }

    if (visu_select == 3) {
      Serial.print(angulo_pitch);
      Serial.print("\t");
      Serial.println(angulo_roll);
    }

    if (visu_select == 4) {
      Serial.print(ESC1_us);
      Serial.print("\t");
      Serial.print(ESC2_us);
      Serial.print("\t");
      Serial.print(ESC3_us);
      Serial.print("\t");
      Serial.println(ESC4_us);

      Pitch_W_Ki = 0;
      Roll_W_Ki  = 0;
      Yaw_W_Ki   = 0;
      Pitch_ang_Ki = 0;
      Roll_ang_Ki  = 0;
    }

    if (visu_select == 999) {
      Serial.println(tiempo_ejecucion_MPU6050);
    }
  }
}
