/*
   Mover motores
   Más información en https://arduproject.es/motores-esc-y-su-programacion-en-arduino/
*/

// !!!!!!!!!!!!!! RETIRAR LAS HELICES - SIN HELICES !!!!!!!!!!!!!!!!!!!!!!!!!!
// Puede que este prodecimiento no sirva para todos los ESC/motores, consultad vuestra hoja de datos:
// 1) Con el Throttle al mínimo, conectar la batería y cerrar el interruptor para alimentar
//    la placa Arduino (no conectar el cable USB). 
// 2) Los motores emitiran unos pitidos pipipi... piiiii.
// 3) Terminado, se pueden girar los motores con el mando.

#define usCiclo 6000    // Ciclo de ejecucion de software en microsegundos (PWM)
#define pin_motor1 3    // Pin motor 1
#define pin_motor2 4    // Pin motor 2
#define pin_motor3 5    // Pin motor 3
#define pin_motor4 6    // Pin motor 4
#define pin_Throttle 8  // Pin throttle del mando RC
#define pin_LED_rojo 13 // Pin LED rojo

#include <EnableInterrupt.h>

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

  // Monitor serie
  Serial.println(RC_Throttle_consigna);
}
