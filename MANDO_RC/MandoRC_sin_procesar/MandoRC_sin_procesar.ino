/*
   Leer emisora radio control (datos raw o sin procesar)
   Más información en https://arduproject.es/mando-rc-y-receptor-programacion-en-arduino-sketch/
*/

// Declaración de pines
#define pin_INT_Throttle 8 // Pin Throttle del mando RC 
#define pin_INT_Yaw 7      // Pin Yaw del mando RC 
#define pin_INT_Pitch 12   // Pin Pitch del mando RC
#define pin_INT_Roll 9     // Pin Roll del mando RC

#include <EnableInterrupt.h>
long loop_timer, tiempo_ejecucion;

// INTERRUPCIÓN MANDO RC --> THROTTLE
volatile long Throttle_HIGH_us;
volatile int RC_Throttle_raw;
void INT_Throttle() {
  if (digitalRead(pin_INT_Throttle) == HIGH) Throttle_HIGH_us = micros();
  if (digitalRead(pin_INT_Throttle) == LOW)  RC_Throttle_raw  = micros() - Throttle_HIGH_us;
}

// INTERRUPCIÓN MANDO RC --> PITCH
volatile long Pitch_HIGH_us;
volatile int RC_Pitch_raw;
void INT_Pitch() {
  if (digitalRead(pin_INT_Pitch) == HIGH) Pitch_HIGH_us = micros();
  if (digitalRead(pin_INT_Pitch) == LOW)  RC_Pitch_raw  = micros() - Pitch_HIGH_us;
}

// INTERRUPCIÓN MANDO RC --> ROLL
volatile long Roll_HIGH_us;
volatile int RC_Roll_raw;
void INT_Roll() {
  if (digitalRead(pin_INT_Roll) == HIGH) Roll_HIGH_us = micros();
  if (digitalRead(pin_INT_Roll) == LOW)  RC_Roll_raw  = micros() - Roll_HIGH_us;
}

// INTERRUPCIÓN MANDO RC --> YAW
volatile long Yaw_HIGH_us;
volatile int RC_Yaw_raw;
void INT_Yaw() {
  if (digitalRead(pin_INT_Yaw) == HIGH) Yaw_HIGH_us = micros();
  if (digitalRead(pin_INT_Yaw) == LOW)  RC_Yaw_raw  = micros() - Yaw_HIGH_us;
}

void setup() {
  // Declaración de interrupciones
  pinMode(pin_INT_Yaw, INPUT_PULLUP);                   // YAW
  enableInterrupt(pin_INT_Yaw, INT_Yaw, CHANGE);
  pinMode(pin_INT_Throttle, INPUT_PULLUP);              // POTENCIA
  enableInterrupt(pin_INT_Throttle, INT_Throttle, CHANGE);
  pinMode(pin_INT_Pitch, INPUT_PULLUP);                 // PITCH
  enableInterrupt(pin_INT_Pitch, INT_Pitch, CHANGE);
  pinMode(pin_INT_Roll, INPUT_PULLUP);                  // ROLL
  enableInterrupt(pin_INT_Roll, INT_Roll, CHANGE);

  // Serial.begin()
  Serial.begin(115200);
}

void loop() {
  // Nuevo ciclo
  while (micros() - loop_timer < 10000);
  tiempo_ejecucion = (micros() - loop_timer) / 1000;
  loop_timer = micros();

  // Monitor Serie
  Serial.print(RC_Throttle_raw);
  Serial.print("\t");
  Serial.print(RC_Pitch_raw);
  Serial.print("\t");
  Serial.print(RC_Roll_raw);
  Serial.print("\t");
  Serial.println(RC_Yaw_raw);
}
