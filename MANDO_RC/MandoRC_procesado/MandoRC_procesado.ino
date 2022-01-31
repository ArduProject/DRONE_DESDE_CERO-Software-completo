/*
   Leer emisora radio control y procesar las lecturas
   Más información en https://arduproject.es/mando-rc-y-receptor-programacion-en-arduino-sketch/
*/

// Declaración de pines
#define pin_INT_Throttle 8 // Pin Throttle del mando RC 
#define pin_INT_Yaw 7      // Pin Yaw del mando RC 
#define pin_INT_Pitch 12   // Pin Pitch del mando RC
#define pin_INT_Roll 9     // Pin Roll del mando RC

#include <EnableInterrupt.h>
long loop_timer, tiempo_ejecucion;
float RC_Throttle_consigna, RC_Pitch_consigna, RC_Roll_consigna, RC_Yaw_consigna;

// AJUSTE MANDO RC - THROTLLE
const int us_max_Throttle_adj = 2000;
const int us_min_Throttle_adj = 950;
const float us_max_Throttle_raw = 2004; // <-- Si teneis la entrada Throttle invertida sustituid este valor
const float us_min_Throttle_raw = 1116; // <-- por este y viceversa

// AJUSTE MANDO RC - PITCH
const float us_max_Pitch_raw = 1952;
const float us_min_Pitch_raw = 992;
const int us_max_Pitch_adj = -30;   // <-- Si teneis la entrada Pitch invertido sustituid este valor
const int us_min_Pitch_adj = 30;    // <-- por este y viceversa

// AJUSTE MANDO RC - ROLL
const float us_max_Roll_raw = 1960;
const float us_min_Roll_raw = 992;
const int us_max_Roll_adj = 30;     // <-- Si teneis la entrada Roll invertido sustituid este valor
const int us_min_Roll_adj = -30;    // <-- por este y viceversa

// AJUSTE MANDO RC - YAW
const float us_max_Yaw_raw = 1928;
const float us_min_Yaw_raw = 972;
const int us_max_Yaw_adj = 30;      // <-- Si teneis la entrada Yaw invertido sustituid este valor
const int us_min_Yaw_adj = -30;     // <-- por este y viceversa

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

  // Ecuaciones de procesamiento
  RC_Throttle_consigna = map(RC_Throttle_raw, us_min_Throttle_raw, us_max_Throttle_raw, us_min_Throttle_adj, us_max_Throttle_adj);
  RC_Pitch_consigna    = map(RC_Pitch_raw, us_min_Pitch_raw, us_max_Pitch_raw, us_min_Pitch_adj, us_max_Pitch_adj);
  RC_Roll_consigna     = map(RC_Roll_raw, us_min_Roll_raw, us_max_Roll_raw, us_min_Roll_adj, us_max_Roll_adj);
  RC_Yaw_consigna      = map(RC_Yaw_raw, us_min_Yaw_raw, us_max_Yaw_raw, us_min_Yaw_adj, us_max_Yaw_adj);
  
  // Monitor Serie
  Serial.print(RC_Throttle_consigna);
  Serial.print("\t");
  Serial.print(RC_Pitch_consigna);
  Serial.print("\t");
  Serial.print(RC_Roll_consigna);
  Serial.print("\t");
  Serial.println(RC_Yaw_consigna);
}
