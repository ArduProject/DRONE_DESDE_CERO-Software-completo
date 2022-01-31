/*
   Leer emisora radio control y calcular ancho de pulso máximo y mínimo
   Más información en https://arduproject.es/mando-rc-y-receptor-programacion-en-arduino-sketch/
*/

// Declaración de pines
#define pin_INT_Throttle 8 // Pin Throttle del mando RC 
#define pin_INT_Yaw 7      // Pin Yaw del mando RC 
#define pin_INT_Pitch 12   // Pin Pitch del mando RC
#define pin_INT_Roll 9     // Pin Roll del mando RC
#define pin_LED_rojo 13    // Pin LED rojo

#include <EnableInterrupt.h>
int RC_Throttle_raw_Max, RC_Pitch_raw_Max, RC_Yaw_raw_Max, RC_Roll_raw_Max;
int RC_Throttle_raw_Min, RC_Pitch_raw_Min, RC_Yaw_raw_Min, RC_Roll_raw_Min;

// INTERRUPCIÓN MANDO RC - THROTTLE
volatile long Throttle_HIGH_us;
volatile int RC_Throttle_raw;
void INT_Throttle() {
  if (digitalRead(pin_INT_Throttle) == HIGH) Throttle_HIGH_us = micros();
  if (digitalRead(pin_INT_Throttle) == LOW)  RC_Throttle_raw = micros() - Throttle_HIGH_us;
}

// INTERRUPCIÓN MANDO RC - PITCH
volatile long Pitch_HIGH_us;
volatile int RC_Pitch_raw;
void INT_Pitch() {
  if (digitalRead(pin_INT_Pitch) == HIGH) Pitch_HIGH_us = micros();
  if (digitalRead(pin_INT_Pitch) == LOW)  RC_Pitch_raw = micros() - Pitch_HIGH_us;
}

// INTERRUPCIÓN MANDO RC - ROLL
volatile long Roll_HIGH_us;
volatile int RC_Roll_raw;
void INT_Roll() {
  if (digitalRead(pin_INT_Roll) == HIGH) Roll_HIGH_us = micros();
  if (digitalRead(pin_INT_Roll) == LOW)  RC_Roll_raw = micros() - Roll_HIGH_us;
}

// INTERRUPCIÓN MANDO RC - YAW
volatile long Yaw_HIGH_us;
volatile int RC_Yaw_raw;
void INT_Yaw() {
  if (digitalRead(pin_INT_Yaw) == HIGH) Yaw_HIGH_us = micros();
  if (digitalRead(pin_INT_Yaw) == LOW)  RC_Yaw_raw = micros() - Yaw_HIGH_us;
}

void setup() {
  // Declaración de LEDs
  pinMode(pin_LED_rojo, OUTPUT);

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

  digitalWrite(pin_LED_rojo, HIGH);
  Serial.print("Encender mando RC");
  // Hasta no encender el mando no se sale de este bucle
  while (RC_Throttle_raw > 2500 || RC_Throttle_raw < 500) delay(100);
  digitalWrite(pin_LED_rojo, LOW);

  // El valor de pulso mínimo inicial se fija el 2100 para que no afecte al cálculo del valor mínimo
  RC_Throttle_raw_Min = 2100;
  RC_Pitch_raw_Min    = 2100;
  RC_Yaw_raw_Min      = 2100;
  RC_Roll_raw_Min     = 2100;
}

void loop() {

  // Calcular ancho de pulso máximo y mínimo
  if (RC_Throttle_raw > RC_Throttle_raw_Max)RC_Throttle_raw_Max = RC_Throttle_raw;
  if (RC_Throttle_raw < RC_Throttle_raw_Min)RC_Throttle_raw_Min = RC_Throttle_raw;

  if (RC_Pitch_raw > RC_Pitch_raw_Max)RC_Pitch_raw_Max = RC_Pitch_raw;
  if (RC_Pitch_raw < RC_Pitch_raw_Min)RC_Pitch_raw_Min = RC_Pitch_raw;

  if (RC_Roll_raw > RC_Roll_raw_Max)RC_Roll_raw_Max = RC_Roll_raw;
  if (RC_Roll_raw < RC_Roll_raw_Min)RC_Roll_raw_Min = RC_Roll_raw;

  if (RC_Yaw_raw > RC_Yaw_raw_Max)RC_Yaw_raw_Max = RC_Yaw_raw;
  if (RC_Yaw_raw < RC_Yaw_raw_Min)RC_Yaw_raw_Min = RC_Yaw_raw;

  // Monitor Serie
  Serial.print("Thmax: ");
  Serial.print(RC_Throttle_raw_Max);
  Serial.print("\t");
  Serial.print("Thmin: ");
  Serial.print(RC_Throttle_raw_Min);
  Serial.print("Pmax: ");
  Serial.print(RC_Pitch_raw_Max);
  Serial.print("\t");
  Serial.print("Pmin: ");
  Serial.print(RC_Pitch_raw_Min);
  Serial.print("\t");
  Serial.print("Rmax: ");
  Serial.print(RC_Roll_raw_Max);
  Serial.print("\t");
  Serial.print("Rmin: ");
  Serial.print(RC_Roll_raw_Min);
  Serial.print("\t");
  Serial.print("Ymax: ");
  Serial.print(RC_Yaw_raw_Max);
  Serial.print("\t");
  Serial.print("Ymin: ");
  Serial.println(RC_Yaw_raw_Min);
}
