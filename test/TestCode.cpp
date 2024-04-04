#include "Arduino.h"

#define VIB1 10
#define VIB2 9
#define VIB3 22
#define TRG 23
#define PWR A22
#define FAN A21

//Stepper Control
#define ENABLE_1_2_3 26
#define ENABLE_4_5_6 38

#define STEP1 28
#define STEP2 24
#define STEP3 12
#define STEP4 37
#define STEP5 15
#define STEP6 17

#define DIR1 27
#define DIR2 25
#define DIR3 11
#define DIR4 36
#define DIR5 39
#define DIR6 16


void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:

  pinMode(PWR, OUTPUT);
  pinMode(FAN, OUTPUT);
  pinMode(VIB1, OUTPUT);
  pinMode(VIB2, OUTPUT);
  pinMode(VIB3, OUTPUT);
  pinMode(TRG, OUTPUT);

  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(DIR4, OUTPUT);
  pinMode(DIR5, OUTPUT);
  pinMode(DIR6, OUTPUT);

  pinMode(STEP1, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(STEP3, OUTPUT);
  pinMode(STEP4, OUTPUT);
  pinMode(STEP5, OUTPUT);
  pinMode(STEP6, OUTPUT);
  pinMode(ENABLE_1_2_3, OUTPUT);
  pinMode(ENABLE_4_5_6, OUTPUT);

  analogWriteResolution(10);
  digitalWrite(DIR1, HIGH);
}

void loop() {
  //  for (int i = 1; i < 1023; i++) {
  //    analogWrite(TRG, i);
  //    analogWrite(PWR, i);
  //    analogWrite(VIB1, i);
  //    analogWrite(VIB2, i);
  //    analogWrite(VIB3, i);
  //
  //    delay(2);
  //    //Serial.println(i);
  //  }

  analogWrite(PWR, 1023);
  //analogWrite(FAN,1);

  digitalWrite(ENABLE_1_2_3, HIGH);
  digitalWrite(ENABLE_4_5_6, LOW);
  
 // delayMicroseconds(10);

  digitalWrite(STEP1, LOW);
  delayMicroseconds(90);
  digitalWrite(STEP1, HIGH);
    //delayMicroseconds(10);

  //  digitalWrite(STEP2, HIGH);
  //  digitalWrite(STEP3, HIGH);
  //  digitalWrite(STEP4, HIGH);
  //  digitalWrite(STEP5, HIGH);
  //  digitalWrite(STEP6, HIGH);

  //  digitalWrite(DIR1, HIGH);
  //  digitalWrite(DIR2, HIGH);
  //  digitalWrite(DIR3, HIGH);
  //  digitalWrite(DIR4, HIGH);
  //  digitalWrite(DIR5, HIGH);
  //  digitalWrite(DIR6, HIGH);

  digitalWrite(ENABLE_1_2_3, LOW);
  digitalWrite(ENABLE_4_5_6, HIGH);

  //digitalWrite(STEP1, LOW);
  //  digitalWrite(STEP2, LOW);
  //  digitalWrite(STEP3, LOW);
  //  digitalWrite(STEP4, LOW);
  //  digitalWrite(STEP5, LOW);
  //  digitalWrite(STEP6, LOW);


  //  digitalWrite(DIR1, LOW);
  //  digitalWrite(DIR2, LOW);
  //  digitalWrite(DIR3, LOW);
  //  digitalWrite(DIR4, LOW);
  //  digitalWrite(DIR5, LOW);
  //  digitalWrite(DIR6, LOW);

  //delay(1);

}
