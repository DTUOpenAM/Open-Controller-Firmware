#ifndef PINS_H
#define PINS_H
//

#if defined (__MK66FX1M0__)

#define CLOCK1 2
#define SYNC1 14
#define CHAN11 7
#define CHAN21 8
#define CLOCK0 6
#define SYNC0 20
#define CHAN10 21
#define CHAN20 5

//XY2-100 OUTPUT
#define OUTCHAN1 0
#define OUTCHAN2 1

//Laser Control
#ifdef OPAL
#define TRIG A2
//#define FAN A22 //this is not the way RND DSGNTN SEAA TODO 
#define POWER A21 // 21
#endif

#if defined(LOOP) || defined(BAXTER)
#define TRIG 23
#define POWER A22
#define FAN A21
#endif

//Recirc

//SENSOR
#define SENS1 A11
#define SENS2 A10
#define SENS3 A14

//ENDSTOP
#define ESTP1 3
#define ESTP2 4
#define ESTP3 34
#define ESTP4 35

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

//Optional outputs for auxillary devices

#ifdef BAXTER

#define VIB1 10
#define VIB2 9
#define VIB3 22

//Beam expander
#define BEX0 29
#define BEX1 30
#define BEX2 31
#define BEX3 32

#endif
#ifdef LOOP

#define GASIN 10
#define GASOUT 9
#define VIB3 22

#elif defined(OPAL)

#define VIB1 10
#define VIB2 9
#define VIB3 22

#endif

#else

Serial.println("Pins not defined for the current board, update pins.h");

#endif

void pinInit() {

  pinMode(SENS1, INPUT_PULLDOWN);
  pinMode(SENS2, INPUT_PULLDOWN);
  pinMode(SENS3, INPUT_PULLDOWN);

  pinMode(CLOCK1, OUTPUT);   // bit 0 - pin 2
  pinMode(SYNC1, OUTPUT); // bit 1 - pin 14
  pinMode(CHAN11, OUTPUT);  // bit 2 - pin 7
  pinMode(CHAN21, OUTPUT);  // bit 3 - pin 8
  pinMode(CLOCK0, OUTPUT);  // bit 4 - pin 6
  pinMode(SYNC0, OUTPUT); // bit 5 - pin 20
  pinMode(CHAN10, OUTPUT); // bit 6 - pin 21
  pinMode(CHAN20, OUTPUT);  // bit 7 - pin 5

  pinMode(OUTCHAN1, INPUT_PULLDOWN);
  pinMode(OUTCHAN2, INPUT_PULLDOWN);

  pinMode(ENABLE_1_2_3, OUTPUT); //Enable stepper output 1 2 and 3
  pinMode(ENABLE_4_5_6, OUTPUT); //Enable stepper output 1 2 and 3

  pinMode(DIR1, OUTPUT); //Toggle direction
  pinMode(STEP1, OUTPUT); //STEP STEPSTICK1
  pinMode(DIR2, OUTPUT); //Toggle direction
  pinMode(STEP2, OUTPUT); //STEP STEPSTICK2
  pinMode(STEP3, OUTPUT); //STEP STEPSTICK3
  pinMode(DIR3, OUTPUT); //Toggle direction

#ifdef BAXTER

  pinMode(VIB1, OUTPUT);
  pinMode(VIB2, OUTPUT);
  pinMode(VIB3, OUTPUT);

#elif defined(LOOP)

  pinMode(GASIN, OUTPUT);
  pinMode(GASOUT, OUTPUT);
  pinMode(VIB3, OUTPUT);

#elif defined(OPAL)

  pinMode(VIB1, OUTPUT);
  pinMode(VIB2, OUTPUT);
  pinMode(VIB3, OUTPUT);

#endif

  pinMode(ESTP1, INPUT_PULLUP);
  pinMode(ESTP2, INPUT_PULLUP);
  pinMode(ESTP3, INPUT_PULLUP);
  pinMode(ESTP4, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);
}

#endif
