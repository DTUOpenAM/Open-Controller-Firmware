/*
   Control of the auxiliary devices connected to LOOP, The BUDSTER and Baxter.


  #ifndef VIBRATOR_H
  #define VIBRATOR_H

  #include "Arduino.h"

  class Vibrator {
  private:
    //pins
    byte VIB1;
    byte VIB2;
    byte VIB3;
    //accounting
    int _vib;
    int _dur;
    int _duty;
    uint16_t _freq;
    // set vibrators

  public:
    //setup
    Vibrator(byte VIB1, byte VIB2, byte VIB3);
    void init();
    void startVibrator(int _vib, int _dur, int _duty, unsigned long _freq);
    void stopVibrator();

    struct {
      volatile byte pin;
      volatile uint32_t dur = 10000; // duration to turn on vibrator
      volatile int duty = 683; // duty cycle required for viration
      volatile uint32_t freq = 100;
      volatile long vibMillis = 0; //Bookkeeping variable to set millis of when the vibrator was initiated
      volatile bool enabled = false;
    } VibratorStruct;
  };
  #endif
*/
