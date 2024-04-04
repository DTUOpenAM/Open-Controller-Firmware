/*
   Control of the auxiliary devices connected to LOOP, The BUDSTER and Baxter.
*/

#ifndef AUXDEV_H
#define AUXDEV_H

#include "Arduino.h"

class AuxDev {

    struct lookuptable {
      uint16_t _setPwrLUT;
      uint16_t _setPwrAnalogValueLUT;
    };

  private:
    //pins
    int _analogpin;
    int _triggerpin;
    bool _tooglelut;
    //accounting
    double  _setPwr;
    volatile int _intsetPwr;
    int  _maxPwr;
    uint16_t  _freq;
    int  _duty;
    unsigned int corrVal;
    uint16_t setVal;
    //int len = sizeof(correctedLaserPower) / sizeof(correctedLaserPower[0]);
    int len;
    lookuptable correctedLaserPower;
    volatile double current_power;
    volatile int current_power_signal;
    volatile int current_laser_duty;
    volatile uint32_t current_laser_frequency;
    volatile int last_laser_duty;
    bool SetSkywriting;
    //struct lookuptable {uint16_t _setPwr; uint16_t _setPwrAnalogValue;};

  public:
    volatile bool laser_is_on;
    int cnt;
    //setup
    AuxDev(int _analogpin, int _triggerpin, int _maxPwr, bool _tooglelut);
    void init();
    double setPower(double _setPwr);
    void trigLaser(uint16_t _freq, int _duty);
    bool is_the_laser_on();
    double get_current_power();
    int get_current_power_signal();
    int get_current_duty();
    int get_last_duty();
    uint32_t get_current_laser_freq();
    void set_next_duty(int _duty);
    
    void set_Skywriting(bool _skywriting);
    bool get_Skywriting();

};
#endif
