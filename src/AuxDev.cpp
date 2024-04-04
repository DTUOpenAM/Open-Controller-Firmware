#include "AuxDev.h"
#include "lookuptables.h"

AuxDev::AuxDev(int _analogpin, int _triggerpin, int _maxPwr, bool _tooglelut) {
  this->_analogpin = _analogpin;
  this-> _triggerpin = _triggerpin;
  this->_maxPwr = _maxPwr;
  this->_tooglelut = _tooglelut;
  this->laser_is_on = laser_is_on;
  init();
}

void AuxDev::init() {
  pinMode(_analogpin, OUTPUT);
  pinMode(_triggerpin, OUTPUT);
  _duty = 0;
  trigLaser(_freq, 0);
  setPower(0);
  set_Skywriting(false);
}

void AuxDev::trigLaser(uint16_t _freq, int _duty) {
  last_laser_duty = current_laser_duty;
  current_laser_duty = _duty;

  _duty = (_duty * 1023) / 100;

  if (_freq != current_laser_frequency ) {
    analogWriteFrequency(_triggerpin, _freq);
    current_laser_frequency = _freq; //accounting
  }

  laser_is_on = (_duty > 0) ? true : false;
  analogWrite(_triggerpin, _duty); //PWM
}

bool AuxDev::is_the_laser_on() {
  return laser_is_on;
}

double AuxDev::get_current_power() {
  return current_power;
}
int AuxDev::get_current_power_signal() {
  return current_power_signal;
}
int AuxDev::get_current_duty() {
  return current_laser_duty;
}
int AuxDev::get_last_duty() {
  return last_laser_duty;
}
uint32_t AuxDev::get_current_laser_freq() {
  return current_laser_frequency;
}
void AuxDev::set_next_duty(int _duty) {
  current_laser_duty  = _duty;
}


void AuxDev::set_Skywriting(bool _skywriting) {
  SetSkywriting = _skywriting;
}
bool AuxDev::get_Skywriting() {
  return SetSkywriting;
}


double AuxDev::setPower(double _setPwr) {

//  Serial.println("HER: ");
//  Serial.println((int)current_power);
//  Serial.println(_setPwr);

  if (_setPwr > _maxPwr) {
    Serial.print(_setPwr);
    Serial.print("W is exeeding the maximal allowed power ");
    Serial.println(_maxPwr + String("W"));
    Serial.println("--->> Power set to maximal power <<---");
    _setPwr = _maxPwr;
  }

  if (_setPwr < 0) {
    Serial.print(_setPwr);
    Serial.println("W is below zero, danger of creating a black hole!! ");
    Serial.println("... Restarting the flux capacitor");
    Serial.println("...");
    Serial.println("... pheeew you got lucky");
    Serial.println("--->> Power set to zero <<---");
    _setPwr = 0;
  }

  current_power = _setPwr;

  if (_tooglelut) {
    for (uint16_t k = 0; k < len; k++) {
      setVal = pgm_read_word_near( correctedLaserPower._setPwrLUT + k);
      if (setVal == _setPwr ) {
        uint16_t corrValue = pgm_read_word(correctedLaserPower._setPwrAnalogValueLUT + k);
        //        Serial.print("Desired Power: ");
        //        Serial.println(setVal);
        //        Serial.print("Corresponds to a set value of: ");
        //        Serial.println((int) round(corrValue));
        _setPwr = corrValue;
        break;
      }
    }
  } else {
    _setPwr = (_setPwr * 1023) / _maxPwr;
  }
  
  
  _intsetPwr =  (int) round(_setPwr); //accounting
  current_power_signal = _setPwr; // to access globally
  analogWrite(_analogpin, (int) _intsetPwr); //Highest value of the DAC is at 1023. _analogpin
  return current_power;
}
