/*



  #include "VIBRATOR.h"

  Vibrator::Vibrator(byte VIB1, byte VIB2, byte VIB3) {
  this->VIB1 = VIB1;
  this->VIB2 = VIB2;
  this->VIB3 = VIB3;
  VibratorStruct v[4];
  init();
  }

  void Vibrator::init() {
  pinMode(VIB1, OUTPUT);
  pinMode(VIB2, OUTPUT);
  pinMode(VIB3, OUTPUT);

  v[1].pin = VIB1;
  v[2].pin = VIB2;
  v[3].pin = VIB3;

  }

  void Vibrator::startVibrator(int _vib, int _dur, int _duty, unsigned long _freq) {
  v[vib].freq = _freq;
  v[vib].dur = _dur;
  v[vib].duty = _duty;
  //  Serial.print("vib designar: ");
  //  Serial.println(_vib);
  //  Serial.print("vib pin: ");
  //  Serial.println(v[_vib].pin);
  analogWriteFrequency(v[_vib].pin, v[_vib].freq);
  analogWrite(v[_vib].pin, 0);
  analogWrite(v[_vib].pin, v[_vib].duty);

  v[_vib].vibMillis = millis();
  v[_vib].enabled = true;
  v[0].enabled = true;
  }

  #if defined(Verbose_Serial)
  Serial.print("Vibrator chosen: ");
  Serial.println(_vib);
  Serial.print("Duraton: ");
  Serial.println(_dur);
  Serial.print("DutyCycle: ");
  Serial.println(_duty);
  Serial.print("Frequency: ");
  Serial.println(_freq);
  #endif

  void stopVibrator() {
  for (int i = 1; i <= NUM_VIBRATORS; i++) {
    if (v[i].enabled == true && millis() - v[i].vibMillis >= v[i].dur) {
      analogWrite(v[i].pin, 0);
      v[i].enabled = false;

      if (v[1].enabled == false && v[2].enabled == false && v[3].enabled == false) {
        v[0].enabled = false;
      }

  #if defined(Verbose_Serial)
      Serial.print("Vibrator ");
      Serial.print(i);
      Serial.println(" stopped");
  #endif
    }
  }
  }
*/
