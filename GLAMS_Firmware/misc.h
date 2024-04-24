#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL)

#if defined(LOOP) && defined(BAXTER) && defined(OPAL) || defined(LOOP) && defined(BAXTER) || defined (BAXTER) &&  defined(OPAL)  || defined(OPAL) && defined(LOOP)
#error "Please ONLY select one platform, either BAXTER, LOOP or OPAL"
#endif

#if !defined(LOOP) && !defined(BAXTER) && !defined(OPAL)
#error "Please select a platform, either BAXTER, LOOP or OPAL"
#endif

#define LaserWatchDog
#define watchdogtime 500

//Timer1.priority(128);

void heartBeat() { //heartbeat function 
  static unsigned long heartbeatMillis = 0; //Bookkeeping variable
  static unsigned int beatCycle = 0; //Where we are in the beat cycle
  static unsigned int beatDelay = 100;

  if (millis() - heartbeatMillis  >= beatDelay ) { //is it alive - provide a heartbeat

    switch (beatCycle) {
      case 1:
        // statements
        digitalWriteFast(LED_BUILTIN, HIGH);

        beatDelay = 100;
        break;
      case 2:
        // statements

        digitalWriteFast(LED_BUILTIN, LOW);
        break;
      case 3:
        // statements
        digitalWriteFast(LED_BUILTIN, HIGH);
        break;
      case 4:
        // statements
        digitalWriteFast(LED_BUILTIN, LOW);

        beatCycle = 0;
        beatDelay = 600;
        break;
    }

    beatCycle++;
    heartbeatMillis = millis();
  }
}
