#include "GalvoControl.h"
#include "Motors.h"

//#include "Process.h"

#include "AuxDev.h"

//------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------

//----- BUFFER VARIABLES
char buffer[MAX_CHAR_BUF];  // where we store the message until we get a '; '
int sofar = 0;  // how much is in the buffer

//----- MOTOR AND PROCESS VARIABLES
int pulses, freq, dur, Revolutions, duty;
int NoDose; int Hopper; int DoseType; int PowderFeedrate; int PowderAcceleration; float posmm; float posum; float Zspeed_mm; float Zacceleration_mm; int motor = 0; float moveZ;
int manual_feedrate; float recDistance; float recAcceleration; int inState; int outState; double stp; int xflow; int allState; int Materials; int HopperSelect;

int Steps;

bool moving_scanner = false;

//------------------------------------------------------------------------------
// VOIDSint
//------------------------------------------------------------------------------

void help() {// Display helpfull information should be updated to match functionality
  Serial.println("Firmware: A unified machine control system for XY2-100 Galvo control, sensors and motors");
  Serial.print("Platform: ");
  Serial.println(PLATFORM);
  Serial.println("Sebastian Aagaard Andersen, Magnus Bolt Kjer, Christian Budden, David Bue Pedersen");
  Serial.println("DTU 2016-2022");
  Serial.print("Version: ");
  Serial.println(VERSION);
  Serial.println("Commands:");
  Serial.println(F("G1  [X/Y(position, mm)] [F(feedrate)] [D(duty cycle)] [P(power)] [F(pulse frequency)]; - linear move mm"));
  Serial.println("G6  [T(duration in microseconds)] [N(number of pulses] - mirror move with stationary laser");
  Serial.println("G7  [D(duty cycle)] [P(power)] [F(pulse frequency)] T[duration in microseconds]- Laser emission with no movement <- use this for power reading");
  Serial.println(String("G8  [P(Set Power Level 0 - ") + MAX_PWR + String("W)]"));
  Serial.println("  Motor commands:");
  Serial.println("M1 - Powder dispensing - [P[Hopper) 1,2,3], [N(Number of doses)],");
  Serial.println("M2 - Z-axis - [T(layer height in mm)]");
  Serial.println("M3 - Layer Cycle - [Recoat powder layer]"); //perform a full recoating cycle
  Serial.println("M4 - Recoat Powder - [Pull powder across build plate and pause at the front]");
  Serial.println("M5 - Retract Recoater - [Move recoater to home position after scanning]");
#if defined (EXPERTMODE) && defined(LOOP)
  Serial.println(" ");
  Serial.println("  Special Motor Commands:");
  Serial.println("M0 - Multi-material control - P[1-3] - P1 select back dispenser, P2 select front dispenser, P3 select both dispensers");
  Serial.println("M6 - Recoating Routine - No Build Plate Movement");
  Serial.println("M7 - Home Recoater - Should Be Called At The Start Of Every Print");
  Serial.println("M8 - Manual Z-axis control:"); //control individual motors in z axis only relevant for loop same more advanced functianlty as M"
  Serial.println("     [F(feedrate in mm/s)], [A(acceleration in mm/s^2)]");
  Serial.println("     Calibration: [Z(Motor select, 1-3)], [T(Move motor in mm)]");
  Serial.println("M11 - Manual Recoater:");
  Serial.println("     [F(Speed in mm/s)] [D(Move to position, mm)] [A(Acceleration mm/s^2)]");
 
  Serial.println("M12 - Reset z-axis - Should only be done if the z-axis was not a zero when starting up machine.");
#endif

  Serial.println("M10 Pause serial input until buffer is empty");
  Serial.println("M20 Disable stepper motor");
  Serial.println("M21 Enable stepper motor");
  Serial.println("M22  Return endstop status");
  Serial.println("M30 Return sensor status");
  Serial.println("M100 Toogle Checksum");
  Serial.println("M114 Get current scanner position");
  Serial.println("M115 Print state machine");


#ifdef LOOP

  Serial.println(" Process commands:");
  Serial.println("P1 - Toggle chamber gas - [T(Toggle 1 = open, 2 = closed)]");
  Serial.println("P2 - Toggle cross-flow gas - [T(Toggle 1 = open, 2 = closed)]");
  Serial.println("P3 - Toggle all gas valves - [T(Toggle 1 = open, 2 = closed)]");
  Serial.println("P4 - Cross flow speed - [F(Flow in m/s)]");

#endif

  Serial.println("M999 See Commands");
  Serial.println("S0  Stops program");
  Serial.println("S1  Pause/unpause");
#ifdef BAXTER
  Serial.println("C0  Home Motors, Clearpath");
  Serial.println("C1  [A/R (Set Zstage Position µm (Absolute/Relative))], Clearpath");
  Serial.println("C2  [A/R (Set Wiper Position µm (Absolute/Relative))], Clearpath");
  Serial.println("C3  Recoate 1 layer, Clearpath");
#endif
#ifdef SCANLABS
  Serial.println("O1  Set Scanner Return Data to Angular");
  Serial.println("O2  Set Scanner Output Data to Set Angular Postition");
  Serial.println("O3  Set Scanner Output Position Error");
  Serial.println("O6  Set Scanner Output Actual Angular Velocity (bit/ms)");
#endif
  Serial.println("All commands must end with a newline");
}


void laserOnWatchDog() {
#ifdef LaserWatchDog
//Serial.println(String("watchdog timer: ")+String(laser_on_timer));
  if (millis() - laser_on_timer >= watchdogtime && aux.is_the_laser_on() && startLaserWatchDog) {
    aux.trigLaser(m[bufindexREAD][0].freq, 0);
    Serial.println(String(millis() - laser_on_timer) + " ms");
    Serial.println("It looks like the laser was left on - I turned it off");
    Serial.println(aux.is_the_laser_on());
  }
#endif
}

float parsenumber(char code, float val) {

  /**
    Look for character /code/ in the buffer and read the float that immediately follows it.
    @return the value found.  If nothing is found, /val/ is returned.
    @input code the character to look for.
    @input val the return value if /code/ is not found.
    // **/
  char *ptr = buffer;
  while (ptr && *ptr && ptr < buffer + sofar) {
    if (*ptr == code) { // if /code/ is found
      return atof(ptr + 1); // return float
    }
    ptr = strchr(ptr, ' ') + 1; //if " " is found increment pointer to the next char after the space

    if (ptr < buffer) break;
  }
  return val;
}

void ready() {
  String str;
  sofar = 0; // clear input buffer
  str = "ok";
#ifdef returnBuflen
  str = str + (String(" B") + String(buflen) + "; R" + String(bufindexREAD) + "; W" + String(bufindexWRITE));
#endif
  Serial.println(String(str));
  str = "";
  ready_sent = true;
}

void processCommand() {
  /**
    Read the input buffer and find any recognized commands.  One G or M command per line.
  */
  int nope = 0; // counter to break the loop if an unvalid command is recived

  moving_scanner = false; // default we assume the move is not a scanner move

  //// CASE SWITCH G-COMMANDS ///////

  int cmd = parsenumber('G', -1);
  switch (cmd) {
    case -1: nope++; break;
    case 1: {// Set mirror position in mm X and Y
        moving_scanner = true;
        //feedrate(parsenumber('F', fr));
        planMove(parsenumber('X', px),
                 parsenumber('Y', py),
                 parsenumber('H', ph),
                 parsenumber('D', pd),
                 parsenumber('P', pwr),
                 parsenumber('R', rmp),
                 parsenumber('F', fr));
        break;

      }
    case 5: {
        break;
      }
    case 6: {
        //Go somewhere and do a certain number of pulses there.
        pulses = parsenumber('N', 0);
        freq = parsenumber('H', oldFreq);
        dur = 1000000 * pulses / freq; //Dur is in microseconds, so convert by multiplying with 1 M.
        //feedrate(parsenumber('F', fr));
        planMove(parsenumber('X', (mode_abs ? px : 0)) + (mode_abs ? 0 : px),
                 parsenumber('Y', (mode_abs ? py : 0)) + (mode_abs ? 0 : py),
                 freq, //OBS!
                 parsenumber('D', pd),
                 parsenumber('P', pwr),
                 parsenumber('R', rmp),
                 parsenumber('F', fr));
        break;
      }
    case 7: {// run laser for T duration
        int dur;
        aux.setPower(parsenumber('P', pwr));
        aux.trigLaser(parsenumber('F', freq), parsenumber('D', duty));
        int lasestationary = millis();
        while (millis() - lasestationary >= parsenumber('T', dur)) {
        }
        delay(parsenumber('T', dur));
        aux.trigLaser(parsenumber('F', freq), 0);
        break;
      }

    case 8: {// set laser power
        aux.setPower(parsenumber('P', pwr));
        break;
      }

    case 10: {// get laser status
        Serial.println("Current Duty: " + String(aux.get_current_duty()));
        Serial.println("Current Power: " + String(aux.get_current_power()));
        break;
      }

    default: {
        Serial.println("Hmm - we don't seem to have that functionality yet :(");
        Serial.println("-- Input will be ignored --");
        break;
      }
  }

  //// CASE SWITCH O-COMMANDS ///////

  cmd = parsenumber ('O', -1);
  uint32_t codeL;

  switch (cmd) {
    case -1: nope++; break;
    case 0:  {
        codeL = 0x00ul;
        Serial.print("Statusword");
        galvo.setDataOutputMode(codeL);
        break;
      }
    case 1: {
        codeL = 0x01ul;  //Actual angular position (-323768 ... 32767)
        Serial.println("Actual angular position (-323768 ... 32767)");
        galvo.setDataOutputMode(codeL);
        break;
      }
    case 2: {
        codeL = 0x02ul;  //Set angular position (-323768 ... 32767)
        galvo.setDataOutputMode(codeL);
        break;
      }
    case 3: {
        codeL = 0x03ul;  //position error (set pos - actual) (-323768 ... 32767)
        galvo.setDataOutputMode(codeL);

        break;
      }
    case 4: {
        codeL = 0x04ul;  //Actual current (output stage current) (-323768 ... 32767)
        galvo.setDataOutputMode(codeL);
        break;
      }
    case 5: {
        codeL = 0x05ul;  ////Relative galvo control  (-1000 ... 1000)
        galvo.setDataOutputMode(codeL);
        break;
      }
    case 6: {
        codeL = 0x06ul;  //Relative galvo control (-323768 ... 32767)
        galvo.setDataOutputMode(codeL);
        break;
      }
    case 14: {
        codeL = 0x14ul;  //Set angular position (-323768 ... 32767)
        galvo.setDataOutputMode(codeL);
        break;
      }//
    case 15: {
        codeL = 0x15ul;  //Set angular position (-323768 ... 32767)
        galvo.setDataOutputMode(codeL);
        break;
      }
    case 16: {
        codeL = 0x16ul;  //Set angular position (-323768 ... 32767)
        galvo.setDataOutputMode(codeL);
        break;
      }
    case 17: {
        codeL = 0x17ul;  //Set angular position (-323768 ... 32767)
        galvo.setDataOutputMode(codeL);
        break;
      }
    case 18: {
        codeL = 0x18ul;  //Set angular position (-323768 ... 32767)
        galvo.setDataOutputMode(codeL);
        break;
      }
    case 19: {
        codeL = 0x19ul;  //Serial.println(Out)Set angular position (-323768 ... 32767)
        galvo.setDataOutputMode(codeL);
        break;
      }
    case 100: { // Print output
        Serial.print("Channel A says: ");
        Serial.println(digitalRead(OUTCHAN1));
        Serial.print("Channel B says: ");
        Serial.println(digitalRead(OUTCHAN2));
        break;
      }

    case 600: { // return a string of sensor data
        sendSensorData();
        break;
      }

    default: {
        Serial.println("This is not the output you are looking for ...");
        Serial.println("-- Input will be ignored --");
        break;
      }
  }

  //// CASE SWITCH M-COMMANDS ///////

  cmd = parsenumber('M', -1);
  switch (cmd) {
    case -1: nope++; break;

    // --- Multi-Material ---
    case 0: {
      MultiMaterial(parsenumber('P',HopperSelect));
      break;
    }

    // --- DOSING ---
    case 1: {   // Powder dosing cycle
        Powder(parsenumber('P', Hopper), parsenumber('N', NoDose), parsenumber('F', PowderFeedrate), parsenumber('A', PowderAcceleration), parsenumber('Q', Steps));
        break;
      }


    // --- Z AXIS ---
    case 2: {  // Move the z-axis in microns
        Zaxis(parsenumber('T', posum));
        break;
      }

    // --- Layer cycle ---

    case 3: {   // Make a complete powder recoate cycle
        LayerCycle();
        break;
      }

    case 4: {   // Only recoat to the front and wait
      #ifdef LOOP
        RecoatPowder();
      #endif
        break;
      }
    case 5: {
      #ifdef LOOP
        RecoatRetract();
      #endif
        break;
      }

    // --- RECOATER ---
    case 6: {   // Do just a recoating routine
        Recoater();
        break;
      }

    // --- Z-AXIS ---
    case 8: {    // Full control of the z-axis, incl. individual motor control in microns
#ifdef LOOP
        ManualZaxis(parsenumber('D', posmm), parsenumber('F', Zspeed_mm), parsenumber('A', Zacceleration_mm), parsenumber('Z', motor), parsenumber('T', posum));
#endif
        break;
      }

    // --- MANUAL RECOATER MOVEMENT ---
    case 11: {    // Full control of the recoater
#ifdef LOOP
        ManualRecoater(parsenumber('F', manual_feedrate), parsenumber('D', recDistance), parsenumber('A', recAcceleration));
#endif
        break;
      }
    case 7: {    // Home the recoater using special movement settings.
        HomingRecoater();
        break;
      }

    case 12: {   // Zero z-axis
#ifdef LOOP
        ResetZaxis();
#endif
        break;
      }


    case 9: {// M9 get some system data
        Serial.print("Current Power: ");
        Serial.println(aux.get_current_power());
        Serial.print("Current Power Signal: ");
        Serial.println(aux.get_current_power_signal());
        Serial.print("Current Duty: ");
        Serial.println(aux.get_current_duty());
        Serial.print("Last Duty: ");
        Serial.println(aux.get_last_duty());
        break;
      }

    case 10: {// clear buffer
#if defined(Verbose_Serial)
        Serial.println("Clearing Buffer");
        Serial.print("Buffer Lenght: ");
        Serial.println(buflen);
#endif
        flushingBuffer = true;
        break;
      }

    case 20: {//disable stepper both groups
        digitalWrite(ENABLE_1_2_3, HIGH);
        digitalWrite(ENABLE_4_5_6, HIGH);

        break;
      }
    case 21: {//enabler stepper both groups
        digitalWrite(ENABLE_1_2_3, LOW);
        digitalWrite(ENABLE_4_5_6, LOW);


        break;
      }
    case 22: {//ENDSTOP readout
        Serial.print("Endstop 1: ");
        Serial.println(digitalRead(ESTP1));
        Serial.print("Endstop 2: ");
        Serial.println(digitalRead(ESTP2));
        Serial.print("Endstop 3: ");
        Serial.println(digitalRead(ESTP3));
        Serial.print("Endstop 4: ");
        Serial.println(digitalRead(ESTP4));
        break;
      }
    case 30: { //readback sensor values
        Serial.print("Sensor 1: ");
        Serial.println(analogRead(SENS1));
        Serial.print("Sensor 2: ");
        Serial.println(analogRead(SENS2));
        Serial.print("Sensor 3: ");
        Serial.println(analogRead(SENS3));
      }

    case 666: {//restart teensy
        CPU_RESTART;
        break;
      }

    case 100: {//toogle checksumming
        //enableChecksum = !enableChecksum;
        //Serial.println(enableChecksum);
        break;
      }
    case 114: {//get current scanner position
        Serial.println("CURRENT_POS_X: " + String(CURRENT_POS_X) + ", CURRENT_POS_Y: " + String(CURRENT_POS_Y));
        Serial.println("px: " + String(px,4) + ", py: " + String(py,4));
        break;
      }

    case 115: {//print state machine
        Serial.println( ":Buffer contains " + String(buflen) + " commands");
        Serial.println( "Reading from element " + String(bufindexREAD));
        Serial.println( "Writing to element " + String(bufindexWRITE));
        Serial.println("CMD READY W: " + String(m[bufindexWRITE][0].CMD_READY));
        Serial.println("CMD READY R: " + String(m[bufindexREAD][0].CMD_READY));
        Serial.println("Max Steps: " + String(m[bufindexREAD][0].maxsteps));
        Serial.println("StepCount: " + String (stepcount));
        Serial.println("Move Completed: " + String(move_completed));
        Serial.println("Bresenham Bool: " + String(m[bufindexREAD][0].maxsteps > 0 && !move_completed && galvo_allowed_to_settle));
        Serial.println("Ready sent: " + String(ready_sent));
        Serial.println("Buffer: " + String(buffer));
        Serial.println("sofar: " +  String(sofar));
        Serial.println("Feedrate: " + String(fr));
        Serial.println("step_factor rounded: " + String(step_factor));
        Serial.println("Galvo allowed to settle: " + String(galvo_allowed_to_settle));
#ifdef BRESENHAM
        Serial.println("Moveperiod [W]: " + String(m[bufindexWRITE][0].movePeriod));
        Serial.println("Moveperiod [R]: " + String(m[bufindexREAD][0].movePeriod));
        Serial.println("Current moveperiod: " + String(current_movePeriod));
#endif
        Serial.println("Current Position X: " + String(CURRENT_POS_X));
        Serial.println("Current Position y: " + String(CURRENT_POS_Y));
        break;
      }

    case 999: {//repeat the help screen
        help();
        break;
      }

    default: {
        Serial.println("Better luck next time - maybe try M999?");
        Serial.println("-- Input will be ignored --");
        break;
      }
  }


  // ------ P-COMMANDS [PROCESS] ------

  cmd = parsenumber('P', -1); // P-commands below

  switch (cmd) {
    case -1: nope++; break;

    case 1: {     // Open or close solenoid valve 1
#ifdef LOOP
        GasIn(parsenumber('T', inState));
#endif
        break;
      }

    case 2: {     // Open or close solenoid valve 2
#ifdef LOOP
        GasOut(parsenumber('T', outState));
#endif
        break;
      }

    case 3: {     // Open or close both solenoid valves
#ifdef LOOP
        AllGas(parsenumber('T', allState));
#endif
        break;
      }


    case 4: { //Set flow SP
#if defined(LOOP) || defined(BAXTER)

        setPointFlow(parsenumber('F', stp));
        Serial.println(stp);
#endif
        break;
      }


    case 5: { // set voltage level of 0-10V output
#if defined(LOOP) || defined(BAXTER)
        flow.setPower(parsenumber('F', xflow));
#endif
        break;
      }

  }

  //// CASE SWITCH S-COMMANDS ///////

  cmd = parsenumber('S', -1);
  switch (cmd) {

    case -1: nope++; break;

    case 0: { //Stop the program and clear the buffer
        //Stop the program
        aux.trigLaser(m[bufindexREAD][0].freq, 0); //zero the duty cycle
        aux.setPower(0);

        //reset the buffer
        bufindexWRITE  = 1;
        bufindexREAD   = 1;
        buflen         = 0;
        Serial.print(" Build stopped - buffer cleared");
        break;
      }


    case 1: {//Instantanious pause
        if (pause == false) {
          //unsigned long duration = parsenumber('T', -1);
          aux.trigLaser(m[bufindexREAD][0].freq, 0);
          galvo.pause();
          Timer1.end();
          Serial.println("Pausing");
          //Pause the duty cycle.
          //This does not stop the power options, however.
          pause = !pause;

        } else {
          Serial.println("Unpausing");
          galvo.unpause();
          //ressume timer intterput doMove
          Timer1.begin(doMove, current_movePeriod);
          //aux.trigLaser(m[bufindexREAD][0].freq, m[bufindexREAD][0].LSR_DUTY); //start the duty cycle.
          pause = !pause;
        }
        break;
      }
    case 2: {

        break;
      }
    default: {
        Serial.println("Nope, no such S command available - maybe try M999?");
        Serial.println("-- Input will be ignored --");
        break;
      }
  }
  if (nope == 5) {
    Serial.println("This, looks like nothing to me ?!?");
    Serial.println("-- Input will be ignored --");
  }
}

bool verifyChecksum (String codeString) { // SEAA
  int commandLength = 0;
  int checksum = 0;
  int recievedCheckSum = 0;

  recievedCheckSum = codeString.substring(codeString.indexOf('*') + 1, codeString.length()).toInt();
  codeString = codeString.substring(0, codeString.indexOf('*'));
  codeString = codeString.trim();

  if (codeString == "M100") {
    enableChecksum = !enableChecksum;
    Serial.println(enableChecksum);
    if (enableChecksum) {
      Serial.println("CheckSum On");
    } else {
      Serial.println("CheckSum Off");
    }
  }

  commandLength = codeString.length();

  while (commandLength) checksum ^= codeString[--commandLength];

  if (enableChecksum) {
    return ((checksum == recievedCheckSum) ? true : false);
  } else {
    return ((checksum == recievedCheckSum) ? true : true);
  }
}

void readSerial() {
  while (Serial.available() > 0) { // if something is available
    char c = Serial.read(); // get it
    //Serial.print(c);  // repeat it back so I know you got the message
    if (sofar < MAX_CHAR_BUF - 1) buffer[sofar++] = c; // store it
    if (c == '\n') {

      //Serial.print(buffer);

      // entire message received
      buffer[sofar] = 0; // end the buffer so string functions work right

      if (verifyChecksum(buffer)) {
        processCommand();  // do something with the command
        ready_sent = false;
      }
      else {
        Serial.println("resend");
      }

      sofar = 0; // clear input buffer
    }
  }
}
