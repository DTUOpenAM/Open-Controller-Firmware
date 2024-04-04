/*
   GLAMS firmware: Developed to provide a unified systems controller relevant for the open architecture systems developed at DTU within DBPE AM group and funded by the Poul Due Jensen foundation
*/

////---- WHAT PLATFORM ARE YOU UPLOADING TO? ----////

//#define BAXTER
#define LOOP
//#define OPAL

//IMPORTANT: uncomment only the targeted platform

#define VERSION                (03.16)

////---- WHAT MOVE ALGO TO USE? ----////
#define BRESENHAM
//#define VECTOR // EXPERIMENTAL - better in version 4.xxx

#define EXPERTMODE
#define returnBuflen
//#define SDCARD
//#define Verbose_Serial true // Verbose output level, set at compile time
//#define machine_state_print
bool enableChecksum = false; // enable checksum, befault is off, the gcode sender will then toogle on when connected

#include "pins.h"
#include"setup.h"
//#include "lookuptables.h"
#include "misc.h"
#include "Commands.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------


void setup() {
  analogWriteResolution(10);
  pinInit();
  limitSwitchInit();
  stepperInit();

  Serial.begin(BAUD);  // open coms
  delay(2000);
  while (!Serial); // while the serial stream is not open, do nothing:
  Serial.println("start");

  setUpScanfield((float) OFFSET_CENTER_X, (float) OFFSET_CENTER_Y, x_min_step, x_max_step, y_min_step, y_max_step);

  ready();

  galvo.begin(); //Start up the galvonometer protocol

  setScannerPosition(0, 0); //Set x and y in the middle.

  help();  // say hello

  startSDcard();

  Timer1.begin(doMove, current_movePeriod); //timing how fast the bresenham algorithm is incremented
  //Timer2.begin(photosensor,500);
#ifdef SCANLABS
  galvo.setDataOutputMode(0x01ul);  // standard is to set the output mode to return Angular values
#endif
  //photosensor();
}

unsigned long looptimer;

void loop() {

  //Serial.println(micros()-looptimer);
  //looptimer = micros();

  if ((buflen == 0) && (flushingBuffer == true)) { //if flushingBuffer is set true, wait until it is empty to resume accepting commands
#ifdef SDCARD
    StoreandResetDataFileString();
#endif
    Serial.println("done flushing buffer << M10 >>");
    flushingBuffer = false;
  }

  if ((buflen <= MAX_BUFLEN) && (m[bufindexWRITE][0].CMD_READY == false) && ((bufindexREAD - bufindexWRITE + MAX_BUF-1) % MAX_BUF) > 5) { 
    // if room in buffer and the current index does not already have a command + safeguard so that it doesn't overtake itself

    //Serial.println("Reading Serial");
    readSerial();
  }

  if ((ready_sent == false) && (flushingBuffer == false) ) {
    ready();
  }


#ifdef SDCARD
  storeToDataFileString();
#endif
  //scanner-laser trajectory functionality
  LaserStartMoveOperations(); // set the power and start the laser with the required delays
  EndOfMoveOperations(); // take care of lookahead and stop the laser after the desiried position is reached

  //  if (domovedur > 1) {
  //    Serial.println("Domove Duration (us):" + String(domovedur));
  //  }

  laserOnWatchDog();
  heartBeat();       //if alive do heart beat
#ifndef OPAL
  limitRec.update(); //update the recoater limit switch ok
  UpdateSteppers();  //update stepper ok
  xflowPID();        //update flow control PID loop ok
  //sensors();         // Read the sensors ok
#endif



#ifdef machine_state_print
  if (oldbuflen == buflen) {
    if (!timerrunning) {
      buffertimer = millis();
      timerrunning = true;
    }

    if (millis() - buffertimer >= 10000) {
      timerrunning = false;
      Serial.println( "Buffer length remain unchanged for 10 s");
      Serial.println("move completed: " + String(move_completed));
      Serial.println("ve move_started: " + String(move_started));
      Serial.println( ":Buffer contains " + String(buflen) + " commands");
      Serial.println( "Reading from element " + String(bufindexREAD));
      Serial.println( "Writing to element " + String(bufindexWRITE));
      Serial.println("CMD READY W: " + String(m[bufindexWRITE][0].CMD_READY));
      Serial.println("CMD READY R: " + String(m[bufindexREAD][0].CMD_READY));
      Serial.println("Ready sent: " + String(ready_sent));
      Serial.println("Buffer: " + String(buffer));
      Serial.println("sofar: " +  String(sofar));
      Serial.println("Feedrate: " + String(fr));
      Serial.println("step_factor rounded: " + String(step_factor));
      Serial.println("Moveperiod [W]: " + String(m[bufindexWRITE][0].movePeriod));
      Serial.println("Moveperiod [R]: " + String(m[bufindexREAD][0].movePeriod));
      Serial.println("Current moveperiod: " + String(current_movePeriod));
      Serial.println("Current Position X: " + String(CURRENT_POS_X));
      Serial.println("Current Position y: " + String(CURRENT_POS_Y));
    }
  }
  oldbuflen = buflen;
#endif

  //writeDatatoSDcard();
  //photosensor(); //read and store photosensor data

}
