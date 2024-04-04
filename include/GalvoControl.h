#include "AuxDev.h"
#include "XY2_100.h"
#include "Process.h"
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

//------------------------------------------------------------------------------
//VARIABLES
//------------------------------------------------------------------------------

int pwr, rmp;
volatile float px, py; // position
volatile float pd, ph;
char mode_abs = 1; // absolute mode?

double PhDSens;
double  PhDInput;
double  PhDOutput;
int tt = millis();

double pwr_increments;
double duty_increments;


//------------------------------------------------------------------------------
//Skywriting
//------------------------------------------------------------------------------
//#define SKYBUG
bool Skywriting;
bool previously_skywriting; //Were we using skywriting for the previous command
bool contour = false;
bool currently_launching = false;
bool sky_time_end = false;
volatile float skywriting_length, l;
volatile float xA, yA, xB, yB, xC, yC, oldx, oldy, currentx, currenty; // position in mm
int skytime = 1000;
int nb_commands = 0;
int extra_command = 0;
int nb_launch = 0;
int nb_doMove = 0;
bool previous_bundle = false;

typedef struct {
  volatile float NEWX = 0;
  volatile float NEWY = 0;
  volatile uint32_t FREQ = 0; //frequency of trigger. Set to 50 kHz for now, add functionality later.
  volatile uint8_t DUTY = 0; //dutycycle of trigger
  volatile double PWR = 0; //Laser power
  volatile int RMP = 0; //what ramping strategy is used 0 means none, 1 ramp power, 2 ramp duty cycle
  volatile float FR = 0;
}Command;

long unsigned int time_end = 0;
long unsigned int time_start = 0;

//------------------------------------------------------------------------------
//MOTION BUFFER
//------------------------------------------------------------------------------

#define  MAX_BUFLEN       (128) // How many commands can be saved in the motion buffer
volatile int bufindexWRITE  = 1; // initialization of entry are we writing into the buffer 1 - MAX_BUFLEN
volatile int bufindexREAD   = 1; // which entry are we reading from the buffer 1 - MAX_BUFLEN
int prev_index = bufindexREAD;
volatile int buflen         = 0; // current bufferlength = Number of entries in buffer
volatile bool updating_buflen;
volatile int oldbuflen         = 0;
unsigned long buffertimer;
bool timerrunning;

bool buff_full               = false;
bool ready_sent              = false;
bool flushingBuffer            = true;
bool pause = false;
volatile bool startLaserWatchDog = false;

volatile bool didstuff = false;
//------------------------------------------------------------------------------
//Move Routine
//------------------------------------------------------------------------------

volatile int stepcount  = 0; // How many steps has been carried out in the move routine?
volatile int RES        = 20; //Control the resolution /1 full res is 1
//volatile int movePeriod = 50; //how often is the galvoes incremented µs
volatile double current_movePeriod = 50;
volatile double update_power;
volatile double update_duty;

int look_ahead_variable;

volatile int laser_on_timer;
bool move_completed = false;
bool move_completed_copy = false;
bool move_started_copy = false;
bool laser_handled = false;
bool move_started = false;
bool pause_movement = false;
bool galvo_allowed_to_settle = false;
bool inital_move_routine_completed = false;
unsigned long whiletimer;
unsigned long laser_ramp_up_timer;
unsigned long tracking_error_timer = 0;
unsigned long step_response_timer = 0 ;
unsigned long timer_storage = 0;
unsigned long timer_storage_step_response = 0;
unsigned long domovedur;
unsigned long scanner_settling_timer;
unsigned long settling_timer_storage = 0;

uint16_t x_min_step;
uint16_t x_max_step;
uint16_t y_min_step;
uint16_t y_max_step;

int one_step_time_limit = 50; //50
double step_factor = 1;

volatile float fr = 200; // mm/s //GLOBAL
uint32_t oldFreq = 50000;
uint16_t oldPwr = 0;

volatile int delayedStartTimer;

volatile uint64_t CMD_START_MICROS;
volatile uint64_t CMD_END_MICROS;
volatile uint64_t NOW;

volatile double CURRENT_CMD_X = 0;
volatile double CURRENT_CMD_Y = 0;

double _checkedposx;
double _checkedposy;

#ifdef VECTOR
double CURRENT_POS_X = 0;
double CURRENT_POS_Y = 0;
#endif
#ifdef BRESENHAM
volatile int CURRENT_POS_X = 0; //in step positions
volatile int CURRENT_POS_Y = 0;
volatile int PREV_POS_X = 0;
volatile int PREV_POS_Y = 0;

#endif


//------------------------------------------------------------------------------
//Struct
//------------------------------------------------------------------------------

typedef struct {
  volatile double delta = 0; // number of steps to move
  volatile double oldPos = 0; // to remember where we are
  volatile double newPos = 0; // new position
  volatile float dist = 0;
  volatile double moveDurationMicros;
#ifdef BRESENHAM
  volatile int over = 0; // for dx/dy bresenham calculations
  volatile int curPos = 0; // to remember where we
  volatile int dir = 0; //direction to increment dac
  volatile double movePeriod; // sets how often the doMove should be called controls scanspeed was 1000 SEAA
  volatile int RES = 1; //resolution
  volatile int maxsteps = 0; //maxumum increments needed
#endif
  volatile uint8_t duty = 0; //dutycycle of trigger
  volatile uint32_t freq = 50000; //frequency of trigger. Set to 50 kHz for now, add functionality later.
  volatile double pwr = 0; //Laser power
  volatile bool CMD_READY = false;
  volatile bool FIRST = false;
  volatile int rmp = 0; //what ramping strategy is used 0 means none, 1 ramp power, 2 ramp duty cycle
  volatile float fr = 200;
  bool stationary;
  volatile bool CMD_BUNDLE; //alternating value btw launchmove() bundles used to know where to put delays
  volatile bool CONTOUR;
  volatile bool SKYWRITING;
} Mirror;


//------------------------------------------------------------------------------
//INITIALIZATION OF STRUCTS
//------------------------------------------------------------------------------

Mirror m[MAX_BUFLEN + 1][NUM_MIRRORS]; //define mirror move buffer size
IntervalTimer Timer1;

AuxDev aux(POWER, TRIG, MAX_PWR, LASER_LUT); //set power pin (0-10v), trigger pin 0-24v ttl, maximal allowed power in W
XY2_100 galvo;

Command C1;
Command C2;
Command C3;


//------------------------------------------------------------------------------
//VOIDS
//------------------------------------------------------------------------------

//void feedrate(float nfr) {
//  if (fr == nfr) return; // same as last time?  quit now.
//  fr = nfr;
//}

void setUpScanfield(float _offset_center_x, float _offset_center_y, uint16_t & _x_min_step, uint16_t & _x_max_step, uint16_t & _y_min_step, uint16_t & _y_max_step) {
  _x_min_step  = (uint16_t) FULLSCALEGALVO / 2 + round((X_MIN + _offset_center_x) * STEPS_PER_MM_X);
  _x_max_step  = (uint16_t) FULLSCALEGALVO / 2 + round((X_MAX + _offset_center_x) * STEPS_PER_MM_X);
  _y_min_step  = (uint16_t) FULLSCALEGALVO / 2 + round((Y_MIN + _offset_center_y) * STEPS_PER_MM_Y);
  _y_max_step  = (uint16_t) FULLSCALEGALVO / 2 + round((Y_MAX + _offset_center_y) * STEPS_PER_MM_Y);
  return;
}

void checkScannerPosition(double _x, double _y, double & _Return_X, double & _Return_Y) {

  //bool IsItInsideLimits = true;

  if ( _x < X_MIN || _x > X_MAX || _y < Y_MIN || _y > Y_MAX) {
    _x = (_x < X_MIN) ? X_MIN : _x;
    _x = (_x > X_MAX) ? X_MAX : _x;
    _y = (_y < Y_MIN) ? Y_MIN : _y;
    _y = (_y < Y_MIN) ? Y_MAX : _y;

    Serial.println("<< Scanner position not reachable >>");
    Serial.println("<< Position set to: (" + String(_x) + ", " + String(_y) + ") >>");

  }

  _Return_X = _x;
  _Return_Y = _y;
}

void storeScannerPosition_planMove(float npx, float npy) {
  px = npx;
  py = npy;
}

void setScannerPosition(double _x, double _y) { // takes cartesian coordinates
  int tmp_x, tmp_y;

  if (INVERT_X_AXIS)
    tmp_x = map(_x, X_MIN, X_MAX, x_max_step, x_min_step);
  else
    tmp_x = map(_x, X_MIN, X_MAX, x_min_step, x_max_step);

  if (INVERT_Y_AXIS)
    tmp_y = map(_y, Y_MIN, Y_MAX, y_max_step, y_min_step);
  else
    tmp_y = map(_y, Y_MIN, Y_MAX, y_min_step, y_max_step );

  //    Serial.println("[steps] Xpos: " + String(tmp_x) + ", Ypos: " + String(tmp_y));
  //    Serial.println("[actual] Xpos: " + String(_x) + ", Ypos: " + String(_y));
  galvo.setXY(tmp_x, tmp_y);

  CURRENT_POS_X = tmp_x;
  CURRENT_POS_Y = tmp_y;

  //storeScannerPosition_planMove(_x, _y);

  //  Serial.println("X: " + String(tmp_x) + ", Y: " + String(tmp_y));
  //  Serial.println("X: " + String(_x) + ", Y: " + String(_y));

}

void savePosToSDcard() {
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.println(String(CURRENT_POS_X) + " ; " + String(CURRENT_POS_Y) + " ; " + String(aux.is_the_laser_on()));

    dataFile.close();
    // print to the serial port too:
    // Serial.println("storing data");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

int datafile_counter = 0;
String dataFilestring = "";

void storeToDataFileString() {
  if (CURRENT_POS_X != PREV_POS_X || CURRENT_POS_Y != PREV_POS_Y ) {
    PREV_POS_X = CURRENT_POS_X;
    PREV_POS_Y = CURRENT_POS_Y;
    dataFilestring += String(CURRENT_POS_X) + ";" + String(CURRENT_POS_Y) + String(";") + String(aux.is_the_laser_on()) + ";";
  }
  //datafile_counter++;
}

void StoreandResetDataFileString() {

  File dataFile = SD.open("scanPos.txt", FILE_WRITE);
  Serial.println("datafile: " + String(dataFile));
  if (dataFile) {
    Serial.println("storing data");

    //for (int i=1; i == datafile_counter,i++;) {
    Serial.println(dataFilestring);
    dataFile.println(dataFilestring);
    //}

    dataFile.close();

    dataFilestring = "";
    //datafile_counter = 0;

    // print to the serial port too:
    Serial.println("data Stored");
  } else {
    Serial.println("Something went wrong with SD card");
  }
}




/** This function is used to calculate the coordinates of a point C based
 * on a calculated lenght and point A and B such that 
 * A-----B---C
 * **/
void skywritingplanner(float xA, float yA,float xB, float yB, float _fr){
  
  // length we are adding from B to C in mm
  skywriting_length = _fr * skytime/1000000; 
  
  l = sqrt(pow(xA-xB,2) + pow(yA-yB,2)) + skywriting_length;
  xC = ((l*xB - skywriting_length*xA)/(l-skywriting_length));
  yC = ((l*yB - skywriting_length*yA)/(l-skywriting_length));
  
  checkScannerPosition(xC, yC, _checkedposx, _checkedposy); //check if we're in the domain
  xC = _checkedposx;
  yC = _checkedposy;

}




/**
 * This function is called by planmove everytime a moves have been processed 
 * and ready to put in the buffer for execution
 * **/
void prepMove(float _newx, float _newy, uint32_t freq, int _duty, int _pwr, int _rmp, float _fr) {

  #ifdef SKYBUG
  Serial.print("prepmove going to X = ");
  Serial.print(_newx,4);
  Serial.print(" , ");
  Serial.print(_newy,4);
  Serial.print("  Laser: ");
  Serial.println(_duty);
  #endif


  if (_newx == px && _newy == py) {
    m[bufindexWRITE][0].stationary = true;
  } else {
    m[bufindexWRITE][0].stationary = false;
  }

  //fr = _fr;
  m[bufindexWRITE][0].fr = _fr;

  m[bufindexWRITE][0].rmp = _rmp;
  //pwr = _pwr; //store globally

  m[bufindexWRITE][0].pwr = _pwr;
  m[bufindexWRITE][0].duty = _duty;

  // add contour attribute
  if (contour){
    m[bufindexWRITE][0].CONTOUR = true;
  }
  else{
    m[bufindexWRITE][0].CONTOUR = false;
  }

  if (Skywriting){
    m[bufindexWRITE][0].SKYWRITING = true;
  }
  else{
    m[bufindexWRITE][0].SKYWRITING = false;
  }


  //check frequency
  m[bufindexWRITE][0].freq = (freq > 0) ? freq : oldFreq; //check if the new
  oldFreq = m[bufindexWRITE][0].freq;


  
#ifdef BRESENHAM

  if ((1000000 / (STEPS_PER_MM * m[bufindexWRITE][0].fr)) < one_step_time_limit) {
    float updated_steps_per_mm;

#if defined(Verbose_Serial)
    Serial.println("Feedrate set to maximum with high Res");
#endif
    
    step_factor = (m[bufindexWRITE][0].fr * one_step_time_limit * STEPS_PER_MM) / 1000000; // caclutates the factor between the
    step_factor = ceil(step_factor); //rounding up
    m[bufindexWRITE][0].RES = step_factor;

    updated_steps_per_mm = STEPS_PER_MM / step_factor;
    m[bufindexWRITE][0].movePeriod = (1000000 / ( (updated_steps_per_mm) * m[bufindexWRITE][0].fr)); // reduces step per mm according to step_factor


#if defined(Verbose_Serial)
    Serial.print("step_factor rounded: ");
    Serial.println(step_factor);
    Serial.print("steps_per_mm set to: " );
    Serial.println(updated_steps_per_mm);
    Serial.print("Feedrate should be: ");
    Serial.println(1000000 / (updated_steps_per_mm * m[bufindexWRITE][0].movePeriod));
#endif

  } else {
    m[bufindexWRITE][0].movePeriod = 1000000 / (STEPS_PER_MM * m[bufindexWRITE][0].fr);
  }

  if (!m[bufindexWRITE][0].stationary) {
    checkScannerPosition(_newx, _newy, _checkedposx, _checkedposy);
    _newx = _checkedposx;
    _newy = _checkedposy;

    m[bufindexWRITE][0].maxsteps = 0;//resets so they are not assumed from previous loop


    //find current step position 
    m[bufindexWRITE][0].curPos = (int) round(ADJUSTED_CENTER_X + STEPS_PER_MM * px);
    m[bufindexWRITE][1].curPos = (int) round(ADJUSTED_CENTER_Y + STEPS_PER_MM * py); //current step position along the Y-axis

    //find new pos step
    m[bufindexWRITE][0].newPos = (int) round(ADJUSTED_CENTER_X + STEPS_PER_MM * _newx);
    m[bufindexWRITE][1].newPos = (int) round(ADJUSTED_CENTER_Y + STEPS_PER_MM * _newy);

    //Find direction
    m[bufindexWRITE][0].dir = (int) (( m[bufindexWRITE][0].newPos - m[bufindexWRITE][0].curPos) > 0 ? 1 : -1); //find direction
    m[bufindexWRITE][1].dir = (int) (( m[bufindexWRITE][1].newPos - m[bufindexWRITE][1].curPos) > 0 ? 1 : -1); //find direction´

    int i;
    //initially find max number of steps, half and direction for each galvo
    for (i = 0; i < NUM_MIRRORS; ++i) {
      m[bufindexWRITE][i].delta = abs(m[bufindexWRITE][i].newPos - m[bufindexWRITE][i].curPos) ;
      m[bufindexWRITE][i].over = m[bufindexWRITE][i].delta / 2;

      if ( m[bufindexWRITE][0].maxsteps < m[bufindexWRITE][i].delta) {
        m[bufindexWRITE][0].maxsteps = m[bufindexWRITE][i].delta; //if maxsteps is lower than delta set maxstep to delta
      }
    }
  }
#endif //ending the bresenham specific commands

#ifdef VECTOR
  //changed actual scannermovements to work in mm and floating points values and then only convert to 16-bit on execution.

  m[bufindexWRITE][0].oldPos = px;
  m[bufindexWRITE][1].oldPos = py;

  //find new pos step
  m[bufindexWRITE][0].newPos = _newx;
  m[bufindexWRITE][1].newPos = _newy;

  //find delta distance for x and y
  m[bufindexWRITE][0].delta = _newx - px ;
  m[bufindexWRITE][1].delta = _newy - py ;

  // Calculate delta distance
  m[bufindexWRITE][0].dist = sqrt((0.0 + m[bufindexWRITE][0].delta) * (0.0 + m[bufindexWRITE][0].delta) + (0.0 + m[bufindexWRITE][1].delta) * (0.0 + m[bufindexWRITE][1].delta));
  m[bufindexWRITE][0].moveDurationMicros = m[bufindexWRITE][0].dist * 1000  * 1000 / m[bufindexWRITE][0].fr;
#endif//ending the VECTOR specific commands

  //disable interrupts briefly so the buflen is not overwritten from domove
  //noInterrupts();


  bufindexWRITE++; //increment buf so we write in next next time
  noInterrupts();
  buflen++; //increment buflen
  interrupts();
  if (bufindexWRITE == MAX_BUFLEN + 1) {
    bufindexWRITE = 1;     //reset index if max is reached
  }
  storeScannerPosition_planMove(_newx, _newy);

}



/**
 * Authorizes the launch of the moves that have just been planned
 * The nb commands that have been launched together will be flagged in the same command bundle
 * This is usefull for the galvo settling delay
 * **/
void launch_move(int nb){ 

  nb_launch = nb;
  currently_launching = true;
  int j;
  
  // Authorize launch from last to first command so that it doesn't start launching when not everything is ready
  j = bufindexWRITE-1;
  for (int i = nb; i > 0 ; i--) {
    if (j == 0){
      j = MAX_BUFLEN;
    }
    m[j][0].CMD_READY = true;
    
    // each launch move bundle gets a bool assigned to it
    if (previous_bundle == false){
      m[j][0].CMD_BUNDLE = true;
    }
    else{
      m[j][0].CMD_BUNDLE = false;
    }
    j--;
  }

  //set previous packet bool for next time
  if (previous_bundle == false){
    previous_bundle = true;
  }
  else{
    previous_bundle = false;
  }


}



/**
 * This function receives the commands that were read from serial COM and prepares the logic for the moves
 * **/
void planMove(float _newx, float _newy, uint32_t freq, int _duty, int _pwr, int _rmp, float _fr){

  Skywriting = aux.get_Skywriting();
  if (Skywriting && (_newx != C1.NEWX || _newy != C1.NEWY)){ //Don't use command if it's exactly the same as the last one

    previously_skywriting = true;
    
    if (nb_commands<1){ //get a 2nd command if it's the very first we're getting
      C1.NEWX = _newx;
      C1.NEWY = _newy;
      C1.FREQ = freq;
      C1.DUTY = _duty;
      C1.PWR = _pwr;
      C1.RMP = _rmp;
      C1.FR = _fr;
      nb_commands++;
      m[bufindexWRITE][0].CMD_READY = false; //trigger request new serial command
    #ifdef SKYBUG
      Serial.println("REQUEST");
    #endif
    }

    else{ // if it's not the very first command we got
      C2.NEWX = _newx;
      C2.NEWY = _newy;
      C2.FREQ = freq;
      C2.DUTY = _duty;
      C2.PWR = _pwr;
      C2.RMP = _rmp;
      C2.FR = _fr;

      m[bufindexWRITE][0].CMD_READY = true; // stop requesting commands


      if (C1.DUTY > 0 && !contour && extra_command == 0){ //if Command 1 uses laser 
        
        skywritingplanner(C1.NEWX,C1.NEWY,px,py,C1.FR);//define skywriting start
        
        prepMove(xC,yC,0,0,0,0,C1.FR);//go to new starting position

        launch_move(1);
              
        


        if (C2.DUTY == 0){//C2 is a movement and not a contour:

          skywritingplanner(oldx,oldy,C1.NEWX,C1.NEWY,C1.FR);

          prepMove(oldx,oldy,C1.FREQ,0,0,C1.RMP,C1.FR);
          prepMove(C1.NEWX,C1.NEWY,C1.FREQ,C1.DUTY,C1.PWR,C1.RMP,C1.FR); //first planned move
          prepMove(xC,yC,C1.FREQ,0,0,C1.RMP,C1.FR); //end skywriting move

          launch_move(3);

        }
        else{ //Command 1 and Command 2 use laser so it's a contour
          contour = true;
          prepMove(px,py,C1.FREQ,0,0,C1.RMP,C1.FR);
          prepMove(C1.NEWX,C1.NEWY,C1.FREQ,C1.DUTY,C1.PWR,C1.RMP,C1.FR);
          launch_move(2);
        }

        oldx = px;
        oldy = py;
      }

      else if(C1.DUTY > 0 && contour && extra_command == 0){ // Command 1 uses laser, we are in a contour and we don't have a 3rd command
        if (C2.DUTY > 0){ // Command 2 uses laser: just execute move continue contour
          prepMove(C1.NEWX,C1.NEWY,C1.FREQ,C1.DUTY,C1.PWR,C1.RMP,C1.FR);
          launch_move(1);
          contour = true;
          oldx = px;
          oldy = py;
        }
        else{// Command 2 doesn't use laser: it's the end of the contour andd skywriting length
          skywritingplanner(oldx,oldy,C1.NEWX,C1.NEWY,C1.FR);

          prepMove(C1.NEWX,C1.NEWY,C1.FREQ,C1.DUTY,C1.PWR,C1.RMP,C1.FR);
          prepMove(xC,yC,C1.FREQ,0,0,C1.RMP,C1.FR);

          launch_move(2);
          contour = false;

        }
      }

      else{ //if Command 1 doesn't use laser
        if (C2.DUTY>0 || extra_command > 0){ // Command 2 uses laser OR we have a 3rd Command that we requested

          if (extra_command<1){ //we don't have 3rd command, go get it
            C3 = C1; //store C1 in C3
            //request new serial command
            extra_command++;
          }
          
          else{ // We have the 3rd command

            C2.NEWX = _newx;
            C2.NEWY = _newy;
            C2.FREQ = freq;
            C2.DUTY = _duty;
            C2.PWR = _pwr;
            C2.RMP = _rmp;
            C2.FR = _fr;

            // New command is in C2
            // previous C1 is in C3
            // previous C2 is in C1
            // Order: C3 - C1 - C2

            skywritingplanner(C1.NEWX,C1.NEWY,C3.NEWX,C3.NEWY,C1.FR); //starting part of the skywriting

            prepMove(xC,yC,C3.FREQ,0,0,0,C3.FR); //prep movement and execute it

            launch_move(1);



            if (C2.DUTY>0){ //Command 2 uses laser: it's a contour
              contour = true;
              prepMove(C3.NEWX,C3.NEWY,C3.FREQ,0,0,C3.RMP,C3.FR);
              prepMove(C1.NEWX,C1.NEWY,C1.FREQ,C1.DUTY,C1.PWR,C1.RMP,C1.FR);

              launch_move(2);
              
              oldx = px;
              oldy = py;
            }

            else{//no laser for C2
              //end part skywriting

              skywritingplanner(C3.NEWX,C3.NEWY,C1.NEWX,C1.NEWY,C1.FR);
              
              prepMove(C3.NEWX,C3.NEWY,C3.FREQ,0,0,C3.RMP,C3.FR);//first skywriting part
              prepMove(C1.NEWX,C1.NEWY,C1.FREQ,C1.DUTY,C1.PWR,C1.RMP,C1.FR);//first planned move
              prepMove(xC,yC,C1.FREQ,0,0,C1.RMP,C1.FR); //end skywriting move

              launch_move(3);

            }
            extra_command = 0;
          }

        }
        else{ //Command 1 and Commmand 2 don't use laser
          prepMove(C1.NEWX,C1.NEWY,C1.FREQ,C1.DUTY,C1.PWR,C1.RMP,C1.FR); //Do C1 (go to beginning of C2)
          launch_move(1);
        }
        
       }

    C1 = C2; // store C2 in C1 and start over

    //}
    }
    m[bufindexWRITE][0].CMD_READY = false; //trigger request new command
    #ifdef SKYBUG
    Serial.println("REQUEST");
    #endif
    
    


    #ifdef SKYBUG
    Serial.print("contour:     ");
    for (int i = 1; i < 20; i++) {
      Serial.print(String(m[i][0].CONTOUR)+" ");
    }
    Serial.println(" ");
          Serial.print("CMD bundle:  ");
          for (int i = 1; i < 20; i++) {
            Serial.print(String(m[i][0].CMD_BUNDLE)+" ");
          }
          Serial.println(" ");
    #endif

  }


  else if(!Skywriting){ //skywriting not activated

    m[bufindexWRITE][0].CMD_READY = true; // stop requesting commands
    

    /////////////////////if Skywriting was previously on //////////////////////////


    if (previously_skywriting && contour && _duty == 0){ //we were doing a contour and the new move doens't use laser: finish contour
      Skywriting = true; // used so that prepmove assigns skywriting to these last moves
      skywritingplanner(px,py,C1.NEWX,C1.NEWY,C1.FR);
      prepMove(C1.NEWX,C1.NEWY,C1.FREQ,C1.DUTY,C1.PWR,C1.RMP,C1.PWR);
      prepMove(xC,yC,C1.FREQ,0,0,C1.RMP,C1.FR); //end skywriting move
      Skywriting = false;
      launch_move(2);
      contour = false;
    }

    else if(previously_skywriting && contour && _duty>0){ //if we we doing a contour and the next move the laser is still on (NOT CRUCIAL)
      Skywriting = true;
      prepMove(C1.NEWX,C1.NEWY,C1.FREQ,C1.DUTY,C1.PWR,C1.RMP,C1.FR);//last move in memory
      Skywriting = false;
      launch_move(1);
      contour = false;
    }

    else if(previously_skywriting && extra_command>0 && _duty == 0){// we were in the case where we needed a C3 command
      Skywriting = true;
      skywritingplanner(C1.NEWX,C1.NEWY,C3.NEWX,C3.NEWY,C1.FR);
      prepMove(xC,yC,C3.FREQ,0,0,0,C3.FR);

      prepMove(C3.NEWX,C3.NEWY,C1.FREQ,0,0,C1.RMP,C1.FR);
      prepMove(C1.NEWX,C1.NEWY,C1.FREQ,C1.DUTY,C1.PWR,C1.RMP,C1.FR);

      skywritingplanner(C3.NEWX,C3.NEWY,C1.NEWX,C1.NEWY,C1.FR);
      prepMove(xC,yC,C1.FREQ,0,0,C1.RMP,C1.FR); //end skywriting move
      
      Skywriting = false;
      launch_move(4);
      extra_command = 0;
    }
    else if(previously_skywriting && extra_command>0 && _duty > 0){// we were in the case where we needed a C3 command
      Skywriting = true;
      skywritingplanner(C1.NEWX,C1.NEWY,C3.NEWX,C3.NEWY,C1.FR);
      prepMove(xC,yC,C3.FREQ,0,0,0,C3.FR);

      prepMove(C3.NEWX,C3.NEWY,C1.FREQ,0,0,C1.RMP,C1.FR);
      prepMove(C1.NEWX,C1.NEWY,C1.FREQ,C1.DUTY,C1.PWR,C1.RMP,C1.FR);
      
      Skywriting = false;
      launch_move(3);
      extra_command = 0;
    }

    else if(previously_skywriting && extra_command == 0 && !contour){ //Otherwise do this
      Skywriting = true;
      prepMove(C1.NEWX,C1.NEWY,C1.FREQ,C1.DUTY,C1.PWR,C1.RMP,C1.FR);//last move in memory
      Skywriting = false;
      launch_move(1);
    }

    previously_skywriting = false;
    nb_commands = 0; // reset for skywriting process
    


    /////////////////////Normal non skywriting stuff //////////////////////////

    //Always doing this:
    prepMove(_newx,_newy,freq,_duty,_pwr,_rmp,_fr); // just do the move you reveived
    launch_move(1);

    
    if (!currently_launching){
      m[bufindexWRITE][0].CMD_READY = false; //trigger request new command
      #ifdef SKYBUG
      Serial.println("REQUEST");
      #endif
    }
  }
}


void updateLaserPower(double _newPower, double _currentPower, bool _CommandReady) {
  //update laser power only if the next laser power is different from the previous
  if (_currentPower != _newPower && _CommandReady) {
    _newPower = aux.setPower(_newPower);
    //Serial.println("Set power to: " + String(_newPower));
    return;
  }
}

void updateMovePeriod(double _newMovePeriod, double _currentMoveperiod, bool _CommandReady) {
  //update moveperiod only if the next laser power is different from the previous
  if (_currentMoveperiod != _newMovePeriod && _CommandReady) {
    current_movePeriod = _newMovePeriod;
    //Serial.println("Set move period to: " + String(current_movePeriod));
    Timer1.update(current_movePeriod);
    return;
  }
}

void LaserStartMoveOperations() {

  if (aux.is_the_laser_on() && startLaserWatchDog) {
    //reset laser on-timer every time the laser is turned on - this is done to ensure the watchdog does not terminate the laser prematurelyy
    laser_on_timer = millis();
  }

  if (move_started && !inital_move_routine_completed) {
    if(!sky_time_end){
      updateLaserPower(m[bufindexREAD][0].pwr, aux.get_current_power(), m[bufindexREAD][0].CMD_READY);
    }
    

#ifdef BRESENHAM

    updateMovePeriod(m[bufindexREAD][0].movePeriod, current_movePeriod, m[bufindexREAD][0].CMD_READY);

#endif


    if(m[bufindexREAD][0].SKYWRITING){
        // take care of the circular buffer
        if (bufindexREAD-1 == 0 && prev_index != bufindexREAD){prev_index = MAX_BUFLEN;}
        else if (bufindexREAD-1 == 0 && prev_index == bufindexREAD){prev_index = bufindexREAD;}
        else{prev_index = bufindexREAD-1;}

    }



    if (m[bufindexREAD][0].stationary) {
      // if no scanner movements are required fo this command return now!
      //galvo_allowed_to_settle = true;
      //inital_move_routine_completed = true;
      move_completed = true;
      return;
    }

    else if ((!aux.is_the_laser_on() && m[bufindexREAD][0].duty > 0) || (m[bufindexREAD][0].SKYWRITING && m[prev_index][0].CMD_BUNDLE != m[bufindexREAD][0].CMD_BUNDLE && !m[prev_index][0].CONTOUR)) {
      //if different packages and not a contour PAUSE to settle galvo

      //if the laser is off and the dutycycle in this buf element is !0 ..
      //if Skywriting is on and we are at the intersection between 2 command bundles and it's not a contour

      //do nothing at the beginning of each laser on move so the scanner to allow it to settle
      
      //
      if (!galvo_allowed_to_settle) {
        if (scanner_settling_timer == 0) {
          scanner_settling_timer = micros();
        }

        settling_timer_storage = micros() - scanner_settling_timer; // calculation since the move was started
        

        if (settling_timer_storage >= SCANNER_SETTLING_TIME || (m[bufindexREAD][0].SKYWRITING && !aux.is_the_laser_on() && m[bufindexREAD][0].duty > 0)) {
          // Settling time bypassed if skywriting is on and we need to turn the laser on
          galvo_allowed_to_settle = true;

          #ifdef SKYBUG
            if ((m[bufindexREAD][0].SKYWRITING && !aux.is_the_laser_on() && m[bufindexREAD][0].duty > 0)){
              Serial.println("settling delay bypassed");
            }
            else{
              Serial.println("waited for settling delay");
            }
          #endif
        }
        

      } 
      else { //when the scanner has been allowed to settle before a laser off to on movement

        if (step_response_timer == 0) { // start timer allowing allowing the scanner response delay to pass before turning on the laser
          step_response_timer = micros();
        }

        timer_storage_step_response = micros() - step_response_timer; //calculation since move was started
        
        //STEP_RESPONSE_DELAY
        if ((timer_storage_step_response >= STEP_RESPONSE_DELAY)) {
          
          aux.trigLaser(m[bufindexREAD][0].freq, m[bufindexREAD][0].duty);  // .. turn laser on
          if (aux.is_the_laser_on() && LASER_STARTUP_DELAY > 0) {
            //small delay laser ramp up whileloop - should not make much havoc. allows accurate laser delay wait
            laser_ramp_up_timer = micros();
            while ((micros() - laser_ramp_up_timer) <= LASER_STARTUP_DELAY && LASER_STARTUP_DELAY > 0) {
            }
          }
          // initial move stuff done
          inital_move_routine_completed = true;
          return;
        }
      }
    }
    else { // if the laser should not turn on - realese the scanner movemements immediatly
      galvo_allowed_to_settle = true;
      inital_move_routine_completed = true;
      return;
    }
  }
}

void EndOfMoveOperations() {

  if (move_completed) {
    if (!m[bufindexREAD][0].stationary) {

      if (tracking_error_timer == 0) {
        tracking_error_timer = micros();
      }

      // lookahead stuff to keep the laser off inbetween moves (typ within the contour)
      if (bufindexREAD == MAX_BUFLEN) { // if the buffer reaches max then look at the beginning of the next round
        look_ahead_variable = 1;
      }  
      else {
        look_ahead_variable = bufindexREAD + 1;
      }

      if ((aux.is_the_laser_on() && m[look_ahead_variable][0].duty > 0 && m[look_ahead_variable][0].CMD_READY && !laser_handled)) {
        //if next command is a laser move keep the laser on
        laser_handled = true;
      }

      else {

        if (m[bufindexREAD][0].SKYWRITING  && inital_move_routine_completed){
          
          if (m[bufindexREAD][0].duty > 0 && m[look_ahead_variable][0].duty == 0){
            sky_time_end = true;
          }
          laser_handled = true;

        }
        else if (!m[bufindexREAD][0].SKYWRITING){ //Skywriting is turned off 
          // turn off the laser
          //start tracking error delay
          timer_storage = micros() - tracking_error_timer; //calculation since move signal was halted

          
          if (((timer_storage >= TRACKING_DELAY || timer_storage >= TRACKING_DELAY - TRACKING_DELAY_OFFSET) && !laser_handled)) {
            //Serial.println("Tracking Delay: " + String(timer_storage));
            aux.trigLaser(m[bufindexREAD][0].freq, 0); // turn the laser off
            laser_handled = true;
          }
        }
      }
    }

    if (laser_handled || m[bufindexREAD][0].stationary) {

      tracking_error_timer = 0;

      timer_storage = 0;
      step_response_timer = 0;
      timer_storage_step_response = 0;
      scanner_settling_timer = 0;
      settling_timer_storage = 0;
      laser_handled = false;

      noInterrupts();
      inital_move_routine_completed = false;
      //if (Skywriting){
        m[bufindexREAD][0].CMD_READY = false; // current command is used, and will not be considered again
      //}
      
      buflen--; //commands executed decrement buffer
      
      move_completed = false; // toogles if any domove operations are allowed
      move_started = false; // toggles initial
      galvo_allowed_to_settle = false;

      bufindexREAD++; // move on to next command
      
      

      if (bufindexREAD == MAX_BUFLEN + 1) {
        bufindexREAD = 1;
      }
      interrupts();

      //Serial.println("move completed");

    }
  
  }

  // This is used for the end delay, keep it on regardless of position, TIME based
  if (sky_time_end){

    if (tracking_error_timer == 0) {
        tracking_error_timer = micros();
    }
      // turn off the laser
      //start tracking error delay
      timer_storage = micros() - tracking_error_timer; //calculation since move signal was halted
      
      //TRACKING_DELAY
      if (timer_storage >= TRACKING_DELAY || timer_storage >= TRACKING_DELAY - TRACKING_DELAY_OFFSET) {
        
        //time_end = time_end + 30;
        //Serial.println("Tracking Delay: " + String(timer_storage));
        aux.trigLaser(m[bufindexREAD][0].freq, 0); // turn the laser off
        updateLaserPower(m[bufindexREAD][0].pwr, aux.get_current_power(), m[bufindexREAD][0].CMD_READY);
        sky_time_end = false;

      }
    
  }
}

void doMove() {
  //unsigned long tt = micros();
  //if a commands is ready in the target buffer element, start the magic

  if (buflen > 0 && m[bufindexREAD][0].CMD_READY) {

    //----->>> begin trajectory logic
    if (!inital_move_routine_completed && !move_started) {
      move_started = true;
      return;
    }



#ifdef BRESENHAM

    //if (m[bufindexREAD][0].maxsteps > 0 && !move_completed) {
    if (!m[bufindexREAD][0].stationary) {
      if (stepcount < (m[bufindexREAD][0].maxsteps / m[bufindexREAD][0].RES) && !move_completed && galvo_allowed_to_settle ) {

        for (int j = 0; j < NUM_MIRRORS; ++j) {
          m[bufindexREAD][j].over += m[bufindexREAD][j].delta;
          if (m[bufindexREAD][j].over >=  m[bufindexREAD][0].maxsteps) {
            m[bufindexREAD][j].over -=  m[bufindexREAD][0].maxsteps;
            m[bufindexREAD][j].curPos += m[bufindexREAD][j].dir * m[bufindexREAD][0].RES;
          }
        }
        galvo.setXY(m[bufindexREAD][0].curPos, m[bufindexREAD][1].curPos);
        CURRENT_POS_X = m[bufindexREAD][0].curPos;
        CURRENT_POS_Y = m[bufindexREAD][1].curPos;
        //        Serial.println("CURPOS_X: " + String(CURRENT_POS_X) + ", CURPOS_Y: " + String(CURRENT_POS_Y));
        //        Serial.println("NEWPOS_X: " + String(m[bufindexREAD][0].newPos) + ", NEWPOS_Y: " + String(m[bufindexREAD][1].newPos));
        //        Serial.println("maxsteps: " + String(m[bufindexREAD][0].maxsteps / m[bufindexREAD][0].RES) + "; stepcount: " + String(stepcount));
        stepcount++;
      //  storeToDataFileString();
      }

      //Accounting for buffer
      //only decrement if move is done eg stepcount = maxsteps
      //check if the final step is new pos otherwise run extra increment
      else if (stepcount >= (m[bufindexREAD][0].maxsteps / m[bufindexREAD][0].RES) && !move_completed) {
        //if (stepcount >= (m[bufindexREAD][0].maxsteps / m[bufindexREAD][0].RES) + 1 && !move_completed) {
        // if the stepcount is reached ensure the last position set is the new position
        if (m[bufindexREAD][0].curPos != m[bufindexREAD][0].newPos ||  m[bufindexREAD][1].curPos != m[bufindexREAD][1].newPos) {
          m[bufindexREAD][0].curPos = m[bufindexREAD][0].newPos;
          m[bufindexREAD][1].curPos = m[bufindexREAD][1].newPos;
          galvo.setXY(m[bufindexREAD][0].curPos, m[bufindexREAD][1].curPos);
          CURRENT_POS_X = m[bufindexREAD][0].curPos;
          CURRENT_POS_Y = m[bufindexREAD][1].curPos;
        }
        //Serial.println("end: X: " + String(CURRENT_POS_X) + " ; Y: " + String(CURRENT_POS_Y));
        move_completed = true;
        stepcount = 0; // reset stepcount for next round

        #ifdef SKYBUG
        Serial.println("doMove completed:  "+String((CURRENT_POS_X - ADJUSTED_CENTER_X)/STEPS_PER_MM)+" , "+String((CURRENT_POS_Y - ADJUSTED_CENTER_X)/STEPS_PER_MM));
        #endif


        if (currently_launching){
          //Serial.println("doMove Status:  "+String(nb_launch)+" - "+String(nb_doMove)+" = "+String((nb_launch - nb_doMove)));
          if((nb_launch - nb_doMove) == 0){
            currently_launching = false;
            nb_doMove = 0;
            #ifdef SKYBUG
            Serial.println("DONE");
            Serial.println("REQUEST");
            #endif
            m[bufindexWRITE][0].CMD_READY = false;
            
          }
          else{
            nb_doMove++;
            m[bufindexWRITE][0].CMD_READY = false;
          }
        }

        
        
      }
   
#endif
    }//<-- bracked ends the movement loop
  }
}

//#ifdef VECTOR
//      unsigned long _NOW = micros(); // this is the time from which everything within this move is calculated
//      didstuff = false;
//
//      // this is done initially on all commands
//      if (m[bufindexREAD][0].FIRST) {
//        move_started = true;
//
//        m[bufindexREAD][0].FIRST = false;
//
//        /// ----- INTERPOLATION MOVE STUFF
//        CMD_START_MICROS = _NOW;
//        CMD_END_MICROS = _NOW +  m[bufindexREAD][0].moveDurationMicros; //ETA: How long time do need to move before reaching the postion
//      }
//
//      if (_NOW <= CMD_END_MICROS) { // we still have some time to go before reaching the target - lets go arround again!
//
//        didstuff = true;
//
//        double fraction_of_move = (double)(_NOW - CMD_START_MICROS) / m[bufindexREAD][0].moveDurationMicros;
//        //Serial.println("fraction of move: " + String(fraction_of_move,4));
//
//        //      Serial.println("oldpos: " + String(m[bufindexREAD][0].oldPos));
//        CURRENT_CMD_X = (m[bufindexREAD][0].oldPos + (m[bufindexREAD][0].delta * fraction_of_move));
//        CURRENT_CMD_Y = (m[bufindexREAD][1].oldPos + (m[bufindexREAD][1].delta * fraction_of_move));
//
//        CURRENT_CMD_X =  CURRENT_POS_X;
//        CURRENT_CMD_Y =  CURRENT_POS_Y;
//
//      } else { // done moving
//        move_completed = true;
//        //Serial.println("Move duration: " +String(micros() - _NOW));
//        CURRENT_CMD_X = m[bufindexREAD][0].newPos;
//        CURRENT_CMD_Y = m[bufindexREAD][1].newPos;
//
//        //EndOfMoveOperations();
//
//      }
//      // setScannerPosition(CURRENT_CMD_X, CURRENT_CMD_Y);
//
//#endif //ends vector specific move algo

//    }//<-- bracked ends the movement loop
//    // domovedur = (micros() - tt);
//  }
//}
