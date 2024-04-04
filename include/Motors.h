#include <AccelStepper.h> // Stepper Library enabling acceleration
#include <MultiStepper.h>
#include <Bounce2.h> // Button/Limit switch library to remove bounce

//------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------

#ifdef LOOP
// Microstepping
int microZ = 32L;
int microP = 2L;
int microR = 8L;

unsigned int minWidth = 45;

// ---------- remap pin for loop

int enableRec = ENABLE_1_2_3;
const int endRec = ESTP1;

// ---------------- Z AXIS VARIABLES ---------------------

// Lead screw per step [mm] = 0.01.
// Steps per mm:
long spmZ = 3250; //microZ / 0.01;
double spumZ = spmZ / 1000;

long Zpos = 0;            // Z position in steps with mm input
long Zposum = 0;          // Z position in steps with um input

float Zacceleration = 0;  // Z acceleration for user input, steps/s^2 with mm/s^2 input
float Zspeed = 0;         // Z speed for user input, steps/s with mm/s input

float Norm_Zacceleration = 0.5 * spmZ;  // Z acceleration normal use, steps/s^2
float Norm_Zspeed = 0.5 * spmZ;          // Z speed normal use, steps/s

float zClear = 0.800;         //The movement of the z-axis when recoater moving back, in mm
double zReverse = 0.3;        // The movement to take care of backlash

float ZposClear = zClear * spmZ; // The movement of the z-axis, when recoater moving back, in steps
float zposReverse = zReverse * spmZ;

float zMaxCorrection = 0.2; // Maximum movement of stepper motors in alignment mode

double currentZPosition = 0;



// ---------------- POWDER VARIABLES ---------------------
//Approx.!

int DoseRotation = 4145; // For powder roller 2


// Doser 1
int sprP = 4146; //Steps per rot with gearing //microGear*(200*(5+2/11));    // Steps per revolution
float StepPerDeg = sprP/360;
int OneDoseDeg = 120;
float OneDose = OneDoseDeg * StepPerDeg;
int EmptyDoseDeg = 180-OneDoseDeg;
float EmptyDose = EmptyDoseDeg * StepPerDeg;

float DosingSteps = 0;


int DoseCounter = 1;    // Incrementing number of dispensed doses

int Norm_P_speed = 1000;   //Normal speed without user input
int Norm_P_acceleration = 500;   // Normal acceleration without user input

bool Hopper1Flag = false;
bool Hopper2Flag = false;
bool Hopper3Flag = false;


// ---------------- RECOATER VARIABLES ---------------------


double spmRec = 17.78;      // Steps per mm, (200*micro)/(18*5)

long posRec = 0L;           // Recoater postion with mm input

long speedRec = 0L;         // Recoater speed in steps/s, with mm/s input
long recAccStep = 0L;       // Recoater acceleration in steps/s^2, with mm/s^2 input


const long recoatDistance_steps = 455 * spmRec; // Recoating cycle distance in steps //455
long extraHomeMove = 2 * spmRec;
long Norm_speedRec = 120 * spmRec;              // Recoating speed during normal use in steps/s
long Norm_recAccStep = 600 * spmRec;            // Recoating acceleration during normal use in steps/s^2

//const long recoatDistance_steps = round(450 * spmRec); // Recoating cycle distance in steps
//int extraHomeMove = round(10 * spmRec);
//int Norm_speedRec = round(150 * spmRec);              // Recoating speed during normal use in steps/s
//int Norm_recAccStep = round(800 * spmRec);            // Recoating acceleration during normal use in steps/s^2

int homingSpeed = round(30 * spmRec);                 // Homing speed in steps/mm
int homingAcceleration = round(400 * spmRec);         // Homing acceleration in steps/s^2

#endif


#ifdef OPAL
// ADD OPAL SPECIFIC
#endif


//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------

// set motors
typedef struct {
  volatile int delta = 0; // number of steps to move
  volatile int curPos = 0; // to remember where we are
  volatile int newPos = 0; // new position
  volatile int dir = 0; //direction to increment dac
  volatile int maxsteps = 0; //maxumum increments needed
  volatile int wait = 0; // duration of break in between steps
  volatile int stepprmm = 0;//
} Motor;

//------------------------------------------------------------------------------
// MOTOR AND LIMIT SWITCH INITIALIZATION (ACCELSTEPPER AND BOUNCE2)
//------------------------------------------------------------------------------

Bounce2::Button limitRec = Bounce2::Button();    // Recoater limit switch object for bounce library

// --- Z-axis steppers ---
AccelStepper stepperZ1(AccelStepper::DRIVER, STEP4, DIR4);
AccelStepper stepperZ2(AccelStepper::DRIVER, STEP5, DIR5);
AccelStepper stepperZ3(AccelStepper::DRIVER, STEP6, DIR6);


// --- Powder steppers ---
AccelStepper stepperP1(AccelStepper::DRIVER, STEP2, DIR2);
AccelStepper stepperP2(AccelStepper::DRIVER, STEP3, DIR3);

// --- Recoater stepper ---
AccelStepper stepperRec(AccelStepper::DRIVER, STEP1, DIR1);


void limitSwitchInit() {
#ifdef LOOP
  ////////////////// LIMIT SWITCHES /////////////////////
  limitRec.attach(ESTP1, INPUT_PULLUP);    // Initialize bounce limit switch for recoater
  limitRec.interval(5);                     // Length of bounce interval
  limitRec.setPressedState(LOW);            // State of button when pressed
#endif
}
void stepperInit() {

#ifdef LOOP
  ///
  stepperZ1.setCurrentPosition(0);
  stepperZ2.setCurrentPosition(0);
  stepperZ3.setCurrentPosition(0);
  stepperZ1.setPinsInverted (/*direction*/ true, /*step*/ false, /*enable*/ false);
  stepperZ2.setPinsInverted (/*direction*/ true, /*step*/ false, /*enable*/ false);
  stepperZ3.setPinsInverted (/*direction*/ true, /*step*/ false, /*enable*/ false);
  // Set Powder Stepper Position
  stepperP1.setCurrentPosition(0);
  stepperP2.setCurrentPosition(0);
  // Set Recoater Stepper Position
  // stepperRec.setPinsInverted (/*direction*/ false, /*step*/ true, /*enable*/ false);
  stepperRec.setCurrentPosition(0);

  // Set speed to normal speed
  stepperZ1.setMaxSpeed(Norm_Zspeed);
  stepperZ2.setMaxSpeed(Norm_Zspeed);
  stepperZ3.setMaxSpeed(Norm_Zspeed);
  stepperRec.setMaxSpeed(Norm_speedRec);
  // Set acceleration to normal
  stepperZ1.setAcceleration(Norm_Zacceleration);
  stepperZ2.setAcceleration(Norm_Zacceleration);
  stepperZ3.setAcceleration(Norm_Zacceleration);
  stepperRec.setAcceleration(Norm_recAccStep);

  unsigned int   minWidth = 40;
  stepperZ1.setMinPulseWidth(minWidth);
  stepperZ2.setMinPulseWidth(minWidth);
  stepperZ3.setMinPulseWidth(minWidth);

  stepperP1.setMinPulseWidth(minWidth);
  stepperP2.setMinPulseWidth(minWidth);
  stepperRec.setMinPulseWidth(minWidth);


  // int PowderFeedrate = 0;
  //int PowderAcceleration = 0;

  // Disable recoater and powder steppers
  digitalWrite(ENABLE_1_2_3, HIGH);

  // Disable Z-axis
  digitalWrite(ENABLE_4_5_6, LOW);

  delay(200);
#endif
}

void UpdateSteppers() {
  stepperZ1.run();
  stepperZ2.run();
  stepperZ3.run();
  stepperP1.run();
  stepperP2.run();
  stepperRec.run();
}

/// M7 HOMING ROUTINE ///

void HomingRecoater() {

#ifdef LOOP
  digitalWrite(ENABLE_1_2_3, LOW);
  delay(500);

  long homing = -1; // Used to Home Stepper
  stepperRec.setMaxSpeed(homingSpeed);
  stepperRec.setAcceleration(homingAcceleration);

  while (limitRec.read() == 0) {  // Make the Stepper move CCW until the switch is activated
    stepperRec.moveTo(homing);  // Set the position to move to
    homing--;  // Decrease by 1 for next move if needed
    stepperRec.run();  // Start moving the stepper
    limitRec.update();
  }

  stepperRec.setCurrentPosition(0);  // Set the current position as zero for now
  stepperRec.setMaxSpeed(homingSpeed);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepperRec.setAcceleration(homingAcceleration);  // Set Acceleration of Stepper
  homing = 1;

  while (limitRec.read() == 1) { // Make the Stepper move CW until the switch is deactivated
    stepperRec.moveTo(homing);
    stepperRec.run();
    homing++;
    limitRec.update();
  }

  stepperRec.move(extraHomeMove);
  while ((stepperRec.distanceToGo() != 0)) {
    stepperRec.run();  // Move Stepper into position
  }


  stepperRec.setCurrentPosition(0);
  Serial.println("Homing Completed");
  Serial.println("");

  digitalWrite(ENABLE_1_2_3, HIGH);
#endif

  //-------------------

#ifdef OPAL

// add OPAL SPECIFIC

#endif
}


/// M0 Multi-Material Control ///
void MultiMaterial(int HopperSelect) {
  #ifdef LOOP
  
if (HopperSelect == 1) {
  Hopper1Flag = true;
  Hopper2Flag = false;
  Serial.println("Dispenser 1 (Back) is- now active.");
}
else if (HopperSelect == 2) {
  Hopper2Flag = true;
  Hopper1Flag = false;
  Serial.println("Dispenser 2 (Front) is- now active.");
}
else if (HopperSelect == 3) {
  Hopper1Flag = true;
  Hopper2Flag = true;
  Hopper3Flag = true;
  Serial.println("Both dispensers are active");
} else {
  Serial.println("Please choose either dispenser 1 or 2, or type 3 for both.");
}

  #endif
}


/// M1 Powder motors ///
void Powder(int Hopper, double NoDose, float PowderFeedrate, float PowderAcceleration, int Steps) {
#ifdef LOOP
  //control dosing
  if (PowderFeedrate == 0) {
    stepperP1.setMaxSpeed(Norm_P_speed);
    stepperP2.setMaxSpeed(Norm_P_speed);
  }
  else {
    stepperP1.setMaxSpeed(PowderFeedrate * sprP);
    stepperP2.setMaxSpeed(PowderFeedrate * sprP);
  }

  if (PowderAcceleration == 0) {
    stepperP1.setAcceleration(Norm_P_acceleration);
    stepperP2.setAcceleration(Norm_P_acceleration);
  }
  else {
    stepperP1.setAcceleration(PowderAcceleration * sprP);
    stepperP2.setAcceleration(PowderAcceleration * sprP);
  }


  

    if (Hopper == 1) {    // Back hopper
      if (Hopper1Flag == true){
        
      Serial.print("Dosing from hopper: ");
      Serial.print(Hopper);
      Serial.print(". Doses: ");
      Serial.println(NoDose);
      
      stepperP1.setPinsInverted (/*direction*/ true, /*step*/ false, /*enable*/ false);
      digitalWrite(ENABLE_1_2_3, LOW);    // Enable powder stepper
      delay(500);

      DosingSteps = NoDose * sprP/3; //OneDose;
      int tempDose = round(DosingSteps);
      stepperP1.move(tempDose); //Loading powder

      while ((stepperP1.distanceToGo() != 0)) {
        stepperP1.run();  // Move Stepper into position
          }
      }  else {
        Serial.println("Dispenser 1 is not active. Use M0");
      }
  
    }
  else if (Hopper == 2) { // Front hopper

    if (Hopper2Flag == true){

      Serial.print("Dosing from hopper: ");
      Serial.print(Hopper);
      Serial.print(". Doses: ");
      Serial.println(NoDose);
      
    stepperP2.setPinsInverted (/*direction*/ false, /*step*/ false, /*enable*/ false);
    digitalWrite(ENABLE_1_2_3, LOW);    // Enable powder stepper
    delay(500);

  
      int TempMove = DoseRotation*NoDose;
      stepperP2.move( TempMove ); //Loading powder

      while ((stepperP2.distanceToGo() != 0)) {
        stepperP2.run();  // Move Stepper into position
      }

    }
    else{
      Serial.println("Dispenser 2 is not active. Use M0");
    }
  }
    else if (Hopper == 3){    // Both hoppers
    
        if (Hopper3Flag == true){

              Serial.println("Dosing from both hoppers: ");
              Serial.print("Doses: ");
              Serial.println(NoDose);
              
            stepperP1.setPinsInverted (/*direction*/ true, /*step*/ false, /*enable*/ false);
            stepperP2.setPinsInverted (/*direction*/ false, /*step*/ false, /*enable*/ false);
            digitalWrite(ENABLE_1_2_3, LOW);    // Enable powder stepper
            delay(500);
        
              DosingSteps = NoDose * sprP/3;
              int tempDose = round(DosingSteps);
              
              stepperP1.move(tempDose); //Loading powder
              stepperP2.move(tempDose); //Loading powder
        
              while ((stepperP2.distanceToGo() != 0)) {
                stepperP1.run();  // Move Stepper into position
                stepperP2.run();  // Move Stepper into position
              }

    }

     else{
        Serial.println("Dispensers (3) is not active. Use M0");
      }
    
    
  }
  
  else if (Hopper != 1 || Hopper != 2) {
    Serial.println("You need to select a hopper. Please choose 1 or 2.");
  }


  // RESETTING EVERYTHING
  digitalWrite(ENABLE_1_2_3, HIGH);
  PowderFeedrate = 0; PowderAcceleration = 0; DoseCounter = 1; Hopper = 0;
  stepperP1.setPinsInverted (/*direction*/ false, /*step*/ false, /*enable*/ false);
  stepperP2.setPinsInverted (/*direction*/ false, /*step*/ false, /*enable*/ false);
  #endif
}

/// M2 Z-AXIS control ///

void Zaxis(float pos_um) {
#ifdef BAXTER
  // do nothing will have to be handled on a seperate comport
#endif
#ifdef LOOP
  // set position of zaxis

  Zposum = pos_um * spmZ;

  stepperZ1.setMaxSpeed(Norm_Zspeed);
  stepperZ2.setMaxSpeed(Norm_Zspeed);
  stepperZ3.setMaxSpeed(Norm_Zspeed);
  stepperZ1.setAcceleration(Norm_Zacceleration);
  stepperZ2.setAcceleration(Norm_Zacceleration);
  stepperZ3.setAcceleration(Norm_Zacceleration);

  stepperZ1.move(Zposum);
  stepperZ2.move(Zposum);
  stepperZ3.move(Zposum);


  while ((stepperZ1.distanceToGo() != 0) && (stepperZ2.distanceToGo() != 0) && (stepperZ3.distanceToGo() != 0)) {

    stepperZ1.run();  // Move Stepper into position
    stepperZ2.run();  // Move Stepper into position
    stepperZ3.run();  // Move Stepper into position

  }

  Serial.print("Moving the build-plate ");
  Serial.print(pos_um*1000);
  Serial.println("um");

  currentZPosition = double(stepperZ1.currentPosition());
  
  Serial.print("Current position: ");
  Serial.print(currentZPosition/1600);
  Serial.println("mm");
#endif

  //--------------------------------------------

#ifdef OPAL
  //add OPAL CODE
#endif
}


/// M3 ONE COMPLETE RECOATING CYCLE ///

void LayerCycle() {   // Recoating routine incl. build plate movements

#ifdef BAXTER
  // do nothing will have to be handled on a seperate comport
#endif

  //--------------------------------------------

#ifdef LOOP
  // CYCLE SETTINGS

  if (stepperRec.currentPosition() != 0) {
    HomingRecoater();
  }

  digitalWrite(ENABLE_1_2_3, LOW);
  delay(500);
  stepperRec.setMaxSpeed(Norm_speedRec);
  stepperRec.setAcceleration(Norm_recAccStep);
  stepperZ1.setMaxSpeed(Norm_Zspeed);
  stepperZ2.setMaxSpeed(Norm_Zspeed);
  stepperZ3.setMaxSpeed(Norm_Zspeed);
  stepperZ1.setAcceleration(Norm_Zacceleration);
  stepperZ2.setAcceleration(Norm_Zacceleration);
  stepperZ3.setAcceleration(Norm_Zacceleration);

  // RECOATER TO THE FRONT


  stepperRec.moveTo(recoatDistance_steps); // Going to the front

  while ((stepperRec.distanceToGo() != 0)) {

    stepperRec.run();  // Move Stepper into position

  }

  Serial.println("Recoater to front");

  // BUILD PLATE DOWN


  stepperZ1.move(ZposClear);
  stepperZ2.move(ZposClear);
  stepperZ3.move(ZposClear);

  Serial.print("Moving the build-plate out of the way ");
  Serial.print(zClear);
  Serial.println("um");


  while ((stepperZ1.distanceToGo() != 0) && (stepperZ2.distanceToGo() != 0) && (stepperZ3.distanceToGo() != 0)) {

    stepperZ1.run();  // Move Stepper into position
    stepperZ2.run();  // Move Stepper into position
    stepperZ3.run();  // Move Stepper into position

  }


  // RECOATER TO HOME

  stepperRec.moveTo(0); // Going home

  while ((stepperRec.distanceToGo() != 0)) {

    stepperRec.run();  // Move Stepper into position

  }

  Serial.println("Recoater is home");


  // BUILD PLATE BACK TO POSITION

  stepperZ1.move(- (ZposClear + zposReverse));
  stepperZ2.move(- (ZposClear + zposReverse));
  stepperZ3.move(- (ZposClear + zposReverse));

  while ((stepperZ1.distanceToGo() != 0) && (stepperZ2.distanceToGo() != 0) && (stepperZ3.distanceToGo() != 0)) {

    stepperZ1.run();  // Move Stepper into position
    stepperZ2.run();  // Move Stepper into position
    stepperZ3.run();  // Move Stepper into position

  }

  stepperZ1.move(zposReverse);
  stepperZ2.move(zposReverse);
  stepperZ3.move(zposReverse);

  while ((stepperZ1.distanceToGo() != 0) && (stepperZ2.distanceToGo() != 0) && (stepperZ3.distanceToGo() != 0)) {

    stepperZ1.run();  // Move Stepper into position
    stepperZ2.run();  // Move Stepper into position
    stepperZ3.run();  // Move Stepper into position

  }

  Serial.println("Recoating routine done");
  digitalWrite(ENABLE_1_2_3, HIGH);

#endif
  //--------------------------------------------
#ifdef OPAL
  //add layer code
#endif

}


/// M4 RECOAT TO THE FRONT AND WAIT ///
void RecoatPowder(){
#ifdef LOOP

  digitalWrite(ENABLE_1_2_3, LOW);
  delay(500);
  stepperRec.setMaxSpeed(Norm_speedRec);
  stepperRec.setAcceleration(Norm_recAccStep);

  // RECOATER TO THE FRONT


  stepperRec.moveTo(recoatDistance_steps); // Going to the front

  while ((stepperRec.distanceToGo() != 0)) {

    stepperRec.run();  // Move Stepper into position

  }
  digitalWrite(ENABLE_1_2_3, HIGH);
  Serial.println("Recoater to front");

#endif

#ifdef BAXTER
  //do nothing will have to done on the computer side
#endif

#ifdef OPAL
  //add layer code
#endif


}

/// M5 RETRACTING RECOATER ///
void RecoatRetract() {
  #ifdef LOOP

  
  stepperRec.setMaxSpeed(Norm_speedRec);
  stepperRec.setAcceleration(Norm_recAccStep);

  // RECOATER TO HOME
  
  if (stepperRec.currentPosition() == recoatDistance_steps) { // If the recoater is in the forward position
      digitalWrite(ENABLE_1_2_3, LOW);
      delay(500);
      stepperRec.moveTo(0); // Going home

      while ((stepperRec.distanceToGo() != 0)) {
          stepperRec.run();  // Move Stepper into position
                                              }
      Serial.println("Recoater is home");
      
  } else if (stepperRec.currentPosition() == 0) {
      Serial.println("Recoater is already home");
  
  } else {
    HomingRecoater();
  }
  

  #endif


  #ifdef BAXTER
  //do nothing will have to done on the computer side
#endif

#ifdef OPAL
  //add layer code
#endif
}



/// M6 RECOATING ROUTINE ///
void Recoater() {
#ifdef BAXTER
  //do nothing will have to done on the computer side
#endif

#ifdef LOOP

  if (stepperRec.currentPosition() != 0) {
    HomingRecoater();
  }

  digitalWrite(ENABLE_1_2_3, LOW);
  delay(500);

  stepperRec.setMaxSpeed(Norm_speedRec);
  stepperRec.setAcceleration(Norm_recAccStep);


  stepperRec.moveTo(recoatDistance_steps); // Going to the front

  while ((stepperRec.distanceToGo() != 0)) {

    stepperRec.run();  // Move Stepper into position

  }

  stepperRec.moveTo(0); // Going home

  while ((stepperRec.distanceToGo() != 0)) {

    stepperRec.run();  // Move Stepper into position
  }

  Serial.println("Recoating done");
  digitalWrite(ENABLE_1_2_3, HIGH);
#endif

#ifdef OPAL
  //ADD OPAL
#endif
}

/// M8 FULL Z-AXIS CONTROL ///

#ifdef LOOP
void ManualZaxis(float pos_mm, float Zspeed_mm, float Zacceleration_mm, int motor, float posum) {

  Zpos = pos_mm * spmZ;
  Zspeed = Zspeed_mm * spmZ;
  Zacceleration = Zacceleration_mm * spmZ;

  Zposum = posum * spmZ;

  if (motor == 0) {
    stepperZ1.setMaxSpeed(Zspeed);
    stepperZ2.setMaxSpeed(Zspeed);
    stepperZ3.setMaxSpeed(Zspeed);
    stepperZ1.setAcceleration(Zacceleration);
    stepperZ2.setAcceleration(Zacceleration);
    stepperZ3.setAcceleration(Zacceleration);

    stepperZ1.move(Zpos);
    stepperZ2.move(Zpos);
    stepperZ3.move(Zpos);

    Serial.print("Moving the build-plate ");
    Serial.print(pos_mm);
    Serial.println("mm");

  }


  else if (motor == 1 && abs(posum) <= zMaxCorrection) {
    stepperZ1.setMaxSpeed(Zspeed);
    stepperZ1.setAcceleration(Zacceleration);
    stepperZ1.move(Zposum);

    Serial.print("Moving motor 1 ");
    Serial.print(posum);
    Serial.println("um");

    motor = 0;
  }

  else if (motor == 2 && abs(posum) <= zMaxCorrection) {
    stepperZ2.setMaxSpeed(Zspeed);
    stepperZ2.setAcceleration(Zacceleration);
    stepperZ2.move(Zposum);

    Serial.print("Moving motor 2 ");
    Serial.print(posum);
    Serial.println("um");

    motor = 0;
  }

  else if (motor == 3 && abs(posum) <= zMaxCorrection) {
    stepperZ3.setMaxSpeed(Zspeed);
    stepperZ3.setAcceleration(Zacceleration);
    stepperZ3.move(Zposum);

    Serial.print("Moving motor 3 ");
    Serial.print(posum);
    Serial.println("um");

    motor = 0;
  }

  else if (abs(posum) > zMaxCorrection) {

    Serial.println("You are trying to move the motor too far.");
    Serial.println("Please only use increments of 0.2mm or smaller");
    Serial.println("or the z-axis may be damaged.");

    motor = 0;
  }

}
#endif

#ifdef LOOP
/// M11 FULL RECOATER CONTROL ///

void ManualRecoater(float manual_feedrate, float recDistance, float recAcceleration) {


  posRec = recDistance * spmRec;
  stepperRec.moveTo(posRec);

  if (stepperRec.distanceToGo() != 0) {
    digitalWrite(ENABLE_1_2_3, LOW);
    delay(500);
    speedRec = spmRec * manual_feedrate;
    stepperRec.setMaxSpeed(speedRec);
    recAccStep = recAcceleration * spmRec;
    stepperRec.setAcceleration(recAccStep);

    Serial.print("Moving recoater ");
    Serial.print(recDistance);
    Serial.println("mm");
    Serial.print("Speed = ");
    Serial.print(manual_feedrate);
    Serial.print("mm/s, and acceleration ");
    Serial.print(recAcceleration);
    Serial.println("mm/s^2");

    //Serial.println(stepperRec.distanceToGo());
    while ((stepperRec.distanceToGo() != 0)) {

      stepperRec.run();  // Move Stepper into position
    }

    Serial.println("Done");

    digitalWrite(ENABLE_1_2_3, HIGH);
  }

  else if (stepperRec.distanceToGo() == 0) {
    Serial.print("The recoater is already at position: ");
    Serial.println(stepperRec.currentPosition());
    Serial.print("Choose another (absolute) position.");
  }

}


/// M12 ZERO Z-AXIS ///

void ResetZaxis() {
#ifdef BAXTER
  // do nothing will have to be handled on a seperate comport
#endif

#ifdef LOOP

  stepperZ1.setCurrentPosition(0);
  stepperZ2.setCurrentPosition(0);
  stepperZ3.setCurrentPosition(0);


  Serial.println("Current position of z-axis now set to zero.");
  Serial.println("Z=0mm");

#endif

  //--------------------------------------------

#ifdef OPAL
  //add OPAL CODE
#endif
}

#endif
