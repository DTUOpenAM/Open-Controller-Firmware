#include <SD.h>
#include <FastPID.h>

//------------------------------------------------------------------------------
// VARIABLES
//------------------------------------------------------------------------------

#define SENSORCOUNT (3)
//#define windowSize (5)
int count_SensorDatapoints;
unsigned long SENSORTIME = 0;
unsigned long SENSORTIME2 = 0;
unsigned long phototimer = micros();
unsigned long phototimer2 = micros();
String sensorString = "";
int phcount;
double avg;
String dataString = "";
String dataStringtmp = "";
int dataTimer = millis();
//volatile bool laser_is_on;
volatile bool storingData;
unsigned long time_now_micros = micros();

//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------

volatile int flowSens;
volatile int flowInput;
volatile double flowInput_metersecond;
volatile double VFDoutput;
volatile int Setpoint = 0;
volatile unsigned long currentTime;
volatile unsigned long previousTime;
volatile double elapsedTime;
uint8_t PIDoutput;
double humanSetpoint;

#ifndef OPAL
AuxDev flow(FAN, -1 , 255, false); //initialize the analog control of the vfd resposible for crossflow
#endif

// SENSOR STRUCT
typedef struct {
  volatile unsigned long sensorMillis = 0; //Bookkeeping variable
  volatile unsigned int Sensor_Delay = 1500;
  volatile double pressSens;
  volatile int oxySens;
  volatile int flowSens;
  volatile double pressInput;
  volatile double oxyInput;
  volatile double flowInput;
  volatile double pressOutput;
  volatile double oxyOutput;
  volatile double flowOutput;
  //  volatile int windowSize = 5;
  int index = 0;
  volatile double sum = 0;
  volatile int readings[5];
  volatile double oxyOutput_avg = 0;
  volatile double phdSens;

} PROCESS_SENSOR;

PROCESS_SENSOR SENS;

//int indexSens = 0;

float Kp = 0.002, Ki = 0.05, Kd = 0.001, Hz = 5;


int output_bits = 10;
bool output_signed = false;

FastPID PIDlib(Kp, Ki, Kd, Hz, output_bits, output_signed);
//bool setOutputConfig(output_bits, output_signed);

//------------------------------------------------------------------------------
// VOIDS
//------------------------------------------------------------------------------


void startSDcard() {
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void storeDataForSDCard(String sensorType, String sensorInput) {
  storingData = true;
  SENSORTIME2 = micros();
  dataString.concat(sensorType);
  dataString.concat(" ");
  dataString.concat(SENSORTIME2);
  dataString.concat(" ");
  dataString.concat(sensorInput);
  dataString.concat(" ; ");
  storingData = false;
}

//void writeDatatoSDcard() {
//  Serial.println(aux.is_the_laser_on());
//  if (!laser_is_on && (micros() - dataTimer) >= 10000000 && storingData == false) {
//
//    dataStringtmp = dataString;
//    dataString = "";
//
//    File dataFile = SD.open("datalog.txt", FILE_WRITE);
//    if (dataFile) {
//      dataFile.println(dataStringtmp);
//      dataFile.close();
//      // print to the serial port too:
//      Serial.println("storing data");
//    }
//    // if the file isn't open, pop up an error:
//    else {
//      Serial.println("error opening datalog.txt");
//    }
//    dataTimer = micros();
//  }
//}

void storeSensorData(String sensorType, String sensorInput) {
  SENSORTIME = micros();

  count_SensorDatapoints++;
  if (sensorString.length() == 0) {
    sensorString = "O600 ";
    sensorString.concat(String(SENSORCOUNT));
    sensorString.concat(" ");
  }

  sensorString.concat(sensorType);
  sensorString.concat(" ");
  sensorString.concat(SENSORTIME);
  sensorString.concat(" ");
  sensorString.concat(sensorInput);
  sensorString.concat(" ");

  if (count_SensorDatapoints > MAX_DATAPOINTS_STORED) {
    // if MAX_DATAPOINTS_STORED data points has been stored remove them to prevent memory overflow
    //todo fix this more elegantly. 
    sensorString = "";
    count_SensorDatapoints = 0;  }
}

void sendSensorData() {
  Serial.println(sensorString);
  sensorString = "";
  count_SensorDatapoints = 0;
}

#ifdef LOOP
// CONTROL GAS INPUT
void GasIn(int inState) {
  if (inState == 1) { // The valve is set to open. The valve is normally-closed. It will close in case of loss of power
    digitalWrite(GASIN, HIGH);
    Serial.println("Gas input valve is open!");
  }
  if (inState == 2) {
    digitalWrite(GASIN, LOW);
    Serial.println("Gas input valve is closed!");
  }
}

// CONTROL GAS OUTPUT
void GasOut(int outState) {

  if (outState == 1) { // The valve is set to open. The valve is normally-closed. It will close in case of loss of power
    digitalWrite(GASOUT, HIGH);
    Serial.println("Gas output valve is open!");
  }
  if (outState == 2) {
    digitalWrite(GASOUT, LOW);
    Serial.println("Gas output valve is closed!");
  }
}

// ALL GAS CONTROL

void AllGas(int allState) {
  if (allState == 1) {
    digitalWrite(GASIN, HIGH);
    digitalWrite(GASOUT, HIGH);
    Serial.println("All gas valves are open!");
  }
  if (allState == 2) {
    digitalWrite(GASIN, LOW);
    digitalWrite(GASOUT, LOW);
    Serial.println("All gas valves are closed!");
  }
}

#endif

void setPointFlow(double stp) {
  if (stp >= 0 && stp <= 15) { // is the setpoint positive?

    Setpoint = map(stp, 0, 15, 0, 1023);
    humanSetpoint = stp;
  }
  else if (stp > 15) {    // Is the setpoint too big?
    Serial.println("The sensor cannot measure faster than 15m/s. Input ignored.");

  }
  else if (stp < 0) {
    Serial.println("We don't blow in that direction. Choose positive Setpoint in m/s. Input ignored.");
  }
}


void xflowPID() { //timed update xflow
  static unsigned long flowMillis = 0; //Bookkeeping variable
  static unsigned int flowUpdateDelay = 200;

  if (millis() - flowMillis  >= flowUpdateDelay ) { //time to update the pid controller

    flowSens = analogRead(SENS3);                //read from flow sensor
    flowInput = map(flowSens, 0, 890, 0, 1023);
    //flowInput_metersecond = flowInput/100;

    //Serial.print("Sense input [m/s] : ");
    //double humanflowInput = map(flowSens, 0, 890, 0, 1500);

    //Serial.println(humanflowInput/100);

    //Serial.print("Raw sensor Input [0-1023] : ");
    //Serial.println(flowSens);

    PIDoutput = PIDlib.step(Setpoint, flowInput);


    //Serial.print("Flow PID output: ");

    //Serial.println(PIDoutput);
#ifndef OPAL
    flow.setPower(PIDoutput);
#endif
    //Serial.print("Setpoint: ");
    //Serial.print(humanSetpoint);
    //Serial.print(" m/s");

    flowMillis = millis();
  }
}


void photosensor() {

  //  Serial.print("phtosensor: ");
  //  time_now_micros = micros() - time_now_micros ;
  //  Serial.println(time_now_micros);
  //  Serial.println("");
  //  time_now_micros = micros();

  //if (micros() - phototimer2 >= 20 ){


  //}
  if (micros() - phototimer >= 180) {
    phototimer2 = 0;
    SENS.phdSens = analogRead(SENS3);
    SENS.phdSens += SENS.phdSens;
    phcount++;

    //Serial.println("are we doing this?");
    phototimer = micros();
    avg = SENS.phdSens;// / phcount;
    phcount = 0;
    storeDataForSDCard("PhD", String(avg));
  }
  //return avg;
}

void sensors() {     // The place where we keep the process sensors.

  //static unsigned long sensorMillis = 0; //Bookkeeping variable
  //static unsigned int Sensor_Delay = 1500;

  if (millis() - SENS.sensorMillis  >= SENS.Sensor_Delay ) { //Time to read flow sensor


    //NOTE TO SELF; make sensor and independet class that can be instatiated with different type sensors TODO

    SENS.pressSens = analogRead(SENS1);              //read from pressure
    SENS.oxySens =  analogRead(SENS2);                // read oxygen sensor
    SENS.flowSens = analogRead(SENS3); //read flow sensor

    SENS.pressInput = map(SENS.pressSens, 0, 940, 0, 200);  // Convert to correct scale
    SENS.oxyInput = map(SENS.oxySens, 204, 885, 0, 2500);     // Convert to correct scale
    SENS.flowInput = map(SENS.flowSens, 0, 1023, 0, 100);

    SENS.flowOutput = SENS.flowInput / 100;
    storeSensorData("Flow", String(SENS.flowOutput));


    SENS.pressOutput = SENS.pressInput / 100; // Convert to bar pressure
    storeSensorData("Pres", String(SENS.pressOutput));

    SENS.oxyOutput = SENS.oxyInput / 100;      // Convert to percentage oxygen

    SENS.sum = SENS.sum - SENS.readings[SENS.index];
    SENS.readings[SENS.index] = SENS.oxyOutput;
    SENS.sum = SENS.sum + SENS.oxyOutput;
    SENS.index = (SENS.index + 1) % 5;

    SENS.oxyOutput_avg = SENS.sum / 5;

    storeSensorData("Oxyg", String(SENS.oxyOutput));

    SENS.sensorMillis = millis();
  }

}
