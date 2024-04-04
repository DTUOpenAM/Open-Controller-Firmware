#define MAX_CHAR_BUF  (64) // What is the longest message Arduino can store?

#ifdef BAXTER
char PLATFORM[] = "BAXTER";
#define SCANLABS              //DEFINE SCANNER
#define BAUD                  (115200)  // How fast is the Arduino talking?
#define NUM_MIRRORS           (2) //Mirrors Controlled By dac -> galvo
#define INVERT_X_AXIS         (false)
#define INVERT_Y_AXIS         (false)
#define STEPS_PER_MM         (195)//  195 scanlabs scanner as found in thesis
#define STEPS_PER_MM_X         (195)//  
#define STEPS_PER_MM_Y         (195)// 
#define BUILDPLATE_DIAMETER (100)// Size of buildplate in mm
#define FULLSCALEGALVO        (65536-1)
//#define ADJUSTED_CENTER_X (31129) //9.5 * STEPS_PER_DEG
//#define ADJUSTED_CENTER_Y (28835) //8.8 * STEPS_PER_DEG
//#define OFFSET_GALVOA     (1639) //  FULLSCALEGALVO/2 - ADJUSTED_CENTER_X
//#define OFFSET_GALVOB     (3933) // FULLSCALEGALVO/2 - ADJUSTED_CENTER_Y
#define ADJUSTED_CENTER_X (65536/2) //9.5 * STEPS_PER_DEG
#define ADJUSTED_CENTER_Y (65536/2) //8.8 * STEPS_PER_DEG
#define OFFSET_CENTER_X (0) // distance in mm from default center of build plate to actual center 
#define OFFSET_CENTER_Y (0) // distance in mm from default center of build plate to actual center 
#define MAX_PWR           (250)
#define LASER_STARTUP_DELAY (0) // how longin us does the galvo hold still before moving when a new laser commands is send
#define X_MAX             (1000)
#define X_MIN             (0)
#define Y_MAX             (1000)
#define Y_MIN             (0)
#define Z_MAX             (1000)
#define Z_MIN             (0)
#define OUTPUTMODE        (1) // return angular position from scanner
#define NUM_VIBRATORS     (3)
#define NUM_MOTORS        (3)
#define LASER_LUT (false)
#endif

#ifdef LOOP
#define SUNNY //SCABLABS // DEFINE SCANNER
char PLATFORM[] = "LOOP";
#define BAUD                  (115200)  // How fast is the Arduino talking?
#define MAX_BUF               (128)  // What is the longest message Arduino can store?
#define NUM_MIRRORS           (2) //Mirrors Controlled By dac -> galvo
#define INVERT_X_AXIS         (false)
#define INVERT_Y_AXIS         (false)
#define STEPS_PER_MM         (204)//  Calibrate by MBOKJ
#define STEPS_PER_MM_X         (204)//  Calibrate by MBOKJ
#define STEPS_PER_MM_Y         (204)//  Calibrate by MBOKJ
#define BUILDPLATE_DIAMETER (250)// Size of buildplate in mm
#define FULLSCALEGALVO        (65536-1)
#define ADJUSTED_CENTER_X (66300/2) //9.5 * STEPS_PER_DEG //65536  33,150
#define ADJUSTED_CENTER_Y (66200/2) //8.8 * STEPS_PER_DEG //65536  33,100
#define OFFSET_CENTER_X (1.876) // distance in mm from default center of build plate to actual center actual 1.8725
#define OFFSET_CENTER_Y (1.63) // distance in mm from default center of build plate to actual center 
#define OFFSET_GALVOA     (0) //  FULLSCALEGALVO/2 - ADJUSTED_CENTER_A
#define OFFSET_GALVOB     (0) // FULLSCALEGALVO/2 - ADJUSTED_CENTER_B
#define MAX_PWR           (300)
#define LASER_STARTUP_DELAY (0) // how long in us does the galvo hold still before moving when a new laser commands is send
#define TRACKING_DELAY (450) // Galvo laser tracking error delay in us 220 set to 450
#define TRACKING_DELAY_OFFSET (10) // offset for accepting tracking error timer
#define STEP_RESPONSE_DELAY (400) // Galvo step response delay in us set to 430
#define STEP_RESPONSE_DELAY_OFFSET (10)
#define SCANNER_SETTLING_TIME (100) // allow the scanner time to settle before commencing a new laser on movement
#define UP2SPEED_TIME (800) //how much time it takes for the galvo to get up to speed
#define X_MAX             (BUILDPLATE_DIAMETER/2)
#define X_MIN             (-BUILDPLATE_DIAMETER/2)
#define Y_MAX             (BUILDPLATE_DIAMETER/2)
#define Y_MIN             (-BUILDPLATE_DIAMETER/2)
#define NUM_VIBRATORS     (3)
#define NUM_MOTORS        (6)
#define MAX_DATAPOINTS_STORED (100) // how many datapoints from sensors is stored in RAM before they are removed
#define LASER_LUT         (false)
#endif


#ifdef OPAL
char PLATFORM[] = "OPAL";
#define SUNNY                   //define scanner
#define BAUD                  (115200)  // How fast is the Arduino talking?
#define NUM_MIRRORS           (2) //Mirrors Controlled By dac -> galvo
#define INVERT_X_AXIS         (false)
#define INVERT_Y_AXIS         (false)
#define STEPS_PER_MM         (361)//  Calibrated by CLEBU 02/05-22
#define STEPS_PER_MM_X         (361)//  Calibrated by CLEBU 02/05-22
#define STEPS_PER_MM_Y         (361)// Calibrated by CLEBU 02/05-22
#define BUILDPLATE_DIAMETER (200)// Size of buildplate in mm
#define FULLSCALEGALVO        (65536-1)
#define ADJUSTED_CENTER_X (65536/2) //9.5 * STEPS_PER_DEG
#define ADJUSTED_CENTER_Y (28000) //8.8 * STEPS_PER_DEG
//#define OFFSET_GALVOX     (0) //  FULLSCALEGALVO/2 - ADJUSTED_CENTER_A
//#define OFFSET_GALVOY     (0) // FULLSCALEGALVO/2 - ADJUSTED_CENTER_B
#define MAX_PWR           (300)
#define LASER_STARTUP_DELAY (0) // how longin us does the galvo hold still before moving when a new laser commands is send
#define TRACKING_DELAY (220) // Galvo laser tracking error delay in us
#define TRACKING_DELAY_OFFSET (10) // offset for accepting tracking error timer
#define STEP_RESPONSE_DELAY(100) // Galvo step response delay in us
#define STEP_RESPONSE_DELAY_OFFSET (10)
#define SCANNER_SETTLING_TIME (100) // allow the scanner time to settle before commencing a new laser on movement
#define OFFSET_CENTER_X (0) // distance in mm from default center of build plate to actual center 
#define OFFSET_CENTER_Y (-13.21) // distance in mm from default center of build plate to actual center
#define X_MAX             (BUILDPLATE_DIAMETER/2)
#define X_MIN             (-BUILDPLATE_DIAMETER/2)
#define Y_MAX             (BUILDPLATE_DIAMETER/2)
#define Y_MIN             (-BUILDPLATE_DIAMETER/2)
#define Z_MIN             (0)
#define NUM_VIBRATORS     (3)
#define NUM_MOTORS        (7)
#define LASER_LUT (false)
#endif
