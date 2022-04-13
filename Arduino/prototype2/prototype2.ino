/*
need to knows during testing:
does increasing pitch value increase or decrease rpm
minimum RPM turbine can operate at/stay alive
minimum RPM limited voltage is at with no load
minimum RPM limited voltage is at under load

max power and pitch at 5m/s
max power and pitch at 6m/s
max power and pitch at 7m/s
max power and pitch at 8m/s
max power and pitch at 9m/s
max power and pitch at 10m/s
max power and pitch at 11m/s

shutdown RPM must be at most 10% of the maximum trubine RPM. 11m/s bin is weighted at .1 so maybe increase this rpm in case min rpm is too high
*/
//------------------------------------------------------------------------------------------------Pitch Control Initializing-----------------------------------------------------------------------------
#include <Stepper.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

//------------------------------------------------------------------------------------------------Wind Speed Initializing-----------------------------------------------------------------------------
#define analogPinForRV    A1   // change to pins you the analog pins are using
#define analogPinForTMP   A0

// to calibrate your sensor, put a glass over it, but the sensor should not be
// touching the desktop surface however.
// adjust the zeroWindAdjustment until your sensor reads about zero with the glass over it. 

const float zeroWindAdjustment =  .3; // negative numbers yield smaller wind speeds and vice versa.

int TMP_Therm_ADunits;  //temp termistor value from wind sensor
float RV_Wind_ADunits;    //RV output from wind sensor 
float RV_Wind_Volts;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;

//------------------------------------------------------------------------------------------------Current Meter Initializing-----------------------------------------------------------------------------
#include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();

//------------------------------------------------------------------------------------------------Tachometer Initializing-----------------------------------------------------------------------------
// digital pin 2 is the hall pin
int hall_pin = 2;
// set number of hall trips for RPM reading (higher improves accuracy)
float hall_thresh = 10;
float rpm_val;

//---------------------------

float voltage;//may not need voltage or voltage correction
float RPMcorrection = .96;//correction factor added for rpm and voltage
float vCorrection = .94;

void setup() {
//------------------------------------------------------------------------------------------------Pitch Control Setup-----------------------------------------------------------------------------
// set the speed at 15 rpm (likely max):
  myStepper.setSpeed(15);
  
//------------------------------------------------------------------------------------------------Wind Speed Setup-----------------------------------------------------------------------------
  Serial.begin(9600);   //can increase to 115200 if needed. not sure how other devices would react
  // faster printing to get a bit better throughput on extended info
  // remember to change your serial monitor

  Serial.println("start");
  // put your setup code here, to run once:

  //   Uncomment the three lines below to reset the analog pins A2 & A3
  //   This is code from the Modern Device temp sensor (not required)
  pinMode(A2, INPUT);        // GND pin      
  pinMode(A3, INPUT);        // VCC pin
  //digitalWrite(A3, LOW);     // turn off pullups

  pinMode(analogPinForRV, INPUT);
  pinMode(analogPinForTMP,INPUT);

//------------------------------------------------------------------------------------------------Current Meter Setup-----------------------------------------------------------------------------
 //Serial.begin(9600);
  // Wait until serial port is opened
  while (!Serial) { delay(10); }

  Serial.println("Adafruit INA260 Test");

  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");


//------------------------------------------------------------------------------------------------Tachometer Setup-----------------------------------------------------------------------------
 pinMode(hall_pin, INPUT);

}
void loop() {
 // readWindSpeed();
 // readCurrent();
 // readRPM();
 maintainRPM(500);

}

void readWindSpeed()
{
   if (millis() - lastMillis > 200){      // read every 200 ms - printing slows this down further
    
    TMP_Therm_ADunits = analogRead(analogPinForTMP);
    RV_Wind_ADunits = analogRead(analogPinForRV);
    RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);

    // these are all derived from regressions from raw data as such they depend on a lot of experimental factors
    // such as accuracy of temp sensors, and voltage at the actual wind sensor, (wire losses) which were unaccouted for.
    TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  

    zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39

    zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  

    // This from a regression from data in the form of 
    // Vraw = V0 + b * WindSpeed ^ c
    // V0 is zero wind at a particular temperature
    // The constants b and c were determined by some Excel wrangling with the solver.
    
    WindSpeed_MPH =  pow((abs(RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265);   
   
    Serial.print("  TMP volts ");
    Serial.print(TMP_Therm_ADunits * 0.0048828125); 
    
    Serial.print(" RV volts ");
    Serial.print(RV_Wind_Volts);

    Serial.print("\t  TempC*100 ");
    Serial.print(TempCtimes100 );

    Serial.print("   ZeroWind volts ");
    Serial.print(zeroWind_volts);

    Serial.print("   WindSpeed M/S ");
    Serial.println(WindSpeed_MPH*0.44704);// probs divide by 2.52 
    lastMillis = millis();    
  } 
}

void readCurrent()
{
  Serial.print("Current: ");
  Serial.print(ina260.readCurrent());
  Serial.println(" mA");
}

void readRPM()
{
   // preallocate values for tach
  float hall_count = 1.0;
  float start = micros();
  bool on_state = false;
  float timeout = 5000000;//5 seconds. rpm is less than 120 -----------change this to make it based on hall_thresh. or sacrifice low end accuracy with just plain timeout. should still work
  
  // counting number of times the hall sensor is tripped
  // but without double counting during the same trip
  while(true)
  {
    if (digitalRead(hall_pin)==0)
    {
      if (on_state==false)
      {
        on_state = true;
        hall_count+=1.0;
      }
    }
    else
    {
      on_state = false;
    }
    
    if (hall_count>=hall_thresh)
    {
      break;
    }
    if(micros()-start>timeout)
    {
      break;
    }
  }
  
  // print information about Time and RPM
  float end_time = micros();
  float time_passed = ((end_time-start)/1000000.0);
  //Serial.print("Time Passed: ");
  //Serial.print(time_passed);
  //Serial.println("s");
  rpm_val = (hall_count/time_passed)*60.0;
  rpm_val = rpm_val*RPMcorrection;
  Serial.print(rpm_val);
  Serial.println(" RPM");
  delay(1);        // delay in between reads for stability


//------
voltage = rpm_val/24.3*vCorrection;
/*
Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");*/
}

void maintainRPM(int desiredRPM)//minimum rpm is 150?
{
/*
need to find appropriate:
delay time between adjustments
pitch correction value (adjust by x amount each iteration if needed)
no correction needed threshold


*/  
int large_correction_val = 200;//number of steps to adjust
int large_threshold_margin = 50;//+/- rpm threshold
int small_correction_val = 100;//number of steps to adjust
int small_threshold_margin = 25;//+/- rpm threshold
int coastThreshold = 15;//rpm
int minimumSpeed = 150;//rpm
int delayTime = 750;
int bigSpeedUp = 0;


int currentRPM=0;
int previousRPM=0;


while(1)
{
  
  readRPM();
  if(rpm_val<minimumSpeed-20)
  {
    stallControl();
  }
  currentRPM = rpm_val;
  
  if(currentRPM>desiredRPM)//if too fast
  {
    if(currentRPM<previousRPM-coastThreshold)//check if already slowing down--------maybe adjust this line
    {
      //do nothing
      Serial.println("Too fast, but already slowing down");
    }
    else//not slowing down
    {
      if(currentRPM>desiredRPM+large_threshold_margin)//large difference
      {
         myStepper.step(large_correction_val);
       Serial.println("Big pitch adjustment slower");
       bigSpeedUp = 0;
      }
      else if(currentRPM>desiredRPM+small_threshold_margin)//small difference
      {
        myStepper.step(small_correction_val);
       Serial.println("Small pitch adjustment slower");
       bigSpeedUp = 0;
      }
    }
  }
  else
  {
    if(currentRPM>previousRPM+coastThreshold)//check if already speeding up--------maybe adjust this line
    {
      //do nothing
      Serial.println("Too slow, but already speeding up");
    }
    else//not slowing down
    {
      if(currentRPM<desiredRPM-large_threshold_margin)//too slow need large difference
      {
        if(bigSpeedUp <=2)//if we need to make a large speed increase too mny times, we are likely going to stall, so back off
        {
         myStepper.step(-large_correction_val);
         bigSpeedUp = bigSpeedUp+1;// need to know if we alread made a large speed up adjustment because maybe we are too pitched and are stalling.
         Serial.println("Big pitch adjustment faster");
        }
        else
        {
           myStepper.step(large_correction_val*3);//one more than the bigSpeedUp threshold just above to get pitch back on track.
           Serial.println("Stall Prevention");
           bigSpeedUp = 0;
           delay(1000);//delay to allow to speed back up hopefully
           
           
        }
         
       
      }
      else if(currentRPM<desiredRPM-small_threshold_margin)//too slow need small difference
      {
        myStepper.step(-small_correction_val);
       Serial.println("Small pitch adjustment faster");
       bigSpeedUp = 0;
      }
    }
  }
  previousRPM = currentRPM;
  delay(delayTime);
}
}

void stallControl()//in case of emergency, probably will only work if plugged in. RPM will be very low
{
  myStepper.step(3.5*2048);//move all the way back
   delay(100);
    myStepper.step(-2048);//adjut this to start turbine
    delay(3000);
    readRPM();
    Serial.println("Restart Due to Stall");
}
