
//------------------------------------------------------------------------------------------------Pitch
// Control
// Initializing-----------------------------------------------------------------------------
#define pitch_direction_pin 9
#define pitch_enable_pin 10
#define limit1 3 // rear limit switch stopped blades
#define limit2 5 // front limit switch full speed

int spd = 500;
bool dir = 0;
bool slowDown = 0;
bool speedUp = 1;
//------------------------------------------------------------------------------------------------Wind
// Speed
// Initializing-----------------------------------------------------------------------------
#define analogPinForRV A1 // change to pins you the analog pins are using
//#define analogPinForTMP  A0
float RV_Wind_ADunits;

//------------------------------------------------------------------------------------------------Current
// Meter
// Initializing-----------------------------------------------------------------------------
#include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();
float current;
bool powerMode = 0;

//------------------------------------------------------------------------------------------------Tachometer
// Initializing-----------------------------------------------------------------------------
// digital pin 2 is the hall pin
int hall_pin = 2;
// set number of hall trips for RPM reading (higher improves accuracy)
float hall_thresh = 20;
float rpm_val;

float RPMcorrection = 0.88; // correction factor added for rpm and voltage

void setup() {
  //------------------------------------------------------------------------------------------------Pitch
  // Control
  // Setup-----------------------------------------------------------------------------
  // set the speed at 15 rpm (likely max):
  pinMode(pitch_direction_pin, OUTPUT);
  pinMode(pitch_enable_pin, OUTPUT);

  //------------------------------------------------------------------------------------------------Wind
  // Speed
  // Setup-----------------------------------------------------------------------------
  Serial.begin(9600); // can increase to 115200 if needed. not sure how other
                      // devices would react
  // faster printing to get a bit better throughput on extended info

  pinMode(analogPinForRV, INPUT);
  // pinMode(analogPinForTMP,INPUT);//currently not using

  //------------------------------------------------------------------------------------------------Current
  // Meter
  // Setup-----------------------------------------------------------------------------
  // Serial.begin(9600);
  // Wait until serial port is opened
  /*
  while (!Serial) { delay(10); }

  Serial.println("Adafruit INA260 Test");

  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");

*/
  //------------------------------------------------------------------------------------------------Tachometer
  // Setup-----------------------------------------------------------------------------
  pinMode(hall_pin, INPUT);
}
void loop() { maintainRPM(1200); }
float readWindSpeed2() {
  float temp = 0;
  for (int i = 0; i < 100; i++) {
    temp = temp + analogRead(analogPinForRV);
  }
  RV_Wind_ADunits = temp / 100;
  return RV_Wind_ADunits;
}
void printWindSpeed() {
  Serial.print(" RV volts 10-bit ");
  Serial.println(RV_Wind_ADunits);
}

void readCurrent() {
  current = -ina260.readCurrent();

  if (current > 50 && powerMode == 0) // determine that we now have a load
  {
    Serial.println("Load Detected");
    powerMode = 1;
  }

  if (powerMode == 1 && current < 50) // if we lose the load brake hard
  {
    Serial.println("Load Lost");
    brakeHard();
    powerMode = 0;
  }
}

void printCurrent() {
  Serial.print("Current: ");
  Serial.print(current);
  Serial.println(" mA");
}

void readRPM() {
  // preallocate values for tach
  float hall_count = 1.0;
  float start = micros();
  bool on_state = false;
  float timeout =
      5000000; // 5 seconds. rpm is less than 120 -----------change this to make
               // it based on hall_thresh. or sacrifice low end accuracy with
               // just plain timeout. should still work

  // counting number of times the hall sensor is tripped
  // but without double counting during the same trip
  while (true) {
    if (digitalRead(hall_pin) == 0) {
      if (on_state == false) {
        on_state = true;
        hall_count += 1.0;
      }
    } else {
      on_state = false;
    }

    if (hall_count >= hall_thresh) {
      break;
    }
    if (micros() - start > timeout) {
      break;
    }
  }

  // print information about Time and RPM
  float end_time = micros();
  float time_passed = ((end_time - start) / 1000000.0);
  // Serial.print("Time Passed: ");
  // Serial.print(time_passed);
  // Serial.println("s");
  rpm_val = (hall_count / time_passed) * 60.0;
  rpm_val = rpm_val * RPMcorrection;
  Serial.print(rpm_val);
  Serial.println(" RPM");
  delay(1); // delay in between reads for stability
}

void maintainRPM(int desiredRPM) // minimum rpm is 150?
{
  /*
  need to find appropriate:
  delay time between adjustments
  pitch correction value (adjust by x amount each iteration if needed)
  no correction needed threshold


  */
  // stepper parameters
  int large_correction_val = 400;   // milliseconds of time to be adjusting
  int large_threshold_margin = 100; //+/- rpm threshold
  int small_correction_val = 100;   // milliseconds of time to adjust for
  int small_threshold_margin = 40;  //+/- rpm threshold
  int coastThreshold = 30;          // rpm
  int minimumSpeed = 150;           // rpm
  int delayTime = 2000;
  int bigSpeedUp = 0;
  int currentRPM = 0;
  int previousRPM = 0;

  while (1) {

    readRPM();
    readCurrent();
    printCurrent(); // only want to print current once per cycle, but need to
                    // read current often
    // readWindSpeed();
    // readAnemometer();
    Serial.print("load selection level: ");
    Serial.println(loadSelection());
    if (rpm_val > 1800) // check if overspeeding
    {
      brakeHard();
    }

    if (currentRPM < minimumSpeed - 20) {
      stallControl();
    }

    if (RV_Wind_ADunits > 600 &&
        currentRPM > 800) // if wind speed is greater than 8 m/s then shorten
                          // delay time and shrink correction amounts
    {
      delayTime = 1000;
      large_correction_val = 200;
      small_correction_val = 50;

    } else // if wind speed isnt above 8 m/s keep original values
    {
      delayTime = 2000;
      large_correction_val = 400;
      small_correction_val = 100;
    }

    if (currentRPM > desiredRPM + small_threshold_margin) // if too fast
    {
      if (currentRPM <
          previousRPM -
              coastThreshold) // check if already slowing down--------maybe
                              // adjust coast threshold
      {
        // do nothing
        Serial.println("Too fast, but already slowing down");
        bigSpeedUp = 0;

      } else // not slowing down
      {
        if (currentRPM >
            desiredRPM + large_threshold_margin) // large difference
        {
          pitchChange(slowDown, large_correction_val);
          bigSpeedUp = 0;
          CurrentCheckDelay(delayTime);
        } else if (currentRPM >
                   desiredRPM + small_threshold_margin) // small difference
        {
          pitchChange(slowDown, small_correction_val);
          bigSpeedUp = 0;
        }
      }
    } else if (currentRPM < desiredRPM - small_threshold_margin) // if too slow
    {
      if (currentRPM >
          previousRPM + coastThreshold) // check if already speeding up--------
      {
        // do nothing
        Serial.println("Too slow, but already speeding up");
        bigSpeedUp = 0;
        // delay(1000);
      } else // not slowing down
      {
        if (currentRPM <
            desiredRPM -
                large_threshold_margin) // too slow need large difference
        {
          if (bigSpeedUp <=
              2) // if we need to make a large speed increase too mny times, we
                 // are likely going to stall, so back off
          {
            pitchChange(speedUp, large_correction_val);
            bigSpeedUp =
                bigSpeedUp +
                1; // need to know if we alread made a large speed up adjustment
                   // because maybe we are too pitched and are stalling.
            CurrentCheckDelay(delayTime);
          } else {
            pitchChange(slowDown, large_correction_val * 3);

            Serial.println("Stall Prevention");
            bigSpeedUp = 0;
            CurrentCheckDelay(
                delayTime); // delay to allow to speed back up hopefully
          }

        } else if (currentRPM <
                   desiredRPM -
                       small_threshold_margin) // too slow need small difference
        {
          pitchChange(speedUp, small_correction_val);

          bigSpeedUp = 0;
        }
      }
    } else {
      Serial.println("No adjustment Made");
      bigSpeedUp = 0;
    }
    previousRPM = currentRPM;
    Serial.println("");
    CurrentCheckDelay(delayTime);
  }
}

void stallControl() // in case of emergency, probably will only work if plugged
                    // in. RPM will be very low
{
  Serial.println("Restarting Due to Stall");
  pitchChange(slowDown, 4000); // move all the way back

  delay(100);
  Serial.println("Pitching forward");
  pitchChange(speedUp, 2000);
  delay(3000);
  readRPM();
}

void startUp() // not using yet
{
  readRPM();
  // while still speeding up do nothing, if at max RPM jump up to next stage
}

void brakeHard() {
  Serial.println("Brake Hard");
  pitchChange(slowDown, 2000); // move all the way back
}

void pitchChange(bool dir, unsigned long duration) {
  int durationThreshold = 250; // only affects what to print make sure its
                               // between large and small duration values
  unsigned long currentTime = micros() / 1000;

  // what to print
  if (duration > durationThreshold) {
    Serial.print("Big pitch adjustment");
  } else {
    Serial.print("Small pitch adjustment");
  }
  if (dir == 1) // change if pitching the wrong way
  {
    Serial.println(" faster");
  } else {
    Serial.println(" slower");
  }

  // carry out pitch change
  while (micros() / 1000 - currentTime <=
         duration) // while we havent reached the duration time
  {
    digitalWrite(pitch_direction_pin, dir); // set direction
    analogWrite(pitch_enable_pin, spd);     // set speed (with pwm)

    if (digitalRead(limit1) == 1 ||
        digitalRead(limit2) == 1) // check if either limit switch was hit
    {
      break;
    }
  }
  stopPitching();
}

void stopPitching() { digitalWrite(pitch_enable_pin, 0); }

void CurrentCheckDelay(int delayTime) {
  unsigned long startTime = micros() / 1000;
  while (micros() / 1000 < startTime + delayTime) {
    readCurrent();
  }
}

int windSpeedLevel() {
  // can we confirm the wind speed is changing?
  // if so determine the level
  // thresholds:

  // max values
  int threshold5 = 586;
  int threshold6 = 593;
  int threshold7 = 599;
  int threshold8 = 605;
  int threshold9 = 610;
  int threshold10 = 614;
  int threshold11 = 620;

  int numChecks = 5;
  float array1[numChecks];
  for (int i = 0; i < numChecks;
       i++) // collect 5 different wind speed samples already averaged
  {
    array1[i] = readWindSpeed2();
  }

  for (int i = 0; i < numChecks - 1; i++) // sort them from least to greatest
  {
    // Last i elements are already in place
    for (int j = 0; j < numChecks - i - 1; j++) {
      if (array1[j] > array1[j + 1]) {
        float temp = array1[j];
        array1[j] = array1[j + 1];
        array1[j + 1] = temp;
        // swap(array1[j], array1[j + 1]);
      }
    }
  }
  /*for(int i = 0; i<numChecks; i++)//prints them
    {
      Serial.print(array1[i]);
      Serial.print(" ");
    }
   Serial.println(" ");*/

  // Serial.println(array1[2]);
  if (array1[2] < threshold5)
    return 5;
  else if (array1[2] < threshold6 && array1[2] > threshold5)
    return 6;
  else if (array1[2] < threshold7 && array1[2] > threshold6)
    return 7;
  else if (array1[2] < threshold8 && array1[2] > threshold7)
    return 8;
  else if (array1[2] < threshold9 && array1[2] > threshold8)
    return 9;
  else if (array1[2] < threshold10 && array1[2] > threshold9)
    return 10;
  else if (array1[2] < threshold11 && array1[2] > threshold10)
    return 11;
  else
    return 12;
}

int loadSelection() // could change this to a void and just send a message
                    // instead of returning the load value
// would need to take RPM into account though first
{
  int numChecks = 5;
  int array2[numChecks];
  int total = 0;

  for (int i = 0; i < 5; i++) // get 5 wind speed levels (must be integers)
  {
    array2[i] = windSpeedLevel();
  }
  for (int i = 0; i < 5;
       i++) // check the first element and see if its the same as the rest
  {
    if (array2[0] == array2[i]) {
      total++;
    } else {
      total = 0;
    }
  }
  if (total >= 3) // if 3 of the 5 are the same then return that value
  {
    return array2[0];
  } else // otherwse return 0 if we cant determine the
  {
    return 0;
  }
}
