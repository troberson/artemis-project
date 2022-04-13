// prototype2.ino
// Artemis Project Wind Turbine Prototype
// Steven Fordham, Tamara Roberson
// Copyright (c) 2022

// need to knows during testing:
// does increasing pitch value increase or decrease rpm
// minimum RPM turbine can operate at/stay alive
// minimum RPM limited voltage is at with no load
// minimum RPM limited voltage is at under load
//
// max power and pitch at 5m/s
// max power and pitch at 6m/s
// max power and pitch at 7m/s
// max power and pitch at 8m/s
// max power and pitch at 9m/s
// max power and pitch at 10m/s
// max power and pitch at 11m/s
//
// shutdown RPM must be at most 10% of the maximum trubine RPM. 11m/s bin is
// weighted at .1 so maybe increase this rpm in case min rpm is too high

// Can increase baud rate to 115200 if needed but not sure how other
// devices would react
// faster printing to get a bit better throughput on extended info
// remember to change your serial monitor
const int SERIAL_PIN_RX = 0;
const int SERIAL_PIN_TX = 1;
const int SERIAL_BAUD = 9600;

// Stepper Motor
#include <Stepper.h>

const int STEPPER_STEPS_PER_REVOLUTION = 2048;
Stepper myStepper(STEPPER_STEPS_PER_REVOLUTION, 8, 10, 9, 11);

// Wind Speed Initialization
// Calibration: To calibrate your sensor, put a glass over it, but the sensor
// should not be touching the desktop surface however. Adjust the
// WIND_SPEED_ADJUSTMENT until your sensor reads about zero with the glass over
// it.

const int WIND_SPEED_PIN_TMP = A0;
const int WIND_SPEED_PIN_RV = A1;

// negative numbers yield smaller wind speeds and vice versa.
const float WIND_SPEED_ADJUSTMENT = 0.3;
const double WIND_SPEED_VOLTAGE_FACTOR = 0.0048828125;

float g_wind_speed_therm_ad = 0; // temp termistor value from wind sensor
float g_wind_speed_rv_ad = 0;    // RV output from wind sensor
float g_wind_speed_rv_volts = 0; // RV voltage from wind sensor
float g_wind_speed_temp_c = 0;   // temperature in celsius x 100
float g_wind_speed_ad = 0;       // wind speed in ???
float g_wind_speed_volts = 0;    // voltage from wind sensor
float g_wind_speed_mph = 0;      // wind speed in miles per hour
unsigned long g_wind_speed_last_millis = 0; // last time wind speed was read

// Current Meter Initialization
#include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();

// Tachometer Initialization
const int TACH_HALL_PIN = 2;

// set number of hall trips for RPM reading (higher improves accuracy)
const float TACH_HALL_THRESH = 10;

// correction factors added for rpm and voltage
const float TACH_CORRECTION_RPM = 0.96;
const float TACH_CORRECTION_VOLTAGE = 0.94;

float g_tach_rpm = 0;   // current RPM
float g_tach_volts = 0; // current voltage

// Communication Initialization
#include <SoftwareSerial.h>

const int COMM_PIN_RX = 10;
const int COMM_PIN_TX = 11;
const int COMM_BAUD = 9600;
SoftwareSerial g_comm_serial(COMM_PIN_RX, COMM_PIN_TX);
char g_comm_input_char = '\0';

// ********** SETUP **********
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }

  Serial.println("start");

  // Initialize Load Control Communications
  g_comm_serial.begin(COMM_BAUD);
  while (!g_comm_serial) {
    delay(10);
  }

  // Pitch Control
  myStepper.setSpeed(15); // in RPM

  // Wind Speed Sensor
  pinMode(WIND_SPEED_PIN_RV, INPUT);
  pinMode(WIND_SPEED_PIN_TMP, INPUT);

  // Current Meter
  Serial.println("Adafruit INA260 Test");

  if (!ina260.begin()) {
    Serial.println("FATAL: Couldn't find INA260 chip");
    while (1) {
    } // FAIL, HANG!
  }

  Serial.println("Found INA260 chip");

  // Tachometer
  pinMode(TACH_HALL_PIN, INPUT);

  // Load Controller Communication
  g_comm_input_char = g_comm_serial.read();
  switch (g_comm_serial) {
  case 0:
    Serial.println("OK");
    break;
  default:
    Serial.print("FAIL (");
    Serial.print(g_comm_input_char);
    Serial.println(")");
  }
}

// ********** MAIN LOOP **********
void loop() {
  // read every 200 ms - printing slows this down further
  // if (millis() - g_wind_speed_last_millis > 200) {
  //   wind_speed_read();
  // }

  // current_read();
  // tach_read_rpm();
  tach_maintain_rpm(500);
}

// ********** WIND SPEED **********
void wind_speed_read() {
  g_wind_speed_therm_ad = analogRead(WIND_SPEED_PIN_TMP);
  g_wind_speed_rv_ad = analogRead(WIND_SPEED_PIN_RV);
  g_wind_speed_volts = (g_wind_speed_rv_ad * WIND_SPEED_VOLTAGE_FACTOR);

  // these are all derived from regressions from raw data as such they depend
  // on a lot of experimental factors such as accuracy of temp sensors, and
  // voltage at the actual wind sensor, (wire losses) which were unaccouted
  // for.
  g_wind_speed_temp_c = 0.005 * g_wind_speed_therm_ad * g_wind_speed_therm_ad -
                        16.862 * g_wind_speed_therm_ad + 9075.4;

  // 13.0C 553 482.39
  g_wind_speed_ad = -0.006 * g_wind_speed_therm_ad * g_wind_speed_therm_ad +
                    1.0727 * g_wind_speed_therm_ad + 47.172;

  g_wind_speed_volts = g_wind_speed_ad * 0.0048828125 - WIND_SPEED_ADJUSTMENT;

  // This from a regression from data in the form of
  // Vraw = V0 + b * WindSpeed ^ c
  // V0 is zero wind at a particular temperature
  // The constants b and c were determined by some Excel wrangling with the
  // solver.
  g_wind_speed_mph =
      pow((abs(g_wind_speed_rv_volts - g_wind_speed_volts) / 0.23), 2.7265);

  Serial.print("  TMP volts ");
  Serial.print(g_wind_speed_therm_ad * 0.0048828125);

  Serial.print("  RV volts ");
  Serial.print(g_wind_speed_rv_volts);

  Serial.print("  TempC*100 ");
  Serial.print(g_wind_speed_temp_c);

  Serial.print("   ZeroWind volts ");
  Serial.print(g_wind_speed_volts);

  Serial.print("   WindSpeed M/S ");
  Serial.println(g_wind_speed_mph * 0.44704); // probs divide by 2.52

  g_wind_speed_last_millis = millis();
}

// ********** CURRENT METER **********
void current_read() {
  Serial.print("Current: ");
  Serial.print(ina260.readCurrent());
  Serial.println(" mA");
}

// ********** TACHOMETER **********
void tach_read_rpm() {
  // preallocate values for tach
  float hall_count = 1.0;
  float start = micros();
  bool on_state = false;

  // counting number of times the hall sensor is tripped
  // but without double counting during the same trip
  // 5 seconds. rpm is less than 120
  // change this to make it based on hall_thresh. or sacrifice low end
  // accuracy with just plain timeout. should still work
  float timeout = 5000000;

  // read hall sensor
  while (true) {
    if (digitalRead(TACH_HALL_PIN) == 0) {
      if (!on_state) {
        on_state = true;
        hall_count += 1.0;
      }
    } else {
      on_state = false;
    }

    if (hall_count >= TACH_HALL_THRESH || (micros() - start) > timeout) {
      break;
    }
  }

  // print information about Time and RPM
  float end_time = micros();
  float time_passed = ((end_time - start) / 1000000.0);

  // Serial.print("Time Passed: ");
  // Serial.print(time_passed);
  // Serial.println("s");

  g_tach_rpm = (hall_count / time_passed) * 60.0 * TACH_CORRECTION_RPM;
  Serial.print(g_tach_rpm);
  Serial.println(" RPM");
  delay(1); // delay in between reads for stability

  g_tach_volts = g_tach_rpm / 24.3 * TACH_CORRECTION_VOLTAGE;

  // Serial.print("Voltage: ");
  // Serial.print(voltage);
  // Serial.println(" V");
}

void tach_maintain_rpm(int rpm_desired) {
  // need to find appropriate:
  // delay time between adjustments
  // pitch correction value (adjust by x amount each iteration if needed)
  // no correction needed threshold

  const int large_correction_val = 200;  // number of steps to adjust
  const int large_threshold_margin = 50; // +/- rpm threshold
  const int small_correction_val = 100;  // number of steps to adjust
  const int small_threshold_margin = 25; // +/- rpm threshold
  const int coast_threshold = 15;        // rpm
  const int minimum_speed = 150;         // rpm
  const int delay_time = 750;

  int big_speed_up = 0;
  int rpm_cur = 0;
  int rpm_prev = 0;

  while (true) {
    tach_read_rpm();
    if (g_tach_rpm < minimum_speed - 20) {
      tach_stall_control();
    }
    rpm_cur = g_tach_rpm;

    // if too fast
    if (rpm_cur > rpm_desired) {
      // check if already slowing down
      // maybe adjust this line
      if (rpm_cur < rpm_prev - coast_threshold) {
        // do nothing
        Serial.println("Too fast, but already slowing down");

        // not slowing down
      } else {

        // large difference
        if (rpm_cur > rpm_desired + large_threshold_margin) {
          myStepper.step(large_correction_val);
          Serial.println("Big pitch adjustment slower");
          big_speed_up = 0;

          // small difference
        } else if (rpm_cur > rpm_desired + small_threshold_margin) {
          myStepper.step(small_correction_val);
          Serial.println("Small pitch adjustment slower");
          big_speed_up = 0;

          // close enough
        } else {
          Serial.println("Close enough");
          big_speed_up = 0;
        }
      }

      // too slow
    } else {
      // check if already speeding up
      // maybe adjust this line
      if (rpm_cur > rpm_prev + coast_threshold) {
        // do nothing
        Serial.println("Too slow, but already speeding up");

        // not speeding up
      } else {
        // too slow need large difference
        if (rpm_cur < rpm_desired - large_threshold_margin) {

          // if we need to make a large speed increase too many times, we
          // are likely going to stall, so back off
          if (big_speed_up <= 2) {
            myStepper.step(-large_correction_val);

            // need to know if we alread made a large speed up adjustment
            // because maybe we are too pitched and are stalling.
            big_speed_up++;
            Serial.println("Big pitch adjustment faster");

            // too slow need small difference
          } else {
            // one more than the big_speed_up threshold just
            // above to get pitch back on track.
            myStepper.step(large_correction_val * 3);
            Serial.println("Stall Prevention");
            big_speed_up = 0;
            delay(1000); // delay to allow to speed back up hopefully
          }

          // too slow need small difference
        } else if (rpm_cur < rpm_desired - small_threshold_margin) {
          myStepper.step(-small_correction_val);
          Serial.println("Small pitch adjustment faster");
          big_speed_up = 0;
        }
      }
    }

    // update previous rpm
    rpm_prev = rpm_cur;
    delay(delay_time);
  }
}

// in case of emergency, probably will only work if plugged in.
// RPM will be very low
void tach_stall_control() {
  myStepper.step(3.5 * 2048); // move all the way back
  delay(100);
  myStepper.step(-2048); // adjust this to start turbine
  delay(3000);
  tach_read_rpm();
  Serial.println("Restart Due to Stall");
}
