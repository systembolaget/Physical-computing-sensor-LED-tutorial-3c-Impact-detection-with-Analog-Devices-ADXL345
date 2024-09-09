// Tutorial 3c. Impact detection with Analog Devices ADXL345

// Main parts: Adafruit HUZZAH 8266, white, yellow and red
// standard LEDs, three resistors, Analog Devices ADXL345

// Libraries required to interface with the sensor via I2C
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Assign an ID to the sensor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Variables that remain constant
const float offset = 9.81; // Offset accelerometer values to baseline 0
const float alpha_EMA = 0.3; // Smaller = smoother = more delay, depends on your application

const float threshold0 = 10; // Observe the serial plotter and select suitable thresholds for your design
const float threshold1 = 30;
const float threshold2 = 50;
const unsigned long timeWait = 150; // Depends what kinds of impacts you want to detect

const int pinsLED[] = {12, 13, 14}; // White, yellow, red
const unsigned long timeLEDOn = 200; // A brief flash

enum {START, WAIT, STOP} state; // The three cases (enum = ordered list of integer constants) for the switch/case logic state variable

// Variables that can change
float mAV_EMA = 9.81; // The variable must be initialised with a start-up value

int previousThreshold = -1; // Start with an impossible value, because 0 is the first and lowest

unsigned long timerStart;

void setup()
{
  // Start the serial plotter
  Serial.begin(115200);

  // // Initialise the LED pins specified above
  for (int i = 0; i < 3; i++)
  {
    pinMode(pinsLED[i], OUTPUT);
  }

  // Initialise the sensor
  accel.begin();

  // Set the G sensitivity and sampling rate to what suits your design
  accel.setRange(ADXL345_RANGE_16_G); // Valid values are 2, 4, 8, 16
  accel.setDataRate(ADXL345_DATARATE_400_HZ); // For valid values see the specification sheet
}

void loop()
{
  // Timestamp that updates every loop() iteration
  unsigned long timeNow = millis();

  mAV_EMA = getAcceleration();
  // Display the filtered value in the serial plotter
  Serial.print("min:"); Serial.print(0); Serial.print(", ");
  Serial.print("max:"); Serial.print(100); Serial.print(", ");
  Serial.print("mAV_EMA:"); Serial.print(mAV_EMA); Serial.print(", ");
  Serial.println();

  // -1, 0, 1, 2 depending on the threshold crossed, with
  // threshold 0 inevitably always crossed first
  int threshold = getThreshold(mAV_EMA);

  switch (state)
  {
    case START:
      if (threshold >= 0) // Has threshold 0, 1, or 2 been crossed?
      {
        previousThreshold = threshold; // If so, remember which one it was
        timerStart = timeNow; // And remember when it was crossed
        state = WAIT; // Go to check if an even higher threshold will be crossed
      }
      break; // Contiue with the code after the switch statement

    case WAIT:
      if (threshold > previousThreshold) // Was a higher threshold crossed?
      {
        previousThreshold = threshold; // Again remember which one it was
        timerStart = timeNow; // And remember when it was crossed
      }

      if (timeNow - timerStart >= timeWait) // If the waiting time of 150 milliseconds is over
      {
        digitalWrite(pinsLED[previousThreshold], HIGH); // Switch on the LED that corresponds to the highest threshold crossed
        timerStart = timeNow; // And remember when it was switched on
        state = STOP; // Go to check if it is time to switch the LED off
      }
      break;

    case STOP:
      if (timeNow - timerStart >= timeLEDOn) // If the 300 milliseconds long flash has passed
      {
        digitalWrite(pinsLED[previousThreshold], LOW); // Switch the LED off
        state = START; // And ready the system for the impact detection
      }
      break;
  }

  delay(10); // Only for serial monitor/plotter
}

float getAcceleration()
{
  float aX; // Raw x-axis acceleration value
  float aY;
  float aZ;

  float mAV; // Magnitude of the acceleration vector ("total acceleration")

  // Fetch a sensor event
  sensors_event_t event;
  accel.getEvent(&event);

  // And read the x, y and z axis acceleration values
  aX = event.acceleration.x;
  aY = event.acceleration.y;
  aZ = event.acceleration.z;

  // Then calculate the magnitude of the acceleration vector
  mAV = sqrt(aX * aX + aY * aY + aZ * aZ) - offset;

  // Push insignificant negative values to 0
  if (mAV < 0)
  {
    mAV = 0;
  }

  // And finally apply an exponential moving average ("EMA")
  // filter to smooth the data
  mAV_EMA = (alpha_EMA * mAV) + ((1 - alpha_EMA) * mAV_EMA);

  return mAV_EMA;
}

int getThreshold(float mA_VEMA)
{
  // Set a threshold value depending on the magnitude of the
  // acceleration vector
  if (mAV_EMA >= threshold2) return 2;
  if (mAV_EMA >= threshold1) return 1;
  if (mAV_EMA >= threshold0) return 0;

  return -1;
}
