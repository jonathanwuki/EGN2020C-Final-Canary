/*
 * "Canary" final project source code
 * 
 * Author: Jonathan Wukitsch
 *         EGN2020C - Engineering Design & Society
 *         Fall 2022
 *         University of Florida
 *         
 * Some code in this file was borrowed from the Adafruit SGP30 documentation as well as the Arduino LiquidCrystal example sketch.
 * SGP30 docs: https://learn.adafruit.com/adafruit-sgp30-gas-tvoc-eco2-mox-sensor/arduino-code
 * LiquidCrystal docs: https://docs.arduino.cc/learn/electronics/lcd-displays
 *         
 * Last Edited: December 3, 2022
 */

#include <LiquidCrystal.h>
#include <Wire.h>
#include <dht.h>
#include "Adafruit_SGP30.h"

#define PIEZO_PIN     9 // Piezo speaker pin - Digital 9
#define LED_PIN      13 // LED pin - Digital 13
#define TEMP_HUM_PIN A0 // Temperature/humidity sensor pin - Analog A0

// Defines for the frequency of the piezo notes (.5 x freq of mid C)
#define TONE_SOUND_1 330 // 658 Hz
#define TONE_SOUND_2 370 // 740 Hz
#define TONE_SOUND_3 392 // 784 Hz

// Defines for the duration of the piezo notes (in ms)
#define TONE_DURATION  170
#define TONE_WAIT_TIME 512

// Defines for the air quality baselines/max thresholds
#define ECO2_BASELINE      400
#define TVOC_BASELINE      0
#define ECO2_MAX_THRESHOLD 1500
#define TVOC_MAX_THRESHOLD 108

// Miscellaneous defines
#define FW_VERSION         1.55
#define CALIBRATION_CYCLES 30

const bool IS_DEBUG_ENABLED = true;
bool isAboveThreshold = false;
bool calibrated = false;
int counter = 0;

// Initialize SGP30 sensor
Adafruit_SGP30 sgp;

// Initialize temp/humidity sensor
dht DHT;

// Initialize LCD library
const int rs = 12,
          en = 11,
          d4 = 5,
          d5 = 4,
          d6 = 3,
          d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/* Return absolute humidity [mg/m^3] with approximation formula
 * @param temperature [°C]
 * @param humidity [%RH]
 */
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // Approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  // (Provided in SGP30 documentation)
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
  return absoluteHumidityScaled;
}

/*
 * Play a short startup tone via the connected Piezo speaker
 */
void playStartupTone() {
  // Each tone sound is a different frequency, we chose what sounded best as a startup tone
  ToneOut(TONE_SOUND_1 * 2, TONE_DURATION);
  ToneOut(TONE_SOUND_2 * 2, TONE_DURATION);
  ToneOut(TONE_SOUND_3 * 2, TONE_DURATION);
  delay(TONE_WAIT_TIME);
}

/*
 * Auxiliary for playStartupTone function
 * Play a specific Piezo tone (in Hz) for a duration
 * @param pitch [Hz]
 * @param duration [ms]
 */
void ToneOut(int pitch, int duration) {
  int delayPeriod;
  long cycles, i;

  pinMode(PIEZO_PIN, OUTPUT); // Turn on output pin
  delayPeriod = (500000 / pitch) - 7; // 500000 is an arbitrary number we found to work best with the tones we wanted to play
  cycles = ((long)pitch * (long)duration) / 1000; // Calculate number of cycles depending on pitch and duration, divide by 1000ms

  for(i = 1; i <= cycles; i++) { // Play noise for specific duration
    digitalWrite(PIEZO_PIN, HIGH); // Setting the Piezo to HIGH plays a tone
    delayMicroseconds(delayPeriod); // Wait for the calculated delay period (again, somewhat arbitrary)
    digitalWrite(PIEZO_PIN, LOW); // Stop playing the tone
    delayMicroseconds(delayPeriod - 1); // Delay for (period - 1) to make up for digitalWrite overhead
  }

  pinMode(PIEZO_PIN, INPUT); // Shut off pin to avoid noise from other operations
}

void setup() {
  Serial.begin(115200);
  while(!Serial) { delay(10); } // Wait for serial console to open

  // LCD setup
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  lcd.setCursor(0, 1);
  lcd.print("FW Version: " + (String)FW_VERSION);
  delay(1000);

  // Sensor not attached
  if(!sgp.begin()) {
    Serial.println("Error 2: Sensor not found");
    lcd.setCursor(0, 0);
    lcd.print("Error 2");
    lcd.setCursor(0, 1);
    lcd.print("Sensor Failure");
    while(1);
  }

  if(IS_DEBUG_ENABLED) {
    // Display SGP30 serial info if debug mode is enabled
    Serial.println("SGP30 initialized!");
    Serial.print("Found SGP30 serial #");
    Serial.print(sgp.serialnumber[0], HEX);
    Serial.print(sgp.serialnumber[1], HEX);
    Serial.println(sgp.serialnumber[2], HEX);
  }

  // Set up output pins
  pinMode(PIEZO_PIN, OUTPUT); // Piezo
  pinMode(LED_PIN, OUTPUT); // LED

  playStartupTone(); // Finish startup by playing the tone
}

void loop() {
  counter++; // Increment loop counter (this is used during the calibration routine to make sure we have accurate baseline readings)
  DHT.read11(TEMP_HUM_PIN); // Read temperature and humidity

  // Set the absolute humidity to enable the humidity compensation for the air quality signals
  sgp.setHumidity(getAbsoluteHumidity(DHT.temperature, DHT.humidity));

  if(!sgp.IAQmeasure()) {
    Serial.println("Error 3: Measurement failed");
    lcd.setCursor(0, 0);
    lcd.print("Error 3");
    lcd.setCursor(0, 1);
    lcd.print("Sensor Failure");
    return;
  }

  if(sgp.TVOC > TVOC_MAX_THRESHOLD) { // TVOC is too high/beyond threshold
    digitalWrite(LED_PIN, HIGH); // Turn LED on

    if(isAboveThreshold == true) {
      ToneOut(523, 1000); // Play a tone on the Piezo speaker to alert the user that TVOC is too high
      isAboveThreshold = false;
    }
  } else { // TVOC is no longer too high/beyond threshold
    isAboveThreshold = false;
    digitalWrite(LED_PIN, LOW); // Turn LED off
  }

  if(sgp.TVOC == TVOC_BASELINE && sgp.eCO2 == ECO2_BASELINE && calibrated == false) {
    // Calibration cycles have not yet been completed, display message accordingly
    lcd.print("Calibrating...");
    lcd.setCursor(0, 1);
    lcd.print("Please wait");
  } else { // Calibration has already happened, display raw values
    lcd.print("TVOC: "); lcd.print(sgp.TVOC); lcd.print(" ppb"); if(sgp.TVOC > 500) { lcd.print("!!!"); }
    lcd.setCursor(0, 1);
    lcd.print((float)DHT.humidity, 0); lcd.print("%  "); lcd.print((float)DHT.temperature * 1.8 + 32, 0); lcd.print("F");
  }

  if(!sgp.IAQmeasureRaw()) { // An error occurred while trying to measure raw air quality values
    Serial.println("Error 1: Raw measurement failed");
    lcd.setCursor(0, 0);
    lcd.print("Error 1");
    lcd.setCursor(0, 1);
    lcd.print("Sensor Failure");
    return;
  }

  if(IS_DEBUG_ENABLED) {
    // Display raw data in console if debug mode is enabled
    Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb \t");
    Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.println(" ppm");
    Serial.print("Raw H2 "); Serial.print(sgp.rawH2); Serial.print(" \t");
    Serial.print("Raw Ethanol "); Serial.print(sgp.rawEthanol); Serial.println("");

    Serial.print("Current humidity = "); Serial.print(DHT.humidity); Serial.print("%  ");
    Serial.print("Temperature = "); Serial.print(DHT.temperature); Serial.println("C  ");
  }

  delay(1000); // Delay for 1000ms before updating/looping again
  lcd.clear(); // Clear the LCD in preparation to start over from the beginning of the loop

  if(counter == CALIBRATION_CYCLES) { // Check if we've reached out set number of calibration cycles
    counter = 0; // Reset the counter
    calibrated = true;

    if(sgp.TVOC > TVOC_MAX_THRESHOLD) { // Check again here if we are above the TVOC threshold, as this could've been the case while calibrating
      isAboveThreshold = true;
    } else {
      isAboveThreshold = false;
    }

    uint16_t TVOC_base, eCO2_base;
    if(!sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) { // Get TVOC and eCO2 readings (eCO2 not used on the display but is still available via the SGP30 sensor)
      // TVOC/eCO2 readings unavailable, display error message accordingly
      Serial.println("Error 4: Failed to get baseline readings");
      lcd.setCursor(0, 0);
      lcd.print("Error 4");
      lcd.setCursor(0, 1);
      lcd.print("Sensor Failure");
      return;
    }

    if(IS_DEBUG_ENABLED) {
      // Print baseline eCO2 and TVOC values post-calibration if debug mode is enabled
      Serial.print("****Baseline values: eCO2: 0x"); Serial.print(eCO2_base, HEX);
      Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
    }
  }
}
