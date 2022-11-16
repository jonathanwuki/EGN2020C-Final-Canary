#define PIEZO_PIN 9
#define LED_PIN 13
#define TEMP_HUM_PIN A0

// defines for the frequency of the notes (.5 x freq of mid C)
#define TONE_SOUND_1    330     // 658 Hz
#define TONE_SOUND_2    370     // 740 Hz
#define TONE_SOUND_3    392     // 784 Hz

// defines for the duration of the notes (in ms)
#define TONE_DURATION     170
#define TONE_WAIT_TIME    512

#include <LiquidCrystal.h>
#include <Wire.h>
#include <dht.h>
#include "Adafruit_SGP30.h"

bool isAboveThreshold = false;
bool calibrated = false;

Adafruit_SGP30 sgp;
dht DHT;

// Initialize the library by associating any needed LCD interface pin
// with the Arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/* return absolute humidity [mg/m^3] with approximation formula
* @param temperature [°C]
* @param humidity [%RH]
*/
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

void play_tune(int tune) {               // play a tune . . .
  switch (tune) {                       // a case for each tune 
  case 1:                             // 007  E, F#, G (mult by 2 for next higher octave)
    ToneOut(TONE_SOUND_1*2, TONE_DURATION);                    
    ToneOut(TONE_SOUND_2*2, TONE_DURATION);
    ToneOut(TONE_SOUND_3*2, TONE_DURATION);
    delay(TONE_WAIT_TIME);
    break;
  }
}

void ToneOut(int pitch, int duration) {  // pitch in Hz, duration in ms
  int delayPeriod;
  long cycles, i;

  pinMode(PIEZO_PIN, OUTPUT);           // turn on output pin
  delayPeriod = (500000 / pitch) - 7;   // calc 1/2 period in us -7 for overhead
  cycles = ((long)pitch * (long)duration) / 1000; // calc. number of cycles for loop

  for (i = 1; i <= cycles; i++) {           // play note for duration ms 
    digitalWrite(PIEZO_PIN, HIGH); 
    delayMicroseconds(delayPeriod);
    digitalWrite(PIEZO_PIN, LOW); 
    delayMicroseconds(delayPeriod - 1); // - 1 to make up for digitaWrite overhead
  }
  
  pinMode(PIEZO_PIN, INPUT);            // shut off pin to avoid noise from other operations
}

void setup() {
  Serial.begin(115200);
  while(!Serial) { delay(10); } // Wait for serial console to open

  Serial.println("SGP30 Initialized!");

  if(!sgp.begin()) {
    Serial.println("Sensor not found :(");
    while (1);
  }
  
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
  
  // set up the LCD's number of columns and rows
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Initializing...");
  lcd.setCursor(0, 1); // Setting the cursor on LCD
  lcd.print("FW Version: 1.54"); // prints on LCD
  delay(1000);

  play_tune(1);
  pinMode(PIEZO_PIN, OUTPUT); // Piezo
  pinMode(LED_PIN, OUTPUT); // LED

  // If you have a baseline measurement from before you can assign it to start, to 'self-calibrate'
  //sgp.setIAQBaseline(0x8E68, 0x8F41);  // Will vary for each sensor!
}

int counter = 0;
void loop() {

    DHT.read11(TEMP_HUM_PIN);
    
    Serial.print("Current humidity = ");
    Serial.print(DHT.humidity);
    Serial.print("%  ");
    Serial.print("temperature = ");
    Serial.print(DHT.temperature); 
    Serial.println("C  ");

   // If you have a temperature / humidity sensor, you can set the absolute humidity to enable the humditiy compensation for the air quality signals
   sgp.setHumidity(getAbsoluteHumidity(DHT.temperature, DHT.humidity));

  if(!sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }

  if(sgp.TVOC > 108) {
    digitalWrite(13, HIGH); // LED
    if(isAboveThreshold == true) {
      tone(PIEZO_PIN, 523, 1000); // play tone 60 (C5 = 523 Hz)
      isAboveThreshold = false;
    }
  } else {
    isAboveThreshold = false;
    digitalWrite(13, LOW); // LED
  }

  if(sgp.TVOC == 0 && sgp.eCO2 == 400 && calibrated == false) {
    lcd.print("Calibrating...");
    lcd.setCursor(0, 1);
    lcd.print("Please wait");
  } else {
    lcd.print("TVOC: "); lcd.print(sgp.TVOC); lcd.print(" ppb"); if(sgp.TVOC > 500) { lcd.print("!!!"); }
    lcd.setCursor(0, 1); // Setting the cursor on LCD
    lcd.print((float)DHT.humidity, 0); lcd.print("%  "); lcd.print((float)DHT.temperature * 1.8 + 32, 0); lcd.print("F");
    //lcd.print("eCO2 "); lcd.print(sgp.eCO2); lcd.print(" ppm");
  }

  if(!sgp.IAQmeasureRaw()) {
    Serial.println("Raw Measurement failed");
    lcd.print("Error 1");
    lcd.setCursor(0, 1);
    lcd.print("Sensor Failure");
    return;
  }
  
  Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb \t");
  Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.println(" ppm");
  Serial.print("Raw H2 "); Serial.print(sgp.rawH2); Serial.print(" \t");
  Serial.print("Raw Ethanol "); Serial.print(sgp.rawEthanol); Serial.println("");
 
  delay(1000);
  lcd.clear();

  counter++;
  if (counter == 30) {
    counter = 0;
    calibrated = true;

    if(sgp.TVOC > 108) {
      isAboveThreshold = true;
    } else {
      isAboveThreshold = false;
    }

    uint16_t TVOC_base, eCO2_base;
    if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
      Serial.println("Failed to get baseline readings");
      return;
    }
    
    Serial.print("****Baseline values: eCO2: 0x"); Serial.print(eCO2_base, HEX);
    Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
  }
}
