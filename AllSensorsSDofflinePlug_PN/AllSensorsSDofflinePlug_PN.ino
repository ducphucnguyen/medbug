#include <Wire.h>
#include <SD.h>

#define VEML6030_ADDR 0x48 // I2C address of VEML6030
#define TMP117_ADDR 0x49 // I2C address of TMP117
const int microphonePin = A1; // Analog input pin for the microphone
const int sensorPin = A0; // piezo sensor input pin

const int numSamplesMicrophone = 10;  // Number of samples for the microphone
const int numSamplesOtherSensors = 5;  // Number of samples for other sensors within the interval
const int numSamplesPressure = 1000; // Number of samples for the pressure sensor
const int numSamples = 1000;
uint16_t ambientLightSamples[numSamplesOtherSensors];
float temperatureSamples[numSamplesOtherSensors];
unsigned long time_stamp[numSamplesOtherSensors];
int microphoneSamples[numSamplesMicrophone];
int pressureData[numSamplesPressure];
int sampleCount = 0;

unsigned long previousMillis = 0;
const unsigned long interval = 10000;  // 10 seconds interval

bool dataLoggingEnabled = false; // Flag to enable data logging

void setup() {
  Wire.begin();
  analogReadResolution(12);  // Set ADC resolution to 12 bits
  Serial.begin(115200);
  delay(1000); // Wait for sensors to stabilize

  // Configure the VEML6030 to operate in high-resolution mode
  Wire.beginTransmission(VEML6030_ADDR);
  Wire.write(0x00); // Command register
  Wire.write(0x10); // Set the high-resolution mode
  Wire.endTransmission();

  // Initialize the SD card
  if (SD.begin()) {
    Serial.println("SD card initialized successfully");
    dataLoggingEnabled = true; // Enable data logging
  } else {
    Serial.println("Failed to initialize SD card!");
  }
}

void loop() {
  if (dataLoggingEnabled) {
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      // Read and store sensor data for multiple samples
      for (int i = 0; i < numSamples; i++) {
        readAmbientLightAndTemperature();
        time_stamp[0] = currentMillis;
        pressureData[i] = analogRead(sensorPin);
        //microphoneSamples[i] = analogRead(microphonePin);
        delay(interval/numSamples);
      
        Serial.println(pressureData[i]);
        //Serial.println(microphoneSamples[i]);
      }

      // Save data to SD card
      saveDataToSDCard(); // Save data to the SD card with the generated filename
    }
  }
}


void readAmbientLightAndTemperature() {
    ambientLightSamples[0] = readAmbientLight();
    temperatureSamples[0] = readTemperature();
    temperatureSamples[0] = readTemperature();
}

void readMicrophoneData() {
  for (int i = 0; i < numSamplesMicrophone; i++) {
    microphoneSamples[i] = readMicrophone();
    delay(100);
  }
}

void readPressureData() {
  for (int i = 0; i < numSamplesPressure; i++) {
    pressureData[i] = convertToPressure(analogRead(sensorPin));
    delay(1);
  }
}




uint16_t readAmbientLight() {
  Wire.beginTransmission(VEML6030_ADDR);
  Wire.write(0x04); // Register address of ambient light data (low byte)
  Wire.endTransmission(false); // Send restart command
  Wire.requestFrom(VEML6030_ADDR, 2); // Request 2 bytes of data

  // Read ambient light data
  byte lsb = Wire.read();
  byte msb = Wire.read();
  uint16_t ambientLight = (msb << 8) | lsb;

  return ambientLight;
}

float readTemperature() {
  Wire.beginTransmission(TMP117_ADDR);
  Wire.write(0x00); // Register address of temperature data
  Wire.endTransmission(false); // Send restart command
  Wire.requestFrom(TMP117_ADDR, 2); // Request 2 bytes of data

  // Read temperature data
  byte msb = Wire.read();
  byte lsb = Wire.read();
  int rawData = (msb << 8) | lsb;

  // Convert raw data to temperature in degrees Celsius
  float temperature = rawData * 0.0078125;

  return temperature;
}

int readMicrophone() {
  int sensorValue = analogRead(microphonePin); // Read the sensor value
 
  // Subtract the DC offset of 512
  int adjustedValue = sensorValue - 512;
 
  return adjustedValue;
}

int convertToPressure(int sensorValue) {
  int pressure = sensorValue - 512; // convert sensor value to pressure
  return pressure;
}

void saveDataToSDCard() {
  File dataFile = SD.open("data_PN_020.txt", FILE_WRITE);

  if (dataFile) {
    // Write the current timestamp to the file
   // dataFile.println(millis());

    // Write the batch data to the file
    //for (int i = 0; i < 1; i++) {
   //   dataFile.print(ambientLightSamples[i]);
    //  dataFile.print("\t"); // Separate light and temperature with a tab
    //  dataFile.println(temperatureSamples[i]);
    //  dataFile.print("\t"); // Separate light and temperature with a tab
    //  dataFile.println(time_stamp[i]);
   // }

    //dataFile.println(); // Empty line for separating sensor data

    //for (int i = 0; i < numSamples; i++) {
    //  dataFile.println(microphoneSamples[i]);
    //}
    
    //dataFile.println(); // Empty line for separating sensor data

    for (int i = 0; i < numSamples; i++) {
    dataFile.println(pressureData[i]);
    }

    // dataFile.println(); // Empty line for separating sensor data

    dataFile.close();
    Serial.println("Data saved to SD card successfully");
  } else {
    Serial.println("Error opening data file");
  }
}
