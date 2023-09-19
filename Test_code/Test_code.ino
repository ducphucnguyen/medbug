const int analogInputPin = A0;
int sensorValue = 0;

void setup() {
  analogReadResolution(12);  // Set ADC resolution to 12 bits
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  sensorValue = analogRead(analogInputPin); // Read analog input from A0
  Serial.println(sensorValue); // Send the value to the serial monitor
  delay(100); // Optional delay to slow down the output for better readability
}
