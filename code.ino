/*
ARDUINO SENSOR TESTER CODE
MADE BY: DHRUV CHHABRA 2022164

NOTE:
LCD NAME & ADDRESS SHOULD BE CHANGED ACCORDINGLY.
SWITCH INPUT OR INPUT_PULLUP, SWITCH ON IS HIGH OR LOW.
SENSORS ARE BEING CONNECTED TO DIFFERENT PINS ON THE ARDUINO.
TUNING THE DELAYS AS PER THE ACTUAL HARDWARE, BUZZER SOUNDS, DISPLAY THINGS ETC.

CONNECTIONS:
Common Analog Pin: A0
Common Digital Pin: 8
MPU6050: SCL_A5, SDA_A4
Ultrasonic: T9, E10

*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
Adafruit_MPU6050 mpu;

// Define the pins for the buttons
#define SELECT_BUTTON 3
#define NEXT_BUTTON 2
#define PREVIOUS_BUTTON 4

// Define the number of sensors and their names
#define NUM_SENSORS 11
const char* sensorNames[NUM_SENSORS] = {
  "Soil Moisture",
  "Ultrasonic",
  "PIR",
  "IR",
  "ACS712",
  "DHT11",
  "GasMQ135",
  "BluetoothHC05",
  "CurrentACS712",
  "ForceFSR402",
  "GyroMPU6050"
};

LiquidCrystal_I2C lcd(0x27, 16, 2);

int selectedSensor = 0;

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

/*This part of the code contains functions to control the BUZZER*/

// defines for the frequency of the notes (.5 x freq of mid C)
#define AN    220     // 440 Hz
#define AS    233     // 466 Hz
#define BN    247     // 493 Hz
#define CN    261     // 523 Hz
#define CS    277     // 554 Hz
#define DN    294     // 588 Hz
#define DS    311     // 622 Hz
#define EN    330     // 658 Hz
#define FN    349     // 698 Hz
#define FS    370     // 740 Hz
#define GN    392     // 784 Hz
#define GS    415     // 830 Hz
// defines for the duration of the notes (in ms)
#define wh    1024
#define h      512
#define dq     448
#define q      256
#define qt     170
#define de     192
#define e      128
#define et      85
#define oo7      1    // 007 jingle


#define PIEZO_PIN 11

void ToneOut(int pitch, int duration){  // pitch in Hz, duration in ms
  int delayPeriod;
  long cycles, i;

  pinMode(PIEZO_PIN, OUTPUT);           // turn on output pin
  delayPeriod = (500000 / pitch) - 7;   // calc 1/2 period in us -7 for overhead
  cycles = ((long)pitch * (long)duration) / 1000; // calc. number of cycles for loop

  for (i=0; i<= cycles; i++){           // play note for duration ms 
    digitalWrite(PIEZO_PIN, HIGH); 
    delayMicroseconds(delayPeriod);
    digitalWrite(PIEZO_PIN, LOW); 
    delayMicroseconds(delayPeriod - 1); // - 1 to make up for digitaWrite overhead
  }
  pinMode(PIEZO_PIN, INPUT);            // shut off pin to avoid noise from other operations
}

void play_tune(int tune){               // play a tune . . .
  switch (tune) {                       // a case for each tune 
  case oo7:                             // 007  E, F#, G (mult by 2 for next higher octave)
    ToneOut(EN*2,qt);                    
    ToneOut(FS*2,qt);
    ToneOut(GN*2,qt);
    delay(h);
    break;
  }
}

void beep(){
    tone(11, 500 ,100);
}

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

/*This section of the code contains the test codes for a specific sensor*/

void testGasMQ135(){
    pinMode(A0, INPUT);
    lcd.clear();

    for (int i = 0; i<200; i++){
        int value = analogRead(A1);
        char buffer[50];
        sprintf(buffer, "AQI = %d PPM", value);

        lcd.print(buffer);
    }
}

void testCurrentACS712(){
    pinMode(A0, INPUT);
    //lcd.begin();
    //lcd.clear();

    for (int i = 0; i<100 ; i++){
        int value = analogRead(A1);
        float voltage = value*5/1023.0;
        float current = (voltage-2.5)/0.185;
        if (current < 0.16){
            current = 0;
        }
        lcd.print("Current: ");
        lcd.print(current);
        delay(300);
    }
}

// void testBluetoothHC05(){

// }


void testGyroMPU6050(){
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  mpu.begin();
 
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  for (int i = 0; i<100; i++){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
  
    /* Print out the values */
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");
  
    Serial.print("Rotation X: ");
    lcd.setCursor(0, 0);
    lcd.print("R:");
    lcd.print(g.gyro.x);
    Serial.print(g.gyro.x);
  
    Serial.print(", Y: ");
    lcd.setCursor(0, 1);
    lcd.print("P:");
    lcd.print(g.gyro.y);
    Serial.print(g.gyro.y);
  
    Serial.print(", Z: ");
    lcd.setCursor(8, 0);
    lcd.print("Y:");
    lcd.print(g.gyro.z);
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");
  
    Serial.print("Temperature: ");
    lcd.setCursor(8, 1);
    lcd.print("T:");
    lcd.print(temp.temperature);
    Serial.print(temp.temperature);
    Serial.println(" degC");
  
    Serial.println("");
    delay(100);
  }
}

void testForceFSR402(){
    const int forceSensorPin = A0; // Analog pin to which the force sensor is connected

    for (int i; i<100; i++){
        int forceValue = analogRead(forceSensorPin); // Read analog value from the force sensor
        float forceVoltage = forceValue * (5.0 / 1023.0); // Convert the analog value to voltage (assuming 5V Arduino)

        // Replace the following calibration values with your specific sensor calibration data
        float forceMinVoltage = 0.1; // Minimum voltage when no force is applied
        float forceMaxVoltage = 4.9; // Maximum voltage when the maximum force is applied
        float forceMinValue = 0.0;   // Minimum force in Newton (N)
        float forceMaxValue = 100.0; // Maximum force in Newton (N)

        // Calculate force in Newton (N) using the calibration data
        float force = ((forceVoltage - forceMinVoltage) / (forceMaxVoltage - forceMinVoltage)) * (forceMaxValue - forceMinValue) + forceMinValue;

        lcd.print("Raw ADC value: ");
        lcd.print(forceValue);
        delay(400);
        lcd.clear();
        lcd.print("\tVoltage: ");
        lcd.print(forceVoltage, 3);
        delay(400);
        lcd.clear();
        lcd.print(" V\tForce: ");
        lcd.print(force, 2);
        //lcd.println(" N");
        delay(400);
        lcd.clear();

        delay(500); // Add a small delay to prevent rapid serial output
        }
}

void testUltrasonic(){
    const int trigPin = 9;
    const int echoPin = 10;
    long duration;
    int distanceCm;

    lcd.begin(16,2); // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    for (int i=0; i<200; i++){
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH);
        distanceCm= duration*0.034/2;
        lcd.setCursor(0,0); // Sets the location at which subsequent text written to the LCD will be displayed
        lcd.print("Distance: "); // Prints string "Distance" on the LCD
        lcd.print(distanceCm); // Prints the distance value from the sensor
        lcd.print(" cm");
        delay(10);
    }
}

void testIR(){
    #define pinIR 8

    pinMode(pinIR, INPUT);

    for (int i=0; i<200; i++){
        int irValue = digitalRead(pinIR);
        lcd.clear();
        
        if (irValue == HIGH){
            lcd.print("No Obstacle");
            delay(200);
        }
        else{
            lcd.print("Obstacle detected");
            delay(200);
        }
    }
    delay(500);
}


void testPIR(){

  lcd.begin(16,2);

  pinMode(8,INPUT_PULLUP);
    lcd.setCursor(0, 0);
    lcd.print("PIR Sensor Test");

  for (int i = 0; i<200; i++){
    int pirValue = digitalRead(8);
    lcd.println(pirValue);

    if (pirValue == LOW){
        lcd.setCursor(0, 1);
        lcd.print("No Motion");
        delay(20);
    }
    else{
        lcd.setCursor(0,1);
        lcd.print("Motion Detected");
        delay(20);
    }
    delay(50);
  }
}


void testSoilMoisture(){
    pinMode(A0, INPUT);
    lcd.begin(16, 2);
    int Soilsensor = 0;

    for (int i= 0; i<50; i++){
        Soilsensor = analogRead(A0);
        lcd.setCursor(0, 0);
        lcd.print("Soil Moisture");
        lcd.setCursor(0, 1);
        lcd.print("Value = ");
        lcd.setCursor(8, 1);
        lcd.print(Soilsensor);
        delay(10); // Delay a little bit to improve simulation performance
    }
}

void testDHT11(){
  #define DHTPIN 8    // what pin we're connected to

  #define DHTTYPE DHT11   // DHT 11

  DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
  //Variables
  //int chk;
  int hum;  //Stores humidity value
  int t; //Stores temperature value
  Serial.begin(9600);
  Serial.println("Temperature and Humidity Sensor Test");
  dht.begin();
  lcd.init(); //initialize the lcd
  lcd.backlight(); //open the backlight

  for(int i=0; i<200; i++){
    //Read data and store it to variables h (humidity) and t (temperature)
      // Reading temperature or humidity takes about 250 milliseconds!
      hum = dht.readHumidity();
      t = dht.readTemperature();

      //Print temp and humidity values to serial monitor
      Serial.print("Humidity: ");
      Serial.print(h);
      Serial.print(" %, Temp: ");
      Serial.print(t);
      Serial.println(" ° Celsius");
  // set the cursor to (0,0):
  // print from 0 to 9:
      lcd.setCursor(0, 0);
      lcd.println(" Now Temperature ");
      lcd.setCursor(0, 1);
      lcd.print("T:");
      lcd.print(t);
      lcd.print("C");
      lcd.setCursor(6, 1);
      lcd.println("2020 ");
      lcd.setCursor(11, 1);
      lcd.print("H:");
      lcd.print(h);
      lcd.print("%");
    delay(1000); //Delay 1 sec.
  }
}

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

/*This part of code controls the LCD Display*/

void updateSensorDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Select Sensor:");
  lcd.setCursor(0, 1);
  lcd.print(sensorNames[selectedSensor]);
}

void testSelectedSensor() {

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Testing:");
    delay(500);
    lcd.setCursor(0, 1);
    lcd.print(sensorNames[selectedSensor]);
    delay(1000);

    if (sensorNames[selectedSensor] == "Ultrasonic"){
        testUltrasonic();
    }
    else if (sensorNames[selectedSensor] == "DHT"){
        testDHT11();
    }
    else if (sensorNames[selectedSensor] == "IR"){
        testIR();
    }
    else if (sensorNames[selectedSensor] == "ACS712"){
        testCurrentACS712();
    }
    else if (sensorNames[selectedSensor] == "PIR"){
        testPIR();
    }
    else if (sensorNames[selectedSensor] == "Soil Moisture"){
        testSoilMoisture();
    }
    else if (sensorNames[selectedSensor] == "MQ135"){
        testGasMQ135();
    }
    // else if (sensorNames[selectedSensor] == "BluetoothHC05"){
    //     testBluetoothHC05();
    // }
    else if (sensorNames[selectedSensor] == "GyroMPU6050"){
        testGyroMPU6050();
    }
    else if (sensorNames[selectedSensor] == "ForceFSR402"){
      testForceFSR402();
    }
}

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

/*Main Code begins here*/

void setup() {
  // Initialize the LCD display
    lcd.init();	
  lcd.setCursor(0,0);
  lcd.backlight();
  lcd.display();

  // Set up the button pins
  pinMode(SELECT_BUTTON, INPUT_PULLUP);
  pinMode(NEXT_BUTTON, INPUT_PULLUP);
  pinMode(PREVIOUS_BUTTON, INPUT_PULLUP);

  // Display welcome message
  lcd.setCursor(0, 0);
  lcd.print("WELCOME!");
  play_tune(oo7);
  
  delay(2000);
	updateSensorDisplay();
}

void loop() {
  	// Check if the select button is pressed
  	if (digitalRead(SELECT_BUTTON) == LOW) {
      beep();
        // Assign the selected sensor pin
      testSelectedSensor();
      lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Hurray!");
          delay(100);
          play_tune(oo7);
          lcd.setCursor(0,1);
      lcd.print("Test Completed");
      delay(1000);
      loop();
      //while(true);
      }

	// Check if the next button is pressed
	else if (digitalRead(NEXT_BUTTON) == LOW) {
    beep();
		selectedSensor = (selectedSensor + 1) % NUM_SENSORS;
		updateSensorDisplay();
		delay(500); // Add a small delay for button debouncing
		loop();
	}

	// Check if the previous button is pressed
	else if (digitalRead(PREVIOUS_BUTTON) == LOW) {
    beep();
		selectedSensor = (selectedSensor - 1 + NUM_SENSORS) % NUM_SENSORS;
		updateSensorDisplay();
		delay(500); // Add a small delay for button debouncing
		loop();
	}
}