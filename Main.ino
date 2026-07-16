#include <SD.h>
#include <SPI.h>
#include <Kalman.h>
#include <ICM42688.h>
#include <Adafruit_BMP3XX> 
#include <Servo.h>

#define SeaLevelPressure_HPA 1023.25
#define PyroChnl1 22 // main motor
#define PyroChnl2 23 // drogue chute
#define PyroChnl3 24 // pararchute
#define PyroChnl4 25 //

#define CanardServo1 6
#define CanardServo2 7
#define FinServo1 2
#define FinServo2 3
#define FinServo3 4
#define FinServo4 5

enum flightState {
  stateIdle,
  stateCalibration,
  stateArmed,
  stateAscent,
  stateCoast,
  stateApogee,
  stateDescent,
  stateLanded,
  stateTest
  
};

Adafruit_BMP3XX BARO;
ICM42688 IMU(SPI, 10, 8000000);

void stateArmed() {
    IMU.readAccel(ax, ay, az);
    IMU.readGyro(gx, gy, gz);

    if (BARO.performReading()) {
        pressure = BARO.pressure / 100.0f;
        altitude = BARO.readAltitude(SeaLevelPressure_HPA);
    }
    if (ay >= 25.0f) {
        if (launchStart == 0)
            launchStart = millis();

        if (millis() - launchStart >= 50) {
            Serial.println("Launch detected!");
            currentState = stateAscent;
        }
    } else {
        launchStart = 0;
    }
}
void stateAscent() {
  
}
void CheckSensors(){
    //testing
  Serial.println("-----Testing-----");
  IMU.readAccel(ax, ay, az);
  IMU.readGyro(gx, gy, gz);

  Serial.print("Accel (g): ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.print(az);

  Serial.print(" | Gyro (deg/s): ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.println(gz);
  Serial.println("IMU TESTING COMPLETE GOING TO BARO");
  if (!BARO.performReading()) {
    Serial.println("Failed to read BMP388");
    return;
  }
  float temp = BARO.temperature;
  float pressure = BARO.pressure / 100.0; // Convert to hPa
  float altitude = BARO.readAltitude(SeaLevelPressure_HPA);
Serial.print(" Temperature: "); Serial.print(temp);
  Serial.print("°C | Pressure: "); Serial.print(pressure);
  Serial.print(" hPa | Altitude: "); Serial.print(altitude);
  Serial.println(" m");
  int BatteryRaw = analogRead(A0);
}
void CheckPyro(){
  
  Serial.println("STARTING PYROTECHNIC TESTS");
  delay(3000);
  digitalWrite(PyroChnl1, HIGH); Serial.println("PYRO CHANNEL 1 IS HIGH MAKE SURE IGNITOR IS NOT CONNECTED");
  delay(3000);
  digitalWrite(PyroChnl1, LOW); Serial.println("PYRO CHANNEL 1 TEST IS OVER STARTING CHANNEL 2 ");
  delay(3000);
  digitalWrite(PyroChnl2, HIGH); Serial.println("PYRO CHANNEL 2 IS HIGH MAKE SURE IGNITOR IS NOT CONNECTED");
  delay(3000);
  digitalWrite(PyroChnl2, LOW); Serial.println("PYRO CHANNEL 2 TEST IS OVER STARTING CHANNEL 3 ");
  delay(3000);
  digitalWrite(PyroChnl3, HIGH); Serial.println("PYRO CHANNEL 3 IS HIGH MAKE SURE IGNITOR IS NOT CONNECTED");
  delay(3000);
  digitalWrite(PyroChnl3, LOW); Serial.println("PYRO CHANNEL 3 TEST IS OVER STARTING CHANNEL 4 ");
  delay(3000);
  digitalWrite(PyroChnl4, HIGH); Serial.println("PYRO CHANNEL 4 IS HIGH MAKE SURE IGNITOR IS NOT CONNECTED");
  delay(3000);
  digitalWrite(PyroChnl4, LOW); Serial.println("PYRO CHANNEL 4 TEST IS OVER  ");
  Serial.println("PyroTechnic tests are completed");

}

unsigned long startTime;
unsigned long endTime;
flightState currentState = stateIdle;
float ax, ay, az, gx, gy, gz;
unsigned long launchStart = 0;
float altitude, pressure;
void setup(){
  currentState = stateIdle;
  
  
  Serial.begin(115200);
  pinMode(PyroChnl1, OUTPUT);
  pinMode(PyroChnl2, OUTPUT);
  pinMode(PyroChnl3, OUTPUT);
  pinMode(PyroChnl4, OUTPUT);
  digitalWrite(PyroChnl1, LOW);
  digitalWrite(PyroChnl2, LOW);
  digitalWrite(PyroChnl3, LOW);
  digitalWrite(PyroChnl4, LOW);
  
  pinMode(27, INPUT); //ARM switch
  pinMode(26, INPUT); //Test switch
    currentState = stateCalibration;
  
  
  while (!IMU.begin()) {
    Serial.println("IMU communication has not been established");
    delay(1000);
  }
  
  while (!BARO.begin_SPI(9)) {
    Serial.println("Barometer communication has not been established");
    delay(1000);
  }
  BARO.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  BARO.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  BARO.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  while (!SD.begin(8)){
    Serial.println("SD card is not initialized");
    delay(1000);
  }

}
void loop(){
  bool Test = digitalRead(26);
    if (digitalRead(27) && currentState == stateCalibration) {
      Serial.println("ARMED");
      currentState = stateArmed;
    }
  else if (Test == 1 && currentState == stateCalibration) {
    Serial.println("CURRENT STATE IS TESTING I AM BEGINING THE TESTING PROCEDURE");
    currentState = stateTest;
    CheckPyro();
    delay(3000);
    CheckSensors();
    delay(3000);
    Serial.println("TEST IS FINSHED PLESE TURN OFF THE TEST SWITCH AND TO TEST AGAIN PLEASE TURN IT BACK ON CURRENT SOFTWARE TEST BUTTON STATUS IS OFF");
    delay(5000);
   }
      if (startTime == 0) {
    startTime = millis();
      
      }
     
     switch (currentState) {
       case stateArmed:
         stateArmed();
         break;
       case stateAscent:
         stateAscent();
         break;
     }
     
}
  

