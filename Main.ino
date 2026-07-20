#include <SPI.h>
#include <Kalman.h>
#include <ICM42688.h>
#include <Adafruit_BMP3XX.h>
#include <Servo.h>
#include <Adafruit_SPIFlash.h>
float SeaLevelPressure_HPA = 1023.25;
#define PyroChnl1 22 
#define PyroChnl2 23 
#define PyroChnl3 24 
#define PyroChnl4 25 

#define CanardServo1 6
#define CanardServo2 7
#define FinServo1 2
#define FinServo2 3
#define FinServo3 4
#define FinServo4 5           

struct IMUData {
  float ax, ay, az, gx, gy, gz;
};
struct BaroData {
  float pressure, altitude, temperature;
};
struct EstimatedData {
  float pitch, roll, yaw;
  float altitude, BatteryVoltage;
};
struct FlightLog {
  //baro data
  BaroData.pressure, BaroData.altitude, BaroData.temperature;
  //imu data
  IMUData.ax, IMUData.ay, IMUData.az, IMUData.gx, IMUData.gy, IMUData.gz;
  //estimated data
  EstimatedData.pitch, EstimatedData.roll, EstimatedData.yaw, EstimatedData.altitude, EstimatedData.BatteryVoltage;
  //state data
  currentState, currPyroChnl;
  servoAngle[1], servoAngle[2], servoAngle[3], servoAngle[4], servoAngle[5], servoAngle[6];

}
FlightLog flightLog;
IMUData imuData;
BaroData baroData;
EstimatedData estimatedData;
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

//OBJECTs
Adafruit_BMP3XX BARO;
ICM42688 IMU(SPI, 10);
Servo servoCanard1;
Servo servoCanard2;
Servo servoFin1;
Servo servoFin2;
Servo servoFin3;
Servo servoFin4;

// Global state variables
unsigned long startTime;
unsigned long endTime;
flightState currentState = stateIdle;
unsigned long launchStart = 0;
float altitude, pressure;
int CurrPyroChnl = 0;
unsigned long now = 0;
unsigned long lastIMURead = 0;
unsigned long lastSlowRead = 0;

// Track output state of pyro channels separately (can't digitalRead OUTPUT pins reliably)
bool pyroChnlState[5] = {false, false, false, false, false};
int servoAngle[6];

// Format: $timestamp,state,ax,ay,az,gx,gy,gz,pitch,roll,yaw,pressure,altitude,battery,pyroChnl,servoAngle[1],servoAngle[2],servoAngle[3],servoAngle[4],servoAngle[5],servoAngle[6]\n
void sendToBlackbox(unsigned long timestamp){

}
void firePyro(int num){
  switch(num){
    case 1: digitalWrite(PyroChnl1, HIGH); pyroChnlState[1] = true; break;
    case 2: digitalWrite(PyroChnl2, HIGH); pyroChnlState[2] = true; break;
    case 3: digitalWrite(PyroChnl3, HIGH); pyroChnlState[3] = true; break;
    case 4: digitalWrite(PyroChnl4, HIGH); pyroChnlState[4] = true; break;
    default: break;
  }
}
void WriteServos(int angle, int num){
  angle = constrain(angle, 70, 110);
  servoAngle[num] = angle;
  switch(num){
    case 1: servoCanard1.write(angle);  break;
    case 2: servoCanard2.write(angle);  break;
    case 3: servoFin1.write(angle);  break;
    case 4: servoFin2.write(angle);  break;
    case 5: servoFin3.write(angle);  break;
    case 6: servoFin4.write(angle);  break;
  }
}
void readIMUat5ms(){
   if (now - lastIMURead >= 5) {
    lastIMURead = now;
    readIMU();
   }
}
void slowTasks(){
  if (now - lastSlowRead >= 50) {
    lastSlowRead = now;

    if (BARO.performReading()) {
      pressure = BARO.pressure / 100.0f;
      altitude = BARO.readAltitude(SeaLevelPressure_HPA);
    }

    sendToBlackBox(now);
  }
}
// Read all IMU data into globals using correct ICM42688 API
void readIMU() {
  IMU.getAGT();
  imuData.ax = IMU.accX();
  imuData.ay = IMU.accY();
  imuData.az = IMU.accZ();
  imuData.gx = IMU.gyrX();
  imuData.gy = IMU.gyrY();
  imuData.gz = IMU.gyrZ();
}

// renamed from stateArmed() to handleArmed() to avoid conflict with enum value stateArmed
void handleArmed() {
  now = millis();

  // --- IMU at 200Hz (every 5ms) ---
  readIMUat5ms();
    // Launch detection — ay spike above 25g for at least 50ms
    if (imuData.ay >= 25.0f) {
      if (launchStart == 0)
        launchStart = now;
      if (now - launchStart >= 50) {
        Serial.println("Launch detected!");
        currentState = stateAscent;
      }
    } else {
      launchStart = 0;
    }
  
  slowTasks();
}

// renamed from stateAscent() to handleAscent() to avoid conflict with enum value stateAscent
void handleAscent() {
  now = millis();

  // --- IMU at 200Hz (every 5ms) ---
  readIMUat5ms();
  // --- Baro + BlackBox TX at 50ms (20Hz) ---
  slowTasks();
}

void handleCalibration() {
  now = millis();
  
  // --- IMU at 200Hz (every 5ms) ---
  readIMUat5ms();
  
  // --- Baro + BlackBox TX at 50ms (20Hz) ---
  if (now - lastSlowRead >= 50) {
    lastSlowRead = now;

    if (BARO.performReading()) {
      pressure = BARO.pressure / 100.0f;
      SeaLevelPressure_HPA = pressure;
      altitude = BARO.readAltitude(SeaLevelPressure_HPA);
    }

    sendToBlackBox(now);
  }
}

void setup() {
  currentState = stateIdle;

  Serial.begin(115200);          // Debug / monitor
  Serial1.begin(BLACKBOX_BAUD);  // UART link to BlackBox MCU (TX1 pin 18, RX1 pin 19)

  pinMode(PyroChnl1, OUTPUT);
  pinMode(PyroChnl2, OUTPUT);
  pinMode(PyroChnl3, OUTPUT);
  pinMode(PyroChnl4, OUTPUT);
  digitalWrite(PyroChnl1, LOW);
  digitalWrite(PyroChnl2, LOW);
  digitalWrite(PyroChnl3, LOW);
  digitalWrite(PyroChnl4, LOW);

  // Servo attaches
  servoCanard1.attach(CanardServo1);
  servoCanard2.attach(CanardServo2);
  servoFin1.attach(FinServo1);
  servoFin2.attach(FinServo2);
  servoFin3.attach(FinServo3);
  servoFin4.attach(FinServo4);

  pinMode(27, INPUT); // ARM switch
  pinMode(26, INPUT); // Test switch
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

  // Initial barometer calibration to establish starting ground pressure reference
  delay(100);
  for (int i = 0; i < 10; i++) {
    BARO.performReading();
    delay(10);
  }
  if (BARO.performReading()) {
    SeaLevelPressure_HPA = BARO.pressure / 100.0f;
  }
}

void loop() {
  if (pyroChnlState[1])      CurrPyroChnl = 1;
  else if (pyroChnlState[2]) CurrPyroChnl = 2;
  else if (pyroChnlState[3]) CurrPyroChnl = 3;
  else if (pyroChnlState[4]) CurrPyroChnl = 4;
  else                       CurrPyroChnl = 0;

  bool Test = digitalRead(26);

  if (digitalRead(27) && currentState == stateCalibration) {
    Serial.println("ARMED");
    currentState = stateArmed;
  } 
  if (startTime == 0) {
    startTime = millis();
  }

  switch (currentState) {
    case stateCalibration:
      handleCalibration();
      break;
    case stateArmed:
      handleArmed();
      break;
    case stateAscent:
      handleAscent();
      break;
    case stateTest:
      if (Test == 1 && currentState == stateCalibration) {
        handleTest();
      }
      break;
    default:
      break;
  }
}
