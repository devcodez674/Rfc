#include <SPI.h>
#include <Kalman.h>
#include <ICM42688.h>
#include <Adafruit_BMP3XX.h>
#include <Servo.h>
#include <Adafruit_SPIFlash.h>
#define SeaLevelPressure_HPA 1023.25
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
}
IMUData imuData;
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
//PID

// Track output state of pyro channels separately (can't digitalRead OUTPUT pins reliably)
bool pyroChnlState[5] = {false, false, false, false, false};

// Send a CSV data packet to the BlackBox over Serial1
// Format: $timestamp,state,ax,ay,az,gx,gy,gz,pressure,altitude,battery,pyroChnl\n
void sendToBlackBox(unsigned long timestamp) {
  int BatteryRaw = analogRead(A0);
  Serial1.print('$');
  Serial1.print(timestamp);      Serial1.print(',');
  Serial1.print(currentState);   Serial1.print(',');
  Serial1.print(ax, 4);          Serial1.print(',');
  Serial1.print(ay, 4);          Serial1.print(',');
  Serial1.print(az, 4);          Serial1.print(',');
  Serial1.print(gx, 4);          Serial1.print(',');
  Serial1.print(gy, 4);          Serial1.print(',');
  Serial1.print(gz, 4);          Serial1.print(',');
  Serial1.print(pressure, 2);    Serial1.print(',');
  Serial1.print(altitude, 2);    Serial1.print(',');
  Serial1.print(BatteryRaw);     Serial1.print(',');
  Serial1.println(CurrPyroChnl);
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
  static unsigned long lastIMURead = 0;
  static unsigned long lastSlowRead = 0;

  unsigned long now = millis();

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
  static unsigned long lastIMURead = 0;
  static unsigned long lastSlowRead = 0;

  unsigned long now = millis();

  // --- IMU at 200Hz (every 5ms) ---
  readIMUat5ms();
  // --- Baro + BlackBox TX at 50ms (20Hz) ---
  slowTasks();
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
