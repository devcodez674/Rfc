void CheckSensors() {
  Serial.println("-----Testing-----");
  readIMU();

  Serial.print("Accel (g): ");
  Serial.print(imuData.ax); Serial.print(", ");
  Serial.print(imuData.ay); Serial.print(", ");
  Serial.print(imuData.az);

  Serial.print(" | Gyro (deg/s): ");
  Serial.print(imuData.gx); Serial.print(", ");
  Serial.print(imuData.gy); Serial.print(", ");
  Serial.println(imuData.gz);
  Serial.println("IMU TESTING COMPLETE GOING TO BARO");

  if (!BARO.performReading()) {
    Serial.println("Failed to read BMP388");
    return;
  }

  float temp = BARO.temperature;
  pressure = BARO.pressure / 100.0f;
  altitude = BARO.readAltitude(SeaLevelPressure_HPA);

  Serial.print(" Temperature: "); Serial.print(temp);
  Serial.print("C | Pressure: "); Serial.print(pressure);
  Serial.print(" hPa | Altitude: "); Serial.print(altitude);
  Serial.println(" m");

  int BatteryRaw = analogRead(A0);
  Serial.print("Battery Raw ADC: "); Serial.println(BatteryRaw);
}
void CheckPyro() {
  Serial.println("STARTING PYROTECHNIC TESTS");
  delay(3000);
  digitalWrite(PyroChnl1, HIGH); Serial.println("PYRO CHANNEL 1 IS HIGH MAKE SURE IGNITOR IS NOT CONNECTED");
  delay(3000);
  digitalWrite(PyroChnl1, LOW);  Serial.println("PYRO CHANNEL 1 TEST IS OVER STARTING CHANNEL 2");
  delay(3000);
  digitalWrite(PyroChnl2, HIGH); Serial.println("PYRO CHANNEL 2 IS HIGH MAKE SURE IGNITOR IS NOT CONNECTED");
  delay(3000);
  digitalWrite(PyroChnl2, LOW);  Serial.println("PYRO CHANNEL 2 TEST IS OVER STARTING CHANNEL 3");
  delay(3000);
  digitalWrite(PyroChnl3, HIGH); Serial.println("PYRO CHANNEL 3 IS HIGH MAKE SURE IGNITOR IS NOT CONNECTED");
  delay(3000);
  digitalWrite(PyroChnl3, LOW);  Serial.println("PYRO CHANNEL 3 TEST IS OVER STARTING CHANNEL 4");
  delay(3000);
  digitalWrite(PyroChnl4, HIGH); Serial.println("PYRO CHANNEL 4 IS HIGH MAKE SURE IGNITOR IS NOT CONNECTED");
  delay(3000);
  digitalWrite(PyroChnl4, LOW);  Serial.println("PYRO CHANNEL 4 TEST IS OVER");
  Serial.println("PyroTechnic tests are completed");
}
void handleTest(){

    Serial.println("CURRENT STATE IS TESTING I AM BEGINNING THE TESTING PROCEDURE");
    currentState = stateTest;
    CheckPyro();
    delay(3000);
    CheckSensors();
    delay(3000);
    Serial.println("TEST IS FINISHED. PLEASE TURN OFF THE TEST SWITCH. TO TEST AGAIN PLEASE TURN IT BACK ON.");
    delay(5000);
    currentState = stateCalibration;
  
}
