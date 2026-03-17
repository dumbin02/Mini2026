#include "Setup.h"
void leerMPU() {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);               // start reading accel data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();       // skip temperature
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}
/*
void printMPU(){
  Serial.print(ax/16384.00); Serial.print(" ");
  Serial.print(ay/16384.00); Serial.print(" ");
  Serial.print(az/16384.00); Serial.print(" ");

  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.println(gz);
}*/

void leerIRs(){
  valorIRIzquierdo  =   analogRead(SENSORIZQUIERDO);
  valorIRFrontal    =   analogRead(SENSORFRONTAL);
  valorIRDerecho    =   analogRead(SENSORDERECHO);
}

void printIRs(){
}