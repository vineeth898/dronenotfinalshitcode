#include <Servo.h>
#include<Wire.h>
const int MPU = 0x68;
Servo m1;
Servo m2;
Servo m3;
Servo m4;
int ch1, ch2, ch3, ch4, ch5, ch6;
int cha1 = A0;
int cha2 = A1;
int cha3 = A2;
int cha4 = A3;
int cha5 = 12;
int cha6 = 13;
int motval = 0;
float add1 = 0;
float add2 = 0;
float add3 = 0;
float add4 = 0;
int mv1 = 1000, mv2 = 1000, mv3 = 1000, mv4 = 1000;
String state;
int avg;
int State = 0;
int no = 100;
#include <Wire.h>
#include <MPU6050.h>
int pitchaccel;
int rollaccel;
MPU6050 mpu;
int avgaccelpitch, avgaccelroll;
unsigned long timer = 0;
float timeStep = 0.01;
float pitch = 0;
float roll = 0;
float yaw = 0;
int count = 0;
void setup() {
  delay(3000);
  Serial.begin(115200);
  Serial.println("Initialize MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  pinMode(cha1, INPUT);
  pinMode(cha2, INPUT);
  pinMode(cha3, INPUT);
  pinMode(cha4, INPUT);
  pinMode(cha5, INPUT);
  pinMode(cha6, INPUT);
  m1.attach(3);
  m2.attach(9);
  m3.attach(6);
  m4.attach(5);
  m1.writeMicroseconds(2000);
  m2.writeMicroseconds(2000);
  m3.writeMicroseconds(2000);
  m4.writeMicroseconds(2000);
  delay(4000);
  m1.writeMicroseconds(1000);
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  m4.writeMicroseconds(1000);
  delay(2000);
  m1.writeMicroseconds(1000);
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  m4.writeMicroseconds(1000);
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);

}
int basevall = 0;
void loop() {
  ch3 = pulseIn(cha3, HIGH);
  basevall = map(ch3, 1224, 1882, 1000, 2000);
if(basevall<900){
    m2.writeMicroseconds(0);
    m4.writeMicroseconds(0);
    m1.writeMicroseconds(0);
    m3.writeMicroseconds(0);
    delay(1000000);
}
  hover(basevall);
}

void readaccelval() {
  Vector normAccel = mpu.readNormalizeAccel();

  // Calculate Pitch & Roll
  pitchaccel = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI;
  rollaccel = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;
  if (rollaccel < (0)) {
    rollaccel = map(rollaccel, -178, -85, 0, -85);
  }
  else {
    rollaccel = map(rollaccel, 178, 85, 0, 85);
  }
}

void readgyro() {
  timer = millis();
  count = count + 1;
  if (count % 100 == 0) {
    int avgr = 0;
    int avgp = 0;
    for (int x = 0; x < 100; x++) {
      readaccelval();
      avgr += rollaccel;
      avgp += pitchaccel;
    }
    avgr = avgr / 100;
    avgp = avgp / 100;
    Serial.print("AVERAGE=");
    Serial.print(avgr);
    Serial.print("<--roll,pitch->>");
    Serial.print(avgp);
    if (avgr < 1 && avgr > -1) {
      roll = 0;
    }
    if (avgp < 1 && avgp > -1) {
      pitch = 0;
    }
    return;
  }
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = (pitch + norm.YAxis * timeStep);
  roll = (roll + norm.XAxis * timeStep);
  yaw = yaw + norm.ZAxis * timeStep;
  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);
  Serial.print(" Yaw = ");
  Serial.print(yaw);
  Serial.print(" , ");

  // Wait to full timeStep period
  delay((timeStep * 1000) - (millis() - timer));
}
void hover(int Speed) {
  readgyro();
  float add;
  if (pitch < 0) {
    add = -(pitch/20);
    add1 += add;
  }
  if (pitch > 0) {
    add =pitch/20;
    add2 += add;
  }
  if (roll < 0) {
    add = -(roll/20);
    add3 += add;
  }
  if (roll > 0) {
    add = roll/20;
    add4 += add;
  }
    m2.writeMicroseconds(Speed + add1);
    m4.writeMicroseconds(Speed + add2);
    m1.writeMicroseconds(Speed + add3);
    m3.writeMicroseconds(Speed + add4);
  Serial.print("ma1= ");
  Serial.print(Speed + add1);
  Serial.print(" ,ma2= ");
  Serial.print(Speed + add2);
  Serial.print(" ,ma3= ");
  Serial.print(Speed + add3);
  Serial.print(" ,ma4= ");
  Serial.println(Speed + add4);

}
