#include<popx2.h>
#define Ki 0
#include "NKP_TCSensor.h"

uint8_t numSensor = 6;
int a = 0;

uint16_t setpoint;
float present_position;
float errors = 0;
float output = 0;
float integral ;
float derivative ;
float previous_error ;
int ref[10];
void setup() {
  OK();
  Serial.begin(115200);
  setSensorPins((const int[]) {
    24, 25, 26, 30 , 28, 29
  }, 6); 
  /*setSensorMax((const int[]) {1, 2, 3, 4});
    setSensorMin((const int[]) {5, 6, 7, 8});*/
  for (int i = 0; i < 5000; i++) {
    setCalibrate();
  }
  /*for (uint8_t i = 0; i < 4; i++)
    {
    Serial.print(ReadSensorMinValue(i));
    Serial.print(' ');
    }
    Serial.println();
    for (uint8_t i = 0; i < 4; i++)
    {
    Serial.print(ReadSensorMaxValue(i));
    Serial.print(' ');
    }
    Serial.println();
    delay(1000);*/
  for (int i = 1; i <= 6; i++) {
    ref[i] = (_min_sensor_values[i - 1] + _min_sensor_values[i - 1]) / 2;
  }
  beep();
  int x = 0 , Nmenu = 4;
  while (1) {
    glcd(1, 0, " %d ", x % Nmenu);
    if (sw1()) {
      x++;
    }
    if (sw_ok()) break;
    delay(100);
  }
  beep(); delay(1000);
  if (x % Nmenu == 0) {
    pid_T(1.25, 6.5, 70, 2500); //save
    pid_T2(1.25, 6.5, 70, 2000);
    pid_T(1.3, 6.75, 70, 100);
    pid_T(1.3, 6.75, 60, 8000);
    esc();
  }
  else if (x % Nmenu == 1 ) {
    pid_T(0.9, 8, 55, 7100); //here 161
    pid_BB(0.85, 6.5, 50);
    sl(40); delay(100); ao();  
    sl(40); while (analog(2) > 500); ao();
    pid_T(0.85, 6.5, 75, 800);
    sapanleft(0.85, 6.5, 55); //reverse
    pid_T(0.85, 6.5, 40, 500);
    pid_T(0.85, 6.75, 60, 2100); //6.5
    for (int i = 0; i < 6; i++) pid_BB(0.85, 6.5, 65);
    pid_T(0.9, 6.5, 50, 400);
    pid_BB(0.85, 6.5, 50);
    for (int i = 0; i < 5; i++) pid_BB(0.85, 6.5, 65);
    left(); pid_BB(0.85, 6.5, 60); left();
    for (int i = 0; i < 5; i++) pid_BB(0.85, 6.5, 70); pid_T(0.9, 6.5, 50, 400);
    for (int i = 0; i < 6; i++) pid_BB(0.85, 6.5, 65);
    left(); pid_BB(0.85, 6.5, 60); left();
    for (int i = 0; i < 5; i++) pid_BB(0.85, 6.5, 70); pid_T(0.9, 6.5, 50, 400);
    for (int i = 0; i < 6; i++) pid_BB(0.85, 6.5, 70);
    pid_T(0.85, 6.5, 60, 500);
    pid_T(1.1, 6.5, 80, 2200);
    pid_BB(0.85, 7, 50);
    for (int i = 0; i < 5; i++) pid_BB(0.85, 6.5, 70);
    right(); pid_BB(0.85, 6.5, 60); right();
    for (int i = 0; i < 5; i++) pid_BB(0.85, 6.5, 70); pid_T(0.9, 6.5, 50, 700);
    //pid_BB(0.85,7,50);
    for (int i = 0; i < 5; i++) pid_BB(0.85, 6.5, 70);
    right(); pid_BB(0.85, 6.5, 60); right();
    for (int i = 0; i < 5; i++) pid_BB(0.85, 6.5, 70); pid_T(0.9, 6.5, 50, 700);
    for (int i = 0; i < 5; i++) pid_BB(0.85, 6.5, 65);
    right(); pid_BB(0.85, 6.5, 60); right();
    for (int i = 0; i < 5; i++) pid_BB(0.85, 6.5, 65); //here
    //pid_T(0.85,6.5,65,5000);
    pid_T(1.1, 6.75, 75, 2500);
  }
}
void loop() {
  /*Serial.print("  readline");
    Serial.println(readline());
    delay(1);*/
  /*for (int i = 0; i <= 5; i++)
    {
    glcd(i, 0, "%d   ", analog(i));
    }//*/
  ao();
}
