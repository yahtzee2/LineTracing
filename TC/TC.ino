#define Ki 0
#include "NKP_TCSensor.h"
#include <popx2.h>
uint8_t numSensor = 4;

// การปรับ PID นั้น เริ่มจากการปรับที่ KP ก่อน แล้วให้ KI KD ตัวอื่น ๆให้เป็น 0 ให้หมด
// เมื่อหุ่นยนต์ของเรา วิ่งกลับมาจุดกลึงกลางแล้วก็ปรับ KD เพิ่มมากชึ้นทำให้การสวิงนั้นต่ำลง
// พร้อมกับลดค่า KP ลงเพิ่มเติม
// ทำไมถึงไม่พูดถึง KI ก็เพราะว่า ตามหลักแล้ว KI ที่จะใช้คือน้อยมาก ๆเลย ผมเลยไม่ขอยกขึ้นมานะครับ
// หรือถ้าปรับ KI ไม่ดีจะทำให้เกิดการ ทำให้ระบบมันระเบิด เอาภาษาง่าย ๆแล้วมันก็จะไม่จับเส้ยนะจ่ะ
uint16_t setpoint;
float present_position;
float errors = 0;
float output = 0;
float integral ;
float derivative ;
float previous_error ;
void setup() {
  OK();
  Serial.begin(115200);
  setSensorPins((const int[]) {25, 26, 27 , 28}, 4);
  //setSensorMax((const int[]) {1, 2, 3, 4});
  //setSensorMin((const int[]) {5, 6, 7, 8});
  for(int i = 0;i<5000;i++){
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
  beep();
  /*for(int i=0;i<4;i++)
  {
    glcd(i,0,"%d    ",ReadSensorMaxValue(i));
    glcd(i,5,"%d    ",ReadSensorMinValue(i));
  }*/
  sw_ok_press();
  /*pid_T(1,0,40,3800);
  pid_T(0.6,3,35,300);
  square(6);
  pid_BB(0.675,4,60);
  square(1);
  sr(40);
  delay(20);
  ao();
  pid_T(0.675,4,40,800);
  pid_BB(0.675,4,60);
  pid_BB(0.675,4,60);
  pid_BB(0.675,4,60);
  pid_BB(0.675,4,60);
  pid_T(0.675,4,40,1300);
  pid_BB(0.65,5,60);
  pid_BB(0.725,7,70);
  pid_T(0.85,8,90,1300);
  pid_BB(0.85,8,80);
  pid_T(0.9,5,90,700);
  pid_ST(0.45,2,50);
  pid_T(0.7,5,90,500);
  pid_ST(0.45,2,50);
  pid_T(0.7,5,90,500);
  pid_ST(0.45,2,50);
  pid_T(0.7,5,90,500);
  pid_STS(0.45,2,50);
  pid_T(0.7,5,90,500);
  pid_STS(0.45,2,50);
  pid_T(0.7,5,90,200);
  pid_ST(0.45,2,50);
  pid_ST(0.45,2,50);*/
  pid_STS(0.3,2,50);
  pid_ST(0.3,2,50);
  squareSP();
}

void loop() {
  /*Serial.print("  readline");
  Serial.println(readline());
  delay(1);*/
  /*for (int i = 1; i <= 4; i++)
  {
    glcd(i, 0, "%d   ", analog(i));
  }//*/
  ao();
}

void pid(float Kp , float Kd , int speed_max)
{
  present_position = readline() / ((numSensor - 1) * 10) ;
  setpoint = 50.0;
  errors = setpoint - present_position;
  integral = integral + errors ;
  derivative = (errors - previous_error) ;
  output = Kp * errors + Ki * integral + Kd * derivative;
  previous_error = errors;
  if(output > 100 )output = 100;
  else if(output < -100)output = -100;
  motor(1, speed_max - output);
  motor(2, speed_max + output);
  delay(1);
}

void pid_T(float Kp , float Kd , int speed_max,long mil)
{
  long a = millis();
  while(a+mil>millis())pid(Kp , Kd , speed_max);
}

void pid_B(float Kp , float Kd , int speed_max)
{
  while(1)
  {
    pid(Kp,Kd,speed_max);
    if((analog(4)<300&&analog(3)<300)||(analog(1)<300&&analog(2)<300))
    {
      break;
    }
  }
}

void square(int num)
{
  while(num--)
  {
    while(1)
    {
      pid(0.6,3,35);
      if(analog(0)<400)
      {
        fd(30);
        delay(60);
        ao();
        sl(40);
        while(analog(3)>500);
        ao();
        break;
      }
      if(analog(5)<400)
      {
        fd(30);
        delay(60);
        ao();
        sr(40);
        while(analog(2)>500);
        ao();
        break;
      }
    }
  }
}

void pid_BB(float Kp , float Kd , int speed_max)
{
  while(analog(5)>300&&analog(0)>300)pid(Kp,Kd,speed_max);
  while(analog(5)<300||analog(0)<300)pid(Kp,Kd,speed_max);
  pid_T(Kp,Kd,speed_max,100);
}

void pid_ST(float Kp , float Kd , int speed_max)
{
  while(analog(5)>300&&analog(0)>300)pid(Kp,Kd,speed_max);
  pid_T(0.7,5,40,750);
}

void pid_STS(float Kp , float Kd , int speed_max)
{
  while(analog(5)>300&&analog(0)>300)pid(Kp,Kd,speed_max);
  pid_T(0.7,5,40,350);
}

void squareSP()
{
  int n=2;
  while(n--)
  {
    while(1){
    if(analog(0)<400)
      {
        fd(30);
        delay(90);
        ao();
        sl(40);
        while(analog(3)>500);
        ao();
        delay(50);
        break;
      }
      if(analog(5)<400)
      {
        fd(30);
        delay(90);
        ao();
        sr(40);
        while(analog(2)>500);
        ao();
        delay(50);
        break;
      }
    if(analog(2)<300||analog(3)<300)pid(0.8,5,40);
    else fd(40);
    }
  }
  long a=millis();
  while(a+600>millis()){
  if(analog(2)<300||analog(3)<300)pid(0.7,5,40);
  else fd(40);
  }
}

