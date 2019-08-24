#include<popx2.h>
#define Ki 0
#include "NKP_TCSensor.h"

uint8_t numSensor = 4;
int a = 0;
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
int ref[10];
void setup() {
  OK();
  Serial.begin(115200);
  setSensorPins((const int[]) {
     25, 26, 30 , 28
  }, 4);
  /*setSensorMax((const int[]) {1, 2, 3, 4});
    setSensorMin((const int[]) {5, 6, 7, 8});*/
 /* for (int i = 0; i < 5000; i++) {
    setCalibrate();
  }
  for (uint8_t i = 0; i < 4; i++)
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
  _min_sensor_values[0] = 146, _max_sensor_values[0] = 982;
  _min_sensor_values[1] = 133, _max_sensor_values[1] = 981;
  _min_sensor_values[2] = 159, _max_sensor_values[2] = 986;
  _min_sensor_values[3] = 79, _max_sensor_values[3] = 950;
  for (int i = 1; i <= 6; i++) {
    ref[i] = (_min_sensor_values[i - 1] + _min_sensor_values[i - 1]) / 2;
  }
  beep();
  int x = 0;
  while (1) {
    glcd(1, 0, " %d ", x % 8);
    if (sw1()) {
      x++;
    }
    if (sw_ok()) break;
    delay(100);
  }
  beep(); delay(1000);
 /*if (x % 8 == 0) {
     pid_BB(1.225 ,6.5 ,80);
    pid_BL(1.225 ,6.5,  70); left();
    pid_BB(1.225 ,6.5 ,70); 
    //ao(); delay(20);
    pid_T(1.225 ,6.5 ,80,500);
    pid_BB(1.25  ,6.6,70);
    pid_BR(1.225 ,6.5,  60); right();
    // ao(); delay(20);
    pid_BB(1.25 ,6.6 ,70);
    pid_T(1.25 ,6.6 ,80,2000);
    pid_T(1.21 ,6.6 ,90,8000);
    pid_BB(1.21 ,6.5 ,90);
    
  }
  else if (x%8==1){
    pid_BB(1.21 ,6.5 ,80);
    pid_BL(1.21 ,6.5,  70); left();
    pid_BB(1.21 ,6.5 ,70); 
    //ao(); delay(20);
    pid_T(1.21 ,6.5 ,80,500);
    pid_BB(1.25  ,6.6,70);
    pid_BR(1.21 ,6.5,  60); right();
    // ao(); delay(20);
    pid_BB(1.25 ,6.6 ,70);
    pid_T(1.25 ,6.6 ,80,2000);
    pid_T(1.21 ,6.5 ,90,8000);
    pid_BB(1.21 ,6.5 ,90);
   
  }
  else if (x%8==2){
    pid_BB(1.21 ,6.5 ,80);
    pid_BL(1.21 ,6.5,  70); left();
    pid_BB(1.21 ,6.5 ,70); 
    //ao(); delay(20);
    pid_T(1.21 ,6.5 ,80,500);
    pid_BB(1.25  ,6.6,70);
    pid_BR(1.21 ,6.5,  60); right();
    // ao(); delay(20);
    pid_BB(1.25 ,6.6 ,70);
    pid_T(1.25 ,6.6 ,80,2000);
    pid_T(1.21 ,6.5 ,95,8000);
    pid_BB(1.21 ,6.5 ,95);
  }*/
   if (x%8==0){
    pid_BB(1.225 ,6.5 ,80);
    pid_BL(1.225 ,6.5,  70); left();
    pid_BB(1.225 ,6.5 ,70); 
    pid_BB(1.225 ,6.5 ,80); 
    pid_BR(1.225 ,6.5,  60); right();
    pid_BB(1.225 ,6.5 ,70); 
    pid_BR(1.225 ,6.5,  60);
    pid_T(1.225 ,6.5 ,50,200); 
    sr(50); delay (100);
    ao(); 
    while(1)
    {
      if (analog(1) > 700 || analog(2) > 700 || analog(6) > 700 || analog(4)>700) break;
      fd(50);
    } 
    pid_T(1.225 ,6.5 ,70,1000); 
    pid_T(1.21 ,6.5 ,90,100000); 
  }
  else if (x%8==1){
    pid_BB(1.21 ,6.5 ,90);
    pid_BL(1.21 ,6.5,  70); left();
    pid_BB(1.21 ,6.5 ,70); 
    pid_BB(1.21 ,6.5 ,90); 
    pid_BR(1.21 ,6.5,  70); right();
    pid_BB(1.21 ,6.5 ,80); 
    pid_BR(1.21 ,6.5,  80);
    pid_T(1.21 ,6.5 ,50,200); 
    sr(50); delay (100);
    ao(); 
    while(1)
    {
      if (analog(1) > 700 || analog(2) > 700 || analog(6) > 700 || analog(4)>700) break;
      fd(50);
    } 
    pid_T(1.21 ,6.5 ,70,1000); 
    pid_T(1.21 ,6.5 ,90,100000); 
  }
   else if (x%8==2){
   pid_BB(1.21 ,6.5 ,90);
    pid_BL(1.21 ,6.5,  70); left();
    pid_BB(1.21 ,6.5 ,70); 
    pid_BB(1.21 ,6.5 ,90); 
    pid_BR(1.21 ,6.5,  70); right();
    pid_BB(1.21 ,6.5 ,80); 
    pid_BR(1.21 ,6.5,  80);
    pid_T(1.21 ,6.5 ,50,200); 
    sr(50); delay (100);
    ao(); 
    while(1)
    {
      if (analog(1) > 700 || analog(2) > 700 || analog(6) > 700 || analog(4)>700) break;
      fd(50);
    } 
    pid_T(1.21 ,6.5 ,70,1000); 
    pid_T(1.21 ,6.5 ,91,100000);
  }
    else if (x%8==3){
     pid_BB(1.225 ,6.5 ,80);
    pid_BL(1.225 ,6.5,  70); left();
    pid_BB(1.225 ,6.5 ,70); 
    pid_BB(1.225 ,6.5 ,80); 
    pid_BR(1.225 ,6.5,  60); right();
    pid_BB(1.225 ,6.5 ,70); 
    pid_BR(1.225 ,6.5,  60);
    pid_T(1.225 ,6.5 ,50,200); 
    sr(50); delay (100);
    ao(); 
    while(1)
    {
      if (analog(1) > 700 || analog(2) > 700 || analog(6) > 700 || analog(4)>700) break;
      fd(50);
    } 
    pid_T(1.225 ,6.5 ,70,1000); 
    pid_T(1.21 ,6.5 ,85,100000); 
    }
    else if (x%8==4){
    pid_BB(1.225 ,6.5 ,80);
    pid_BL(1.225 ,6.5,  70); left();
    pid_BB(1.225 ,6.5 ,70); 
    pid_BB(1.225 ,6.5 ,80); 
    pid_BR(1.225 ,6.5,  60); right();
    pid_BB(1.225 ,6.5 ,70); 
    pid_BR(1.225 ,6.5,  60);
    pid_T(1.225 ,6.5 ,50,200); 
    sr(50); delay (100);
    ao(); 
    while(1)
    {
      if (analog(1) > 700 || analog(2) > 700 || analog(6) > 700 || analog(4)>700) break;
      fd(50);
    } 
    pid_T(1.225 ,6.5 ,70,1000); 
    pid_T(1.21 ,6.5 ,80,100000); 
    }
    else if (x%8==7){
      while(1){
      //glcd(0, 0, "%d   ", analog(0));
     glcd(1, 0, "%d   ", analog(1));
     glcd(2, 0, "%d   ", analog(2));
     glcd(3, 0, "%d   ", analog(6));
     glcd(4, 0, "%d   ", analog(4));
     //glcd(5, 0, "%d   ", analog(5));
      }
      }
  //pid_T(1.3,7.5,90,40000);
  //sw_ok_press();

  /*pid_T(0.85,6.5,61,1500);
    for(int i=0;i<6;i++) pid_BB(0.85, 6.5,70);
    left(); pid_BB(0.85, 6.5,70); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,70);
    right(); pid_BB(0.85, 6.5,70); right();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,70);
    left(); pid_BB(0.85, 6.5,70); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,70);
    right(); pid_BB(0.85, 6.5,70); right();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,70);
    left(); pid_BB(0.85, 6.5,70); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,70);

    //////////
    pid_T(0.85,6.5,60,500);
    pid_T(0.85,6.5,70,2500); pid_BB(0.85, 6.5,60);
    for(int i=0;i<6;i++) pid_BB(0.85, 6.5,60);
    left(); pid_BB(0.85, 6.5,60); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    right(); pid_BB(0.85, 6.5,60); right();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    left(); pid_BB(0.85, 6.5,60); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    right(); pid_BB(0.85, 6.5,60); right();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    left(); pid_BB(0.85, 6.5,60); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    pid_T(0.85,6.5,60,1500);
    sapan(0.85,6.5,55);
    pid_T(0.85,6.5,40,1000);
    pid_BB(0.85, 6.5,60);
    sr(50); delay(100); ao();  sr(50);
    while(analog(6)>500); ao();
    pid_T(0.9,8,55,9000);*/



  /*pid_T(0.85,6.5,61,1500);
    for(int i=0;i<6;i++) pid_BB(0.85, 6.5,60);
    left(); pid_BB(0.85, 6.5,60); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    right(); pid_BB(0.85, 6.5,60); right();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    left(); pid_BB(0.85, 6.5,60); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    right(); pid_BB(0.85, 6.5,60); right();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    left(); pid_BB(0.85, 6.5,60); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);

    //////////
    pid_T(0.85,6.5,60,500);
    pid_T(0.85,6.5,70,2500); pid_BB(0.85, 6.5,60);
    for(int i=0;i<6;i++) pid_BB(0.85, 6.5,60);
    left(); pid_BB(0.85, 6.5,60); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    right(); pid_BB(0.85, 6.5,60); right();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    left(); pid_BB(0.85, 6.5,60); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    right(); pid_BB(0.85, 6.5,60); right();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    left(); pid_BB(0.85, 6.5,60); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,60);
    pid_T(0.85,6.5,60,1500);
    sapan(0.85,6.5,55);
    pid_T(0.85,6.5,40,1000);
    pid_BB(0.85, 6.5,60);
    sr(50); delay(100); ao();  sr(50);
    while(analog(6)>500); ao();
    pid_T(0.9,8,55,9000);*/


  ///////////////////////////////////////////////////////////////////////////////////////
  ////top ten
  //pid_T(0.9,6.5,75,2500);

  /* pid_T(0.9,8,55,7100);  //here 161
    pid_BB(0.85, 6.5,50);
    sl(40); delay(100); ao();  sl(40);
    while(analog(2)>500); ao();
    pid_T(0.85,6.5,75,800);
    sapanleft(0.85,6.5,55); //reverse
    pid_T(0.85,6.5,40,500);
    pid_T(0.85,6.75,60,2100); //6.5

    for(int i=0;i<6;i++) pid_BB(0.85, 6.5,65);
    pid_T(0.9,6.5,50,400);
    pid_BB(0.85, 6.5,50);
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,65);
    left(); pid_BB(0.85, 6.5,60); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,70); pid_T(0.9,6.5,50,400);
    for(int i=0;i<6;i++) pid_BB(0.85, 6.5,65);
    left(); pid_BB(0.85, 6.5,60); left();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,70); pid_T(0.9,6.5,50,400);
    for(int i=0;i<6;i++) pid_BB(0.85, 6.5,70);

    pid_T(0.85,6.5,60,500);
    pid_T(1.1,6.5,80,2200);
    pid_BB(0.85,7,50);
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,70);
    right(); pid_BB(0.85, 6.5,60); right();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,70); pid_T(0.9,6.5,50,700);
    //pid_BB(0.85,7,50);
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,70);
    right(); pid_BB(0.85, 6.5,60); right();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,70); pid_T(0.9,6.5,50,700);
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,65);
    right(); pid_BB(0.85, 6.5,60); right();
    for(int i=0;i<5;i++) pid_BB(0.85, 6.5,65);//here
    //pid_T(0.85,6.5,65,5000);
    pid_T(1.1,6.75,75,2500);///////////here*/
  //real code 33.38
  /* pid_T(0.85,6.5,61,10000);
    pid_T(0.9,6.5,70,1500);
    pid_T(0.85,6.5,61,10000);
    sapan(0.85,6.5,55);
    pid_T(0.85,6.5,40,1000);
    pid_BB(0.85, 6.5,60);
    sr(50); delay(100); ao();  sr(50);
    while(analog(3)>500); ao();
    pid_T(0.9,8,50,9000);*/

  // real code 33.60
  /*pid_T(0.85,6.5,61,23000);
    sapan(0.85,6.5,55);
    pid_T(0.85,6.5,40,1000);
    pid_BB(0.85, 6.5,60);
    sr(50); delay(100); ao();  sr(50);
    while(analog(3)>500); ao();
    pid_T(0.9,8,50,9000);*/
  // real code 34.08
  /*
    pid_T(0.85,6.5,60,23000);
    sapan(0.85,6.5,55);
    pid_T(0.85,6.5,40,1000);
    pid_BB(0.85, 6.5,60);
    sr(50); delay(100); ao();  sr(50);
    while(analog(3)>500); ao();
    pid_T(0.9,8,45,9000);
  */


  //real code 35.4

  /*pid_T(0.85,6.5,60,23000);
    sapan(0.85,6.5,55);
    pid_T(0.85,6.5,40,1000);
    pid_BB(0.85, 6.5,60);
    sr(50); delay(100); ao();  sr(50);
    while(analog(3)>500); ao();
    pid_T(0.9,8,40,10000);*/

  //real code 35.9
  /* pid_T(0.85,6.5,60,23000);
    sapan(0.85,6.5,55);
    pid_T(0.85,6.5,40,2000);
    pid_BB(0.85, 6.5,60 );
    sr(50); delay(100); ao();  sr(50);
    while(analog(3)>500); ao();
    pid_T(0.9,8,40,10000);*/

  //pid_T(0.9,7,70,21000);
}

void loop() {
  /*Serial.print("  readline");
    Serial.println(readline());
    delay(1);*/
  /*for (int i = 0; i <= 5; i++)
    {
    glcd(i, 0, "%d   ", analog(i));
    }//*/
    /* glcd(0, 0, "%d   ", analog(0));
     glcd(1, 0, "%d   ", analog(1));
     glcd(2, 0, "%d   ", analog(2));
     glcd(3, 0, "%d   ", analog(6));
     glcd(4, 0, "%d   ", analog(4));
     glcd(5, 0, "%d   ", analog(5));*/
  ao();
}

void left() {
 // fd(50); delay(50);
  sl(40); delay(100); ao();  sl(40);
  while (analog(2) < 500); ao();
  ao(); delay(20);

}
void right() {
  sr(40); delay(100); ao();  sr(40);
  while (analog(6) < 500); ao();
  ao(); delay(20);

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
  if (output > 100 )output = 100;
  else if (output < -100)output = -100;
  motor(1, speed_max - output);
  motor(2, speed_max + output);
  delay(1);
}
void pid2(float Kp , float Kd , int speed_max)
{
  
    if (analog(1) > 500 && analog(2) > 500 && analog(3) > 500 && analog(4) > 500) {
      fd(speed_max);
    }
    present_position = readline() / ((numSensor - 1) * 10) ;
    setpoint = 50.0;
    errors = setpoint - present_position;
    integral = integral + errors ;
    derivative = (errors - previous_error) ;
    output = Kp * errors + Ki * integral + Kd * derivative;
    previous_error = errors;
    if (output > 100 )output = 100;
    else if (output < -100)output = -100;
    motor(1, speed_max - output);
    motor(2, speed_max + output);
    delay(1);
  
}
void pid_T2(float Kp , float Kd , int speed_max, long mil)
{
  long a = millis();
  while (a + mil > millis())pid2(Kp , Kd , speed_max);
}
void pid_T(float Kp , float Kd , int speed_max, long mil)
{
  long a = millis();
  while (a + mil > millis())pid(Kp , Kd , speed_max);
}

void pid_B(float Kp , float Kd , int speed_max)
{
  while (1)
  {
    pid(Kp, Kd, speed_max);
    if ((analog(4) < 300 && analog(3) < 300) || (analog(1) < 300 && analog(2) < 300))
    {
      break;
    }
  }
}
void squarest(int num) {
  int x = 0;
  while (1) {
    if (num == 0)  break;
    else {
      if ( analog(0) < 500 ) {
        fd(30); delay(100);
        ao(); num -= 1; x = 0;
        while (analog(4) > 500)  sl(30);

      }
      else if ( analog(5) < 500 ) {
        fd(30); delay(100);
        ao(); num -= 1; x = 0;
        while (analog(1) > 500)  sr(30);

      }
      else if (analog(1) > 500 && analog(4) < 500) {
        motor(1, 40);
        motor(2, 10);
      }
      else if (analog(1) < 500 && analog(4) > 500) {
        motor(1, 10);
        motor(2, 40);
      }
      else {
        motor(1, 20 + x);
        motor(2, 20 + x);
      }
      x += 3;
      if (x > 20) x = 20;
    }
  }
}
void square(int num)
{
  while (num--)
  {
    while (1)
    {
      pid(0.6, 3, 35);
      if (analog(0) < 400)
      {
        fd(30);
        delay(60);
        ao();
        sl(40);
        while (analog(3) > 500);
        ao();
        break;
      }
      if (analog(5) < 400)
      {
        fd(30);
        delay(60);
        ao();
        sr(40);
        while (analog(2) > 500);
        ao();
        break;
      }
    }
  }
}
void sapanleft(float Kp , float Kd , int speed_max)
{
  while (analog(5) > 300 && analog(0) > 300)pid(Kp, Kd, speed_max);
  while (analog(5) < 300 || analog(0) < 300)pid(Kp, Kd, speed_max);
  ao(); sl(40); delay(200); ao();
  //pid_T( Kp ,Kd ,speed_max,100);
  sl(40); while (analog(2) > 500); ao();
  //sr(40); while (analog(3) < 500); ao();
  //pid_T(Kp,Kd,speed_max,100);
}
void sapan(float Kp , float Kd , int speed_max)
{
  while (analog(5) > 300 && analog(0) > 300)pid(Kp, Kd, speed_max);
  while (analog(5) < 300 || analog(0) < 300)pid(Kp, Kd, speed_max);
  ao(); sr(40); delay(200); ao();
  //pid_T( Kp ,Kd ,speed_max,100);
  sr(40); while (analog(6) > 500); ao();
  //sr(40); while (analog(3) < 500); ao();
  //pid_T(Kp,Kd,speed_max,100);
}
void pid_BB(float Kp , float Kd , int speed_max)
{
  while (analog(5) < 500 && analog(0) < 500)pid(Kp, Kd, speed_max);
  while (analog(5) > 500 || analog(0) > 500)pid(Kp, Kd, speed_max);
  ao();
  //pid_T(Kp,Kd,speed_max,100);
}

void pid_ST(float Kp , float Kd , int speed_max)
{
  while (analog(5) > 300 && analog(0) > 300)pid(Kp, Kd, speed_max);
  pid_T(0.7, 5, 40, 750);
}

void pid_STS(float Kp , float Kd , int speed_max)
{
  while (analog(5) > 300 && analog(0) > 300)pid(Kp, Kd, speed_max);
  pid_T(0.7, 5, 40, 350);
}

void squareSP()
{
  int n = 2;
  while (n--)
  {
    while (1) {
      if (analog(0) < 400)
      {
        fd(30);
        delay(90);
        ao();
        sl(40);
        while (analog(3) > 500);
        ao();
        delay(50);
        break;
      }
      if (analog(5) < 400)
      {
        fd(30);
        delay(90);
        ao();
        sr(40);
        while (analog(2) > 500);
        ao();
        delay(50);
        break;
      }
      if (analog(2) < 300 || analog(3) < 300)pid(0.8, 5, 40);
      else fd(40);
    }
  }
  long a = millis();
  while (a + 600 > millis()) {
    if (analog(2) < 300 || analog(3) < 300)pid(0.7, 5, 40);
    else fd(40);
  }
}

void esc() {
  while (in(18) != 0)  pid(1.3, 6.75, 60);
  ao(); delay(300);
  sr(50); delay(75); ao();//75
  fd(50); delay(500); ao();
  sl(50); delay(150); ao();
  while (analog(1) > ref[1] && analog(2) > ref[2] && analog(3) > ref[3] && analog(4) > ref[4]) fd(50);
  ao();
  pid_T(1.3, 6.75, 70, 10000);
  //pid_T(0.85,6.5,60,200);
}


void pid_BL(float Kp , float Kd , int speed_max)
{
  while (analog(0)  <500  ||  analog(5) >  500) pid(Kp, Kd, speed_max);
  ao();
  //pid_T(Kp,Kd,speed_max,100);
}

void pid_BR(float Kp , float Kd , int speed_max)
{
  while (analog(5)  <500  ||  analog(0) >  500) pid(Kp, Kd, speed_max);
  ao();
  //pid_T(Kp,Kd,speed_max,100);
}
