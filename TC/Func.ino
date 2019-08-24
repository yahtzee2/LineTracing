void left() {
  sl(40); delay(100); ao();  sl(40);
  while (analog(2) > 500); ao();
  ao(); delay(20);
}
void right() {
  sr(40); delay(100); ao();  sr(40);
  while (analog(6) > 500); ao();
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
  while (analog(5) > 500 && analog(0) > 500)pid(Kp, Kd, speed_max);
  while (analog(5) < 500 || analog(0) < 500)pid(Kp, Kd, speed_max);
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
