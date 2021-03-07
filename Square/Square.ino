#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

int encoder_pin_1 = 2;
int encoder_pin_2 = 3;
int rpm_r, rpm_l;
volatile unsigned int pulses_1;
volatile unsigned int pulses_2;
unsigned long timeold;

#define HOLES_DISC 20

#define ENA 11
#define ENB 10
#define IN4 6
#define IN3 7
#define IN2 8
#define IN1 9

float err_1, err_2, u_1, u_2;
float rpm_setpoint = 100;
float pwm_min = 100;
float pwm_max = 255;
float S_r, S_l, S;

float dt_v, prev_time_v, prev_err_v, curr_time_v, sum_v, u_v;
float k_p_v = 1;
float k_i_v = 0.00;
float k_d_v = 0;

float dt_w, prev_time_w, prev_err_w, curr_time_w, sum_w, u_w;
float k_p_w = 2;
float k_i_w = 0.00;
float k_d_w = 0;

float L = 12.3;
float r = 6.5 / 2;
float rpm_max = 250;
float v_max = rpm_max * r;
float w_max = (r / L) * 250;
float setpoint_v = 5;
float setpoint_w = 0;
float v_pwm_r, v_pwm_l, w_pwm, v, w, pwm_r, pwm_l;

float yaw = 0;
float maxangle[] = {90, 180, 270};
int turn = 0;
bool mode = true;
float off = 0;

void counter_1() {
  pulses_1++;
}

void counter_2() {
  pulses_2++;
}
// для прямого движения использовал регуляторы для линейной и угловой скорости
void Move() {
  if (millis() - timeold >= 1000) {
    detachInterrupt(digitalPinToInterrupt(encoder_pin_1));
    detachInterrupt(digitalPinToInterrupt(encoder_pin_2));

    rpm_r = (pulses_1 * 60) / (HOLES_DISC);
    rpm_l = (pulses_2 * 60) / (HOLES_DISC);

    v_pwm_r = 90;
    v_pwm_l = 99;
    w_pwm = 0;
    v = (rpm_r + rpm_l) / (2 * rpm_max) * 100;
    w = (rpm_r - rpm_l)  / rpm_max * 100;

    float err_v = setpoint_v - v;
    float err_w = setpoint_w - w;
    float pidv = PID_v(err_v);
    float pidw = PID_w(err_w);
    pwm_r = v_pwm_r + pidv + w_pwm + pidw;
    pwm_l = v_pwm_l + pidv - w_pwm - pidw;

    FORWARD(min(max(90, pwm_l), 255), min(max(90, pwm_r), 255));

    timeold = millis();
    pulses_1 = 0;
    pulses_2 = 0;

    attachInterrupt(digitalPinToInterrupt(encoder_pin_1), counter_1, FALLING);
    attachInterrupt(digitalPinToInterrupt(encoder_pin_2), counter_2, FALLING);
  }

}

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  mpu.begin();

  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  mpu.setFilterGyroCoef(0.98);
  pinMode(encoder_pin_1, INPUT);
  pinMode(encoder_pin_2, INPUT);

  attachInterrupt(0, counter_1, FALLING);
  attachInterrupt(1, counter_2, FALLING);

  pulses_1 = 0;
  pulses_2 = 0;
  rpm_r = 0;
  rpm_l = 0;
  timeold = 0;

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

}

void loop()
{
  mpu.update();
  S_r = (pulses_1 / HOLES_DISC) * 3.14 * 6.5;
  S_l = (pulses_2 / HOLES_DISC) * 3.14 * 6.5;
  S =  (S_r + S_l) / 2;
  yaw = mpu.getAngleZ();

  if (turn != 4)
  {
    float angle = maxangle[turn];

    if (mode)
    {
      if (S > sm(30))
      {
        Stop();
        mode = !mode;
        delay(1000);
      }
      else
        Move();
    }
    else
      ROTATE_TO(angle);
  }
}

float sm(float sm)
{
  return sm * 0.566;
}

// Для измерения углов были использованы показания гироскопа и отрегулирована мощность сигнала на вход моторов
// Данный подход был использован так как другие методы такие как пид регуляторы не дали удовлетворительных результатов 
// и измерение по угловой скорости было не таким точным
void ROTATE_TO (float angle)
{
  if (abs(yaw) >= maxangle[turn] + off)
  {
    Stop();
    turn++;
    mode = !mode;
    pulses_1 = 0;
    pulses_2 = 0;
    off = yaw - maxangle[turn - 1];
    delay(1000);

  }
  else
  {
    ANTIROTATE(97);
  }

}

float PID_v(float err_v)
{
  curr_time_v = millis();
  dt_v = millis() - prev_time_v;

  sum_v += err_v * dt_v;
  float u = k_p_v * err_v + k_i_v * sum_v + k_d_v * (err_v - prev_err_v) / dt_v;
  prev_err_v = err_v;
  prev_time_v = curr_time_v;
  return u;
}

float PID_w(float err_w)
{
  curr_time_w = millis();
  dt_w = millis() - prev_time_w;

  sum_w += err_w * dt_w;
  float u = k_p_w * err_w + k_i_w * sum_w + k_d_w * (err_w - prev_err_w) / dt_w;
  prev_err_w = err_w;
  prev_time_w = curr_time_w;
  return u;
}



void ROTATE (int Speed)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
}

void ANTIROTATE (int Speed)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
}

void FORWARD (int Speed1, int Speed2)
{

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, Speed1);
  analogWrite(ENB, Speed2);
}

void Stop()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
