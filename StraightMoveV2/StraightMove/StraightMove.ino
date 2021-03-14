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
float pwm_min = 35;
float pwm_max = 255;
float S_r, S_l, S;

float dt_v, prev_time_v, prev_err_v, curr_time_v, sum_v, u_v;
float k_p_v = 2.5;
float k_i_v =0.000;
float k_d_v = 0.00;

float dt_w, prev_time_w, prev_err_w, curr_time_w, sum_w, u_w;
float k_p_w = 1;//2.48
float k_i_w = 0.0000;
float k_d_w = 0.00000;

float L = 12.3;
float r = 6.5 / 2;
float rpm_max = 700;
float v_max = (rpm_max) * (r);
float w_max = (r / L) * rpm_max;
float setpoint_v = 55;
float setpoint_w = 0;
float v_pwm_r, v_pwm_l, w_pwm, v, w, pwm_r, pwm_l;


void counter_1() {
  pulses_1++;
}

void counter_2() {
  pulses_2++;
}

void Move() {
  if (millis() - timeold >= 1000) {
    detachInterrupt(digitalPinToInterrupt(encoder_pin_1));
    detachInterrupt(digitalPinToInterrupt(encoder_pin_2));

    rpm_r = (pulses_1 * 60) / (HOLES_DISC);
    rpm_l = (pulses_2 * 60) / (HOLES_DISC);

    v_pwm_r = 90;
    v_pwm_l = 99;
    w_pwm = 0;
    v = (rpm_r + rpm_l) / (rpm_max) * 100;
    w = (rpm_r - rpm_l)  / rpm_max * 100;

    float err_v = setpoint_v - v;
    float err_w = setpoint_w - w;
    float pidv = PID_v(err_v);
    float pidw = PID_w(err_w);
    pwm_r = v_pwm_r + pidv + w_pwm + pidw;
    pwm_l = v_pwm_l + pidv - w_pwm - pidw;

    Serial.print("v: ");
    Serial.print(v);
    Serial.print(" w: ");
    Serial.print(w);
    Serial.print(" rpml: ");
    Serial.print(rpm_l);
    Serial.print(" rpmr: ");
    Serial.println(rpm_r);
    FORWARD(min(max(90, pwm_l), 255), min(max(90, pwm_r), 255));
     //FORWARD(90,90);
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
  S_r = (pulses_1 / HOLES_DISC) * 3.14 * 6.5;
  S_l = (pulses_2 / HOLES_DISC) * 3.14 * 6.5;
  S =  (S_r + S_l) / 2;

  Move();


}
float sm(float sm)
{
  return sm * 0.566;
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
