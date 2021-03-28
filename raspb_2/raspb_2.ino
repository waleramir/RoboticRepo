const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 7;
const int IN4 = 6;
const int ENA = 11;
const int ENB = 10;

float dt, prev_time, prev_err, err, curr_time, sum;
float k_p = 0.15;
float k_i = 0.00;
float k_d = 0;
float setpoint = 160;
int areathresh = 15;
unsigned long timeold;

float dt_a, prev_time_a, prev_err_a, err_a, curr_time_a, sum_a;
float k_p_a = 6.5;
float k_i_a = 0.00;
float k_d_a = 0;
float setpoint_a = areathresh;

int cx = 160;
int area = areathresh;
int cxold = 160;
int speed = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop()
{
  if (millis() - timeold >= 10)
  {
    if (Serial.available() > 0)
    {
      cx = Serial.parseInt();
      area = Serial.parseInt();
      Serial.print("cx ");
      Serial.print(cx);
      Serial.print(" area ");
      Serial.println(area);

      if (area == 0 || cx < 1)
      {
        area = areathresh;
        cx = 160;
      }
      float pid = PID(cx);
      float pid_a = PID_a(area);

      if (pid_a > 0)
        FORWARD(min(90 + pid_a - pid, 255), min(99 + pid_a + pid, 255));
      else if (pid_a < 0)
        BACKWARD(min(90 + pid_a, 255), min(99 + pid_a, 255));
      else
        Stop();
      Serial.println(pid_a);
      cxold = cx;
      timeold = millis();
    }
    else
    {
      Stop();
    }
  }
}

float PID(float input)
{
  curr_time = millis();
  dt = millis() - prev_time;

  err = setpoint - input;
  sum += err * dt;
  if (abs(sum) > 10000)
    sum = 0;
  float u = k_p * err + k_i * sum + k_d * (err - prev_err) / dt;
  prev_err = err;
  prev_time = curr_time;
  return u;
}

float PID_a(float input)
{
  curr_time_a = millis();
  dt_a = millis() - prev_time_a;

  err_a = setpoint_a - input;
  sum_a += err_a * dt_a;
  if (abs(sum_a) > 10000)
    sum_a = 0;
  float u = k_p_a * err_a + k_i_a * sum_a + k_d_a * (err_a - prev_err_a) / dt_a;
  prev_err_a = err_a;
  prev_time_a = curr_time_a;
  return u;
}

void FORWARD(int Speed1, int Speed2)
{

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, Speed1);
  analogWrite(ENB, Speed2);
}
void BACKWARD(int Speed1, int Speed2)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, Speed1);
  analogWrite(ENB, Speed2);
}

void Stop()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}