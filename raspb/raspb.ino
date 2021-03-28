const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 7;
const int IN4 = 6;
const int ENA = 11;
const int ENB = 10;

float dt, prev_time, prev_err, err, curr_time, sum;
float k_p = 0.39;
float k_i = 0.001;
float k_d = 27;
unsigned long timeold;

float setpoint = 160;
int cx = 160;
int cxold = 160;
int speed = 90;

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
      float pid = PID(cx);
      FORWARD(min(max(speed - pid, speed), 255), min(max(speed + pid, speed), 255));
      cxold = cx;
      timeold = millis();
    }
    else
    {
      float pid = PID(cxold);
      FORWARD(min(max(speed - pid, speed), 255), min(max(speed + pid, speed), 255));
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

void FORWARD(int Speed1, int Speed2)
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