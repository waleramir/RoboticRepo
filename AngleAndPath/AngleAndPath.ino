int encoder_pin_1 = 2;
int encoder_pin_2 = 3;
int rpm_1, rpm_2;
volatile unsigned int pulses_1;
volatile unsigned int pulses_2;
unsigned long timeold;
#define HOLES_DISC 20

#define enA 10
#define enB 11
#define motorA1 6 //IN1
#define motorA2 7 //IN2
#define motorB1 8 //IN3 
#define motorB2 9 //IN4

float k_p = 0.7;
float L = 12.3;
float err_1, err_2, u_1, u_2;
float rpm_setpoint = 200;
float pwm_min = 100;
float pwm_max = 255;
float S_1, S_2, S, teta;
void counter_1() {
  pulses_1++;
}

void counter_2() {
  pulses_2++;
}

void readEncoder() {
  if (millis() - timeold >= 1000) {
    detachInterrupt(digitalPinToInterrupt(encoder_pin_1));
    detachInterrupt(digitalPinToInterrupt(encoder_pin_2));

    S_1 = (pulses_1 / HOLES_DISC) * 3.14 * 6.5;
    S_2 = (pulses_2 / HOLES_DISC) * 3.14 * 6.5;
    S = S + (S_1 + S_2) / 2;
    teta = teta + (S_1 - S_2) / L ;

    timeold = millis();
    pulses_1 = 0;
    pulses_2 = 0;
    Serial.print("S = ");
    Serial.print(S);
    Serial.print(" teta = ");
    Serial.print(teta);
    Serial.println(" ");
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
  rpm_1 = 0;
  rpm_2 = 0;
  timeold = 0;

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  analogWrite(enA, 90);
  analogWrite(enB, 90);

  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);

  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);

}
void loop()
{
  readEncoder();

}
