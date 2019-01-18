const int pin_hallSensorA = 2;
const int pin_hallSensorB = 3;
const int pin_m1 = 5;
const int pin_m2 = 6;
const int pin_Btn_UP = 8;
const int pin_Btn_DN = 9;

volatile long position_count = 0;

unsigned long lastMicros_int   = 0;
const int pulseTime_minReal    = 12400;
const int pulseTime_minTolered =  9500;

void setup() 
{
  pinMode(pin_m1, OUTPUT);
  pinMode(pin_m2, OUTPUT);
  digitalWrite(pin_m1, LOW);
  digitalWrite(pin_m2, HIGH);
  pinMode(pin_hallSensorA, INPUT);
  pinMode(pin_hallSensorB, INPUT);
  pinMode(pin_Btn_UP, INPUT_PULLUP);
  pinMode(pin_Btn_DN, INPUT_PULLUP);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(pin_hallSensorA), callback_pin2, FALLING);
}

int accu = 0;
int polaru = 1;
void loop() 
{
  delay(100);
  int mm = (float)position_count * 0.3;
  Serial.println(mm);

  accu = 0;
  if(digitalRead(pin_Btn_UP) == LOW)
  {
    digitalWrite(pin_m1, HIGH);
    digitalWrite(pin_m2, LOW);
  }
  else if(digitalRead(pin_Btn_DN) == LOW)
  {
    digitalWrite(pin_m1, LOW);
    digitalWrite(pin_m2, HIGH);
  }
  else
  {
    digitalWrite(pin_m1, LOW);
    digitalWrite(pin_m2, LOW);
  }
  
}

void callback_pin2()
{
  unsigned long diff = micros() - lastMicros_int;
  //Serial.println(diff);
  if(diff > pulseTime_minTolered)
  {
    if(digitalRead(pin_hallSensorB) == LOW)
    {
      position_count ++;
    }
    else
    {
      position_count --;
    }
  }
  lastMicros_int = micros();
}
