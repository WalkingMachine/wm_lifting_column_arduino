#include <ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

#define UP 1
#define DOWN -1
#define STOP 0

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

std_msgs::Int32 actuator_state; // -1 down, 0 stop, 1 up


ros::NodeHandle nh;
std_msgs::Int32 position_value;

ros::Publisher state_pub("column/state", &actuator_state);
ros::Publisher position_pub("column/position", &position_value); // debugging purpose

void set_state(const std_msgs::Int32& value)
{
  if(value.data == -1)
    actuator_state.data = -1;
  else if(value.data == 0)
    actuator_state.data = 0;
  else if(value.data == 1)
    actuator_state.data = 1;
  state_pub.publish( &actuator_state );
}

void set_zero(const std_msgs::Int32& value)
{
  position_count = 0;
}

ros::Subscriber<std_msgs::Int32> sub_cmd("/column/cmd", set_state );
ros::Subscriber<std_msgs::Empty> sub_zero("/column/setZero", set_zero );


void setup() 
{
  nh.initNode();
  nh.advertise(position_pub);
  nh.advertise(state_pub);
  nh.subscribe(sub_cmd);
  nh.subscribe(sub_zero);

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
  if(digitalRead(pin_Btn_UP) == LOW || actuator_state.data == UP)
  {
    digitalWrite(pin_m1, HIGH);
    digitalWrite(pin_m2, LOW);
  }
  else if(digitalRead(pin_Btn_DN) == LOW  || actuator_state.data == DOWN)
  {
    digitalWrite(pin_m1, LOW);
    digitalWrite(pin_m2, HIGH);
  }
  else
  {
    digitalWrite(pin_m1, LOW);
    digitalWrite(pin_m2, LOW);
  }
  nh.spinOnce();
  position_pub.publish( &position_value );
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
    position_value.data = position_count;
    //position_pub.publish( &position_value );
  }  
  lastMicros_int = micros();
}
