#include <ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

#define UP 1
#define DOWN 0
#define topic_DOWN -1
#define topic_UP 1
#define topic_STOP 0 
#define START 1
#define STOP 0


const int pin_hallSensorA = 2;
const int pin_hallSensorB = 3;
const int pin_Btn_UP = 6;
const int pin_Btn_DN = 7;
const int pin_DIR = 4;
const int pin_PWM = 5;
const int max_value = 2387;

volatile long position_count = 0;

unsigned long lastMicros_int   = 0;
const int pulseTime_minReal    = 12400;
const int pulseTime_minTolered =  9500;

std_msgs::Int32 actuator_state; // -1 down, 0 stop, 1 up
std_msgs::Int32 pwm_value; // 0 , 100 to 255 , -100 to -255


ros::NodeHandle nh;
std_msgs::Int32 position_value;

ros::Publisher state_pub("column/state", &actuator_state);
ros::Publisher position_pub("column/position", &position_value); // debugging purpose



void set_state(const std_msgs::Int32& value)
{
  if(value.data  <= -1)
    actuator_state.data = topic_DOWN;
  else if(value.data == 0)
    actuator_state.data = topic_STOP;
  else if(value.data >= 1)
    actuator_state.data = topic_UP;
  pwm_value.data = abs(value.data);
  state_pub.publish( &actuator_state );
}

void set_zero()
{
  position_count = 0;
  position_value.data = position_count;
}
void set_max()
{
  position_count = max_value;
  position_value.data = position_count;

}

ros::Subscriber<std_msgs::Int32> sub_cmd("/column/cmd", set_state );
ros::Subscriber<std_msgs::Empty> sub_zero("/column/setZero", set_zero );
ros::Subscriber<std_msgs::Empty> sub_max("/column/setMax", set_max );

void setup() 
{
  nh.initNode();
  nh.advertise(position_pub);
  nh.advertise(state_pub);
  nh.subscribe(sub_cmd);
  nh.subscribe(sub_zero);
  nh.subscribe(sub_max);

  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_DIR, OUTPUT);
  digitalWrite(pin_PWM, STOP);
  digitalWrite(pin_DIR, UP);
  pinMode(pin_hallSensorA, INPUT);
  pinMode(pin_hallSensorB, INPUT);
  pinMode(pin_Btn_UP, INPUT_PULLUP);
  pinMode(pin_Btn_DN, INPUT_PULLUP);
  Serial.begin(57600);
  pwm_value.data = 0 ;
  attachInterrupt(digitalPinToInterrupt(pin_hallSensorA), callback_pin2, FALLING);
  
}


void loop() 
{
  
  if(digitalRead(pin_Btn_UP) == LOW)
  {
    actuator_state.data = topic_STOP; // override topic if btn is used
    digitalWrite(pin_DIR, UP);
    digitalWrite(pin_PWM, START);
  }
  else if(digitalRead(pin_Btn_DN) == LOW)
  {
    actuator_state.data = topic_STOP; // override topic if btn is used
    digitalWrite(pin_DIR, DOWN);
    digitalWrite(pin_PWM, START);
  }
  else if(actuator_state.data == topic_UP)
  {
    actuator_state.data = topic_UP; 
    digitalWrite(pin_DIR, UP);
    analogWrite(pin_PWM, pwm_value.data);
  }
  else if(actuator_state.data == topic_DOWN)
  {
    actuator_state.data = topic_DOWN; 
    digitalWrite(pin_DIR, DOWN);
    analogWrite(pin_PWM, pwm_value.data);
  }
  else
  {
    digitalWrite(pin_DIR, DOWN);
    digitalWrite(pin_PWM, STOP);
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
