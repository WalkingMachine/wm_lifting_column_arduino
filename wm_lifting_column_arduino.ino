#include <ros.h>
#include <std_msgs/Int32.h>
#include <Encoder.h>
#include <EEPROM.h>

#define UP 1
#define DOWN 0
#define START 1
#define STOP 0

/// Constantvalues
const int ROMADDR = 500;
const int pin_Btn_UP = 6;
const int pin_Btn_DN = 7;
const int pin_DIR = 4;
const int pin_PWM = 5;
const int max_value = 2230;
const int pin_hallSensorA = 2;
const int pin_hallSensorB = 3;

// Define the EEPROM storage type
union EEPROMSpace {
  long value;
  byte table[4];
};


// Encoder creation
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(pin_hallSensorA, pin_hallSensorB);
//   avoid using pins with LEDs attached


// Position encoding variables
EEPROMSpace oldPosition;
long inactivityCounter = 0;
long lastMllis = 0;

// Buttons config
bool oldButtonUPState = false;
bool oldButtonDOWNState = false;

// ROS callbacks
void cmdCallback(const std_msgs::Int32& cmd) {
  // Send commanded speed and direction to the drive
  if(cmd.data  < 0){
    digitalWrite(pin_DIR, DOWN);
  } else if(cmd.data > 0){
    digitalWrite(pin_DIR, UP);
  }
  analogWrite(pin_PWM, abs(cmd.data));
}


void setPosition(const std_msgs::Int32& pos) {
  myEnc.write(pos.data);
}


// ROS objects
ros::NodeHandle nh;
volatile std_msgs::Int32 positionMessage;
ros::Publisher positionPub("column/position", &positionMessage);
ros::Subscriber<std_msgs::Int32> subCmd("/column/cmd", cmdCallback );
ros::Subscriber<std_msgs::Int32> subSetPosition("/column/set_position", setPosition );



// Start function
void setup() {

  Serial.begin(57600);
  nh.initNode();
  nh.advertise(positionPub);
  nh.subscribe(subCmd);
  nh.subscribe(subSetPosition);

  // Configure pins
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_DIR, OUTPUT);
  digitalWrite(pin_PWM, STOP);
  digitalWrite(pin_DIR, UP);
  pinMode(pin_Btn_UP, INPUT_PULLUP);
  pinMode(pin_Btn_DN, INPUT_PULLUP);

  // Load previous position from EEPROM
  oldPosition.table[0] = EEPROM.read(ROMADDR+0);
  oldPosition.table[1] = EEPROM.read(ROMADDR+1);
  oldPosition.table[2] = EEPROM.read(ROMADDR+2);
  oldPosition.table[3] = EEPROM.read(ROMADDR+3);


  oldPosition.value = -999;
}


// Loop function
void loop() {

  // Enforce rate
  long currentMilis = millis();
  if(currentMilis - lastMllis > 50){
  
    // Manage override buttons

  
    if(digitalRead(pin_Btn_UP) == LOW) {
      oldButtonUPState = true;
      digitalWrite(pin_DIR, UP);
      digitalWrite(pin_PWM, START);
    } else if (oldButtonUPState) {
      oldButtonUPState = false;
      digitalWrite(pin_DIR, DOWN);
      digitalWrite(pin_PWM, STOP);
    }

    else if(digitalRead(pin_Btn_DN) == LOW) {
      oldButtonDOWNState = true;
      digitalWrite(pin_DIR, DOWN);
      digitalWrite(pin_PWM, START);
    } else if (oldButtonDOWNState) {
      oldButtonDOWNState = false;
      digitalWrite(pin_DIR, DOWN);
      digitalWrite(pin_PWM, STOP);
    }


    // Get the latest position
    positionMessage.data = myEnc.read();

    // Check if the collumn is moving and save the position in EEPROM once if not.
    if (positionMessage.data == oldPosition.value){
      inactivityCounter ++;
      if (inactivityCounter == 20){  // Counter to add some tolerance.
        EEPROM.put(ROMADDR+0,oldPosition.table[0]);
        EEPROM.put(ROMADDR+1,oldPosition.table[1]);
        EEPROM.put(ROMADDR+2,oldPosition.table[2]);
        EEPROM.put(ROMADDR+3,oldPosition.table[3]);
      }
    } else {
      inactivityCounter = 0;
    }
    
    // Update ROS
    nh.spinOnce();
    positionPub.publish( &positionMessage );
    lastMllis = currentMilis;
    oldPosition.value = positionMessage.data;
  }
}
