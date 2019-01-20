# Description
This repo contains a ROS driver to control a TimMotion TL5 linear actuator. There's a custom board between our driver and the lifting column. 

## Arduino ROS interface
### File
* wm_lifting_column_arduino.ino : code on the arduino that control the actuator
### Topics provided
* Subscriber
    * /column/cmd (std_msgs/Int32)
        * command can be : -1 (down), 0 (stop), 1 (up)
    * /column/setMax (std_msgs/Empty)
        * will set the actual position to 0
    * /column/setZero (std_msgs/Empty)
        * will set the actual position to 2387
* Publisher
    * /column/position (std_msgs/Int32)
        * relative position from the hall sensor
    * /column/state (std_msgs/Int32)
        * publish the actuator state : -1 (down), 0 (stop), 1 (up)
        
## ROS jointState interface
### File
* src/column_state_publisher.py : interface between the arduino and jointState
### Topics
* Subscriber
    * /column/position (std_msgs/Int32)
        * receive the relative position to calculate the position in meters
* Publisher
    * /jointState (sensor_msgs/JointState)
        * publish the column position relative to it's parent
