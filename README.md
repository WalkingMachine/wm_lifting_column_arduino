# Description
This repo contains a ROS driver to control a TimMotion TL5 linear actuator. There's a custom board between our driver and the lifting column. 

# Dependencies
* [Encoder.h](https://www.pjrc.com/teensy/td_libs_Encoder.html)

# Topics provided
* Subscriber
    * /column/cmd (std_msgs/Int32)
        * command can range from -255 (down) to 255 (up).
    * /column/set_position (std_msgs/Int32)
        * Allow to set the actual position
* Publisher
    * /column/position (std_msgs/Int32)
        * relative position from the hall sensor
        
