# Deligreencs_bms
ROS node and publish for Deligreencs Smart BMS

# How to install from source
change name of catkin_ws to your name space

     $ cd catkin_ws/src
     $ git clone https://github.com/jarubank/deligreencs_bms.git
     $ cd deligreencs_bms/script
     $ sudo chmod +x * 
     $ cd 
     $ cd catkin_ws
     $ catkin_make

# Configure
In its default configuration, deligreencs_bms expects a launch file `launch/deligreencs_bms.launch`
* `bms_port` USB port to use 
* `/battery_state` publish topic

# Launch
`$ roslaunch deligreencs_bms deligreencs_bms.launch`
