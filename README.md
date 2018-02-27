# Short description
This ROS package has been developed to collect data from 1) a Hokuyo lidar (20 Hz), 2) a Phidgets IMU (250 Hz), 3) a camera (~20 fps), 4) 
a GPS sensor (1 Hz), and 5) a flag button. In the [DIV project](http://divproject.eu), this data logger was used to collect data on road.
A pedestrian wore the equipment and overtaking manoeuvres of vehicles were collected.
Notice that the datalogger package is based on Kinetic ROS' distribution. 

# Installation
This div_datalogger package is dependent on the following ROS packages:
  I) urg_node http://wiki.ros.org/urg_nodeHoduyo for the Hukoyo lidar. Notice that it is necessary to edit the file     'lidar.launch'in order to set specific values for the scan-angle of a different sensor. This package is related to the rangefinder Hokuyo UXM-30LAH-EWA.
  II) phidgets_drivers http://wiki.ros.org/phidgets_drivers?distro=kinetic for the IMU
  III) cv_camera http://wiki.ros.org/cv_camera for the Creative Live! Cam Connect. Edit the cv_camera.launch file for a different camera resolution (e.g. 1280x720)
  IV) nmea_navsat_driver http://wiki.ros.org/nmea_navsat_driver to parse NMEA strings and publish a GPS message.
  V) rosbridge_suite http://wiki.ros.org/rosbridge_suite for a WebSocket interface to rosbridge.

# How to use it
Together with the aforementioned list of sensors, the hardware required has to be: 1)Backpack to arrange the proper power supply for the lidar. The backpack has also to fullfill the role of hosting a single board computer (e.g. raspberry pi or arduino with available WiFi hotspot). 2) Lidar support, useful to fasten the lidar to pedestrian's waist 3) Button: he equipped pedestrian can use the button (see 3D stl model for printing and reproducing it) once a relevant event is noticed. 
It is worth mentioning that the centerpoint of the IMU should be positioned over the lidar's beam origin axis.

# Web interface to control the data logger
Please follow [this link](https://github.com/roaduserinteraction/div_datalogger_webapp) to download the web interface to control the data logger.

# Issues
If you find any issue in this ROS package, please report it using Github's issue reporting tool.

# Authors
Gabriele Panero, Alexander Rasch, and Christian-Nils Åkerberg Boda
