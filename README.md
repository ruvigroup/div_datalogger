# DIV-WP5 data logger pedestrian
This ROS package has been developed to collect data from multiple sensors like LiDAR or IMU. For the [DIV project](http://divproject.eu), this data logger was used in work package 5 (DIV-WP5) to collect field data. A pedestrian wore the equipment and overtaking manoeuvres of vehicles were collected as the pedestrian was walking on or next to the lane marking. To facilitate finding timestamps of overtakings in the post processing step, a flag button was added which indicates overtaking when pressed. 

The data logger is running entirely on a Raspberry Pi (RPi), version 3 B, to which all sensors are directly connected.

## Hardware
### Sensors
Here is an overview of the sensors connected to the RPi:

| Sensor    	| Product 			| Connection type | Sampling rate |
|---------------|-------------------------------|-----------------|---------------|
| LiDAR		| Hokuyo UXM-30LAH-EWA		| Ethernet        | 20 Hz         |
| IMU		| PhidgetSpatial 1044_0 	| USB             | 250 Hz        |
| GPS		| Globalsat BU-353S4		| USB		  | 1 Hz          |
| Camera	| Creative Live! Cam Sync HD	| USB		  | 15 fps        |
| Flag button	| -				| GPIO		  | 10 Hz         |

> **Note:** The frame rate of the camera was set to 30 fps but due to the processing load on the RPi, the resulting frame rate decreased to 15.

### Other gadgets
To have the correct time stamps with the data also when the RPi is not connected to internet, a real time clock (RTC) is added. We use the Adafruit DS1307 RTC.

In order to handle larger amount of recorded data, we use a standard USB pen drive (64 GB) which is always connected to the RPi.

### Power management
The LiDAR is powered from a 12 V battery. We use a Yuasa NP7-12 battery (12 V, 7000 mAh).

The RPi is powered via Micro USB from a standard power bank. We use a Deltaco power bank (5 V, 6000 mAh).

## Installation
### GPIO pin assignment
### ROS installation on Raspberry Pi
To install ROS kinetic on the RPi, follow [this guide](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi).
### WiringPi installation
### Package installation
This div_datalogger package is dependent on the following ROS packages:
- [**urg_node**](http://wiki.ros.org/urg_node) for the LiDAR. Note that it is necessary to edit the file `launch/lidar.launch` in order to set specific values for the scan-angle of a different sensor. This package is related to the rangefinder Hokuyo UXM-30LAH-EWA.
- [**phidgets_drivers**](http://wiki.ros.org/phidgets_drivers?distro=kinetic) for the IMU
- [**cv_camera**](http://wiki.ros.org/cv_camera) for the Creative Live! Cam Connect. Edit the `launch/cv_camera.launch file` for a different camera resolution (e.g. 1280x720)
- [**nmea_navsat_driver**](http://wiki.ros.org/nmea_navsat_driver) to parse NMEA strings and publish a GPS message.
- [**rosbridge_suite**](http://wiki.ros.org/rosbridge_suite) for a WebSocket interface to rosbridge.

To install the datalogger package, simply clone from Github and install for example via catkin:
```bash
$ cd ∽/catkin_ws/src
$ git clone https://github.com/ruvigroup/div_datalogger.git
$ cd ∽/catkin_ws
$ catkin_make
```

Before running the data logger, folders should be arranged according to lines 79-82 in `scripts/div_datalogger_node.py`:
```python
if len(os.listdir('/media/ubuntu/')) > 0:
	path_usb_drive = ('/media/ubuntu/' + os.listdir('/media/ubuntu/')\[0\])
else:
	path_usb_drive = '/home/ubuntu/bagfiles'
```
The script stores the bag file in the first device listed under `/media/ubuntu` (to where we mount the USB stick) and if no devices are listed then under `/home/ubuntu/bagfiles`

## How to use it
Together with the aforementioned list of sensors, the hardware required has to be:
- **Backpack** to arrange the proper power supply for the lidar. The backpack has also to fullfill the role of hosting a single board computer (e.g. raspberry pi or arduino with available WiFi hotspot). 
- **LiDAR support**, useful to fasten the lidar to pedestrian's waist
- **Flag button**, the equipped pedestrian can use the button (see 3D stl model for printing and reproducing it) once a relevant event is noticed. 
> **Note:** The center point of the IMU should be positioned over the LiDAR's beam origin axis.

Run the data logger using the provided shell script
```bash
$ ./startup.sh
```


### Logged topics
The following topics are recorded in bag files (bag file split every 1024 MB):
- */flagbutton_pressed*: 1 if pressed, 0 if not
- */tf*: transformation output
- */imu/data*: Madgwick filtered IMU data, includes orientation
- */imu/data_raw*: IMU raw data, without orientation
- */imu/mag*: IMU magnetometer readings
- */scan*: LiDAR scans
- */time_reference*: GPS reference time
- */fix*: GPS position
- */vel*: GPS velocity
- */cv_camera/image_raw/compressed*: Compressed image from the camera
- */cv_camera/camera_info*: Additional infos from camera
- */rosout*: ROS log output 

## Web interface to control the data logger
Please follow [this link](https://github.com/roaduserinteraction/div_datalogger_webapp) to download the web interface to control the data logger.

## Issues
If you find any issue in this ROS package, please report it using Github's issue reporting tool.

## Authors
Gabriele Panero, Alexander Rasch and Christian-Nils Åkerberg Boda
