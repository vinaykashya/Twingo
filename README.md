# Team Twingo - CarND Capstone
This project was the culmination of 9 months of accelerated learning in the self-driving space. The goal of the project was to implement and integrate everything we have learned over the the last 3 terms to be tested on in a real-world scenario. The skeleton software stack for the self-driving car was provided in ROS with autoware and dbw integrations. We had to implement a planning module (*waypoint updater*), a control module (*twist_control*) and a perception module (*traffic light detection*).

## Implementation details
### Planning Module (Waypoint loader)
Here we obtain the pose of the car with respect to the waypoints from the simulator or the test track. We figure out the closest waypoint to the car. The next 200 waypoints from the track are then obtained and published to the controller node. If a traffic light is detected, the waypoints upto the light will be given decelerating velocities and the waypoints following the traffic lights are given a velocity of 0. This node also decides if it is safe to stop given the distance to the trafic light based on the current velocity of the car.

### Controller Module (Twist Controller)
The waypoints loaded into the controller module, and the waypoint loader issues twist commands based onthe twist controller. In the twist controller we have a PID controller for the throttle/brake action based on the waypoint list. The steering in controlled by a yaw controller based on the angular and linear velocity. The brake, throttle and steering command are then published to the dbw node to control the car.

### Perception Module (Traffic light detection and classification)
This node publishes the waypoint of the closest treaffic light that is red when it is detected. The traffic light detection a optimised detector-classifer network. For the detector network we use the pre-trained SSD-Mobilenet architecture which is pretrained on the COCO dataset ( as one of the class labels in the dataset is traffic lights) followed by a low copmute and storage version of alexnet called squeezenet. The single shot detector architecture with mobilenet was chosen because of the accuracy and performance benefits as they are designed to run on low power hardware on the clients. Squeezenet was selected as it dramatically reduces the size for parameter storage and hence all the weights can fit on the RAM simultaneously. It was trained on some TL images from COCO and a few captured from the simulator.

## Team members
* Vinay Kashyap (Team lead) - vpkashya@asu.edu
* Tom Zheng - sotomso@hotmail.com
* Anh Le - atle2@uci.edu
* Daedeepya Yendluri - dyendlur@gmail.com
* Marcos Ahuizotl Fragoso Iñiguez - mafragosoi@gmail.com


# Original Repo Instructions 
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
