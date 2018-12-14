# ME495 Embedded Systems Final Project

#### *Virtual Pong with Sawyer*
#### *Group 5: Ethan Park, Victor Ozoh, Petras Swissler, Andrew Thompson, Evan Li*

![group photo](groupphoto.jpg)

### Overview

The purpose of this project was to develop an interactive game of Pong on Sawyer. This system includes ultrasonic distance sensors which detect the location of the two pong paddles (the users' hands). These locations are translated to our 'game coordinates' and subsequently implemented in our game logic to determine the trajectory of the ball. The ball, the rectangular game area, and the ball's trajectory are represented in Sawyer's world coordinate frame, and the arm will trace out these trajectories in response to the gameplay.

### Video Demo:
[Link to demo video](https://youtu.be/s9HeBjx-4tQ)

### Arm Control

##### armcontrolT node
Description: [armcontrolT.py](src/armcontrolT.py) is responsible for publishing the location of Sawyer's end-effector in the space frame and for subscribing to twists. Using these twists, the node also computes and sets Sawyer's joint velocities.

Subscribes to:
* `/pongvelocity` : this topic message takes the form of a `geometry_msgs/Twist`, with three linear velocities and three angular velocities.

Publishes to:
* `/endpoint_Pose` : this topic message takes the form of a `geometry_msgs/Pose`, with coordinate information in Cartesian position (x,y,z) as well as quaternion orientation (x,y,z,w).

The Twist and Pose messages used are both with respect to Sawyer's world frame. We use the `/intera_interface/Limb` class to store current joint angles of Sawyer (via `limb.joint_angles()`) and to set calculated joint velocities (via `limb.set_joint_velocities()`).

 The joint velocities are calculated by using `mr.Adjoint()` and `mr.JacobianSpace()` from the Modern Robotics Code Library (mr) as well as `np.linalg.pinv()` from NumPy (np). The pseudoinverse of the space Jacobian was dot multiplied with our end-effector Twist to find updated joint velocities.

 We use the home configuration matrix and world frame screws from [sawyer_MR_description.py](\src\sawyer_MR_description.py) for computing the Jacobian.

### Pong logic

##### pong_MASTER node

Description: [pong_MASTER.py](src/pong_MASTER.py) is responsible for detecting impacts, calculating and publishing updated Twists, translating hand positions and ball position into the 'game space', and generating the GUI to illustrate the current game. It also includes modifiable parameters for ball_velocity and paddle_size (effectively allowing the user to choose their difficulty).

Subscribes to:
* `\endpoint_Pose` : see above.
* `\hand_positions` : this topic message takes the form of a custom `measured_distances` message, which consists of two int32s representing the right and left hand distances in millimeters.

Publishes to:
* `\pongvelocity` : see above.

The script begins by setting the arm to the game start position before 'serving' the ball with random x and y velocities. These velocities are normalized in order to maintain the ball's speed according to the configurable parameter mentioned above. We set the game boundary and use flags to keep track of contact between the ball and said boundaries. There is specific logic in place to prevent multiple impact updates during overshot (i.e., when the arm goes slightly past a boundary, caused by accumulated drift in the robotic arm).

If the ball position is greater than or equal to either of the left or right boundaries (entering the goal zones), and are not reflected by the players' paddles, a score is counted and the arm resets to its start position before beginning the next round. The maximum score is likewise configurable.

The GUI is illustrated by using the pyfiglet Python module, with relevant classes defined in [pong_classes.py](\src\pong_classes.py) and helper functions defined in [pong_plot.py](\src\pong_plot.py).

Support functions for resetting the hand to the default serving position are included in [hand_interface.py](\src\hand_interface.py).

### (Player) Hand Sensors

Sensing the location of the players' hands makes use of two main files: the Arduino portion (senseDistance.ino) and the ROS portion (sense_hands node)

##### senseDistance.ino

Description: [senseDistance.ino](https://github.com/victorozoh/ME495_Embedded_Systems_Final_Project/blob/master/Arduino/SenseDistance/senseDistance.ino) is an Arduino sketch responsible for sensing the positions of the player's hands using two [hcsr04](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf) distance sensors. 
These sensors use sonar in order to achieve these measurements. 
The sketch applies a simple time-averaging filter to smooth out measurement noise .

The code relies on an existing Arduino library containing a [.h](https://github.com/victorozoh/ME495_Embedded_Systems_Final_Project/blob/master/Arduino/SenseDistance/hcsr04.h) file and a [.cpp](https://github.com/victorozoh/ME495_Embedded_Systems_Final_Project/blob/master/Arduino/SenseDistance/hcsr04.cpp) file.
This library was created by gitHub user  [jeremylindsayni](https://github.com/jeremylindsayni/Bifrost.Arduino.Sensors.HCSR04).
The only portion of this code that we modified was the echo listening portion, in order to allow for time-out exceptions to occur. 
In the event that a time-out exception occurs, the sketch will illuminate a red LED to indicate to the player that it has lost track of their hand position. 
In this case, the previous position is re-used.

The sketch then reports this data over a serial communication port using a buad rate of 115200.
This measurement - report process occurrs indefinately.

##### sense_hands node

Description: [sense_hands.py](https://github.com/victorozoh/ME495_Embedded_Systems_Final_Project/blob/master/src/sense_hands.py) is responsible for recieving the serial communications from the Arduino, interpreting that data, and finally publishing that data for use in other ROS nodes

Monitors:
* The first active serial port: The node scans through available serial data streams and selects the first one it sees (this has the opportunity to cause issues for systems with more serial connections, but our laptops only ever had one serial device plugged in).
This serial port is assumed to have a baud rate of 115200, and take the form of "leftHandPosition,rightHandPosition\r\n".
Try and except blocks are used to ensure any issues with the serial port or recieved data do not crash the node.

Publishes to:
* `/hand_positions` : this topic message takes the form of a [`sawyer_pong.msg/measured_distances`](https://github.com/victorozoh/ME495_Embedded_Systems_Final_Project/blob/master/msg/measured_distances.msg) message, which is a custom message written for this project.
This message takes the form of two int32s: left_distance and right_distance.

### How to run:

We consolidated our nodes into [pongTest.launch](\launch\pongTest.launch).
From the Sawyer workspace containing this package, run:

`roslaunch sawyer_pong pongTest.launch`

Configurable parameters can be edited in the launch file.


