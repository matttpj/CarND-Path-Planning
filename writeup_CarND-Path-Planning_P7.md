# **Highway Driving**

## Writeup by Matthew Jones

### Project: CarND-Path-Planning

---

**Path Planning Project**

The major steps taken to complete this project included:
* Reviewing **Highway Driving** project goals, project instructions and coding tips in the [Q&A video](https://youtu.be/7sI3VHFPP0w)
* Getting familiar with the commands needed in the Udacity workspace to compile project code and launch the driving simulator
* Using code from the Getting Started and More Complex Paths lesson segments to get my car moving
* Writing my code in __main.cpp__ as recommended by the Q&A video to enable my car to drive around the track  
* Adapting my code to ensure that my car does not violate any of the requirements defined in the [project rubric](https://review.udacity.com/#!/rubrics/1971/view)
* Explaining in detail the code in __main.cpp__ for generating paths **_[see below](#Reflection)_**


[//]: # (Image References) **TODO: update screenshot image**  

__Screenshot of the driving simulator__  
<img src="./output_images/video_output_run2.jpg" width=100% height=100%>

---
### Writeup / README
Here is a link to my [project code](https://github.com/matttpj/CarND-Path-Planning)  

---
### Files Submitted

#### Submission includes all files required to run my car in the driving simulator

Key files are in source directory __src/*__:
 * Connects to the simulator and initiates path planning: __main.cpp__   
 * Calls the spline library for path planning: __spline.h__
 * Manipulate waypoints between Map (cartesian) and Frenet coordinate systems:  __helpers.h__
 * Manipulate JSON format data structures and messages:  __json.hpp__
 * C++ template library for linear algebra including use of matrices, vectors, numerical solvers etc..:  __Eigen-3.3/*__

---
### Compilation
The above source code compiles successfully in the Udacity workspace.

---
### Valid trajectories
| Criteria       		|     Specification	        					|    Status  |
|:---------------------:|:-------------------------------:|:--------:|
| The car is able to drive at least 4.32 miles without incident	| The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.	|  PASS |
| The car drives according to the speed limit | The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic. | PASS |
| Max Acceleration and Jerk are not Exceeded | The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3. | PASS |
| Car does not have collisions | The car must not come into contact with any of the other cars on the road. | PASS |
| The car stays in its lane, except for the time between changing lanes. | The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road. | PASS |
| The car is able to change lanes | The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic. | PASS |


---
### Reflection
The code for Generating Paths is included in file **_main.cpp_**  from line 54 onwards. An explanation of how it works is outlined below.

__Localisation >>__ The simulator provides information about position of my car on the track, including: x, y, s, d, theta, yaw, speed

__Previous Path >>__ The simulator provides previous path data sent to the path planner

__Sensor Fusion >>__ The simulator provides data about positions of other cars on my side of the track, including: id, x, y, vx, vy, s, d

__Spline >>__  Uses an external library to create a series of waypoints for my car to drive through

__Json >>__ Constructs a message in Json format with a series of new X and Y values to tell my car which path to follow

__Web Sockets >>__ Sends message to the simulator

__Detailed Steps__
 *  __Check position of other cars >>__ Use sensor fusion data to determine position of other cars that are in close proximity  __TO DO__
 *  __Change lane >>__ Change the lane of my car if safe to do so  __TO DO__
 * __Initial reference points >>__ For defining the path for my car to follow, if previous path is almost empty (eg. < 2 points because my car is near the start) use my car's actual position as reference points; or use my car's previous path's last 2 end points as reference points
 *  __Define next waypoints >>__ Add 3x additional waypoints evenly spaced at 30m intervals ahead of the starting reference in s,d Frenet coordinates and then convert to x,y cartesian coordinates
 *  __Create spline >>__ Use above waypoints to create a spline of points to follow
 *  __Path planner >>__ Fill up path planner of future points to follow  __TO DO__
