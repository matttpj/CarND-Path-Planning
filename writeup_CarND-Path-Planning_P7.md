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

#### Submission includes all files required to run the driving simulator

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
The code for Generating Paths is included in file **_main.cpp_**  from line 54 onwards.
