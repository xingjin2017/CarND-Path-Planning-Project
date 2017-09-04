# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Project Reflection

In order to generate a path that the car can safely drive on based on its environment, the following steps are taken.

### Assess the current environment

This includes the car's current position, speed, yaw, and other cars on the road through the sensor fusion data. There is a dedicated PathPlanner class that records and handles all these, and the whole path planning process. 

The car's current position is recorded using the MyCar class and gets updated to the PathPlanner using PathPlanner::resetMyCar, for every time interval. The SensedCar class is used to record the individual cars on the road. The vector of sensorFusion gets translate to the vector of SensedCars for easier handling.

### Decide on lane change or not

In the PathPlanner::planWaypoints function, the assessment of the current situation is made and a target rudimentary path is selected. This inculdes lane assesment, whether it is desirable to stay in the current lane (PathPlanner::scoreStaySameLane), if there is a car in front that is close and driving slow, it maybe desirable to switch to another lane. Switching to left lane and right lane are assessed in the functions PathPlanner::scoreChangeToLeftLane and PathPlanner::scoreChangeToRightLane. Lane changing depends on if there is a safe distance between the ego car and the cars in front and behind in the target lane. There is a minimum required distance before switching is desirable. It also depends on thoese cars' speed. All these are put together in the PathPlanner::decideOnLane function, which decides on which target lane to use for the planning. The function also returns the nearest front car in the target lane to help determine the target speed. Once the target lane and the reference speed are known, based on the maximum allowed acceleration (gMaxAcceleration), a target speed for the end of planning period is calculated. Once we know all these, assume the end acceleration is 0.0, given the long (250) planning steps - about 5 seconds.

### Jerk minimization path

Then in PathPlanner::planWaypoints, the jerk minimization functions are used in both the S direction and the D direction to find out the desired trajectories based on time. The corresponding function is PathPlanner::jerkMinimization. The results are S/D trajectories keyed on time. The jerk minimization formula is from the class lectures.

### Generate x/y values using spline with smooth transition

Once we have the target trajectories from the PathPlanner::planWaypoints function, the PathPlanner::generateSmoothTrajectory function is called to generate the target next_x_vals and next_y_vals, which are used as the input to the simulator for the actual driving. Here, in order to have smoothing driving, a fixed number of x/y points from the previous path are used (gKeepPreviousSteps 40 in the code). The earlier trajectories generated in PathPlanner::planWaypoints are based on time, here in the generateSmoothTrajectory function, we sample a few sparse points at the second half of the planning time period, and use these points plus the first two points from the previous path, to fit splines for both X and Y. Once we have the xSpline, ySpline that are keyed on time, we get the corresponding x and y values for each time step (interval of 0.02 second) and add them to the next_x_vals and next_yvals, which already have certain number (gKeepPreviousSteps) of x/y values from the previous path.

This is it. A bit extra tuning to make sure the maximum speed, acceleration, etc. are not violated. The car is able to drive more than one complete loop repeatedly!

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
