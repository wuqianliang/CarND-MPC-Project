# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Overview
In this C++ project, I implemented a Model Predictive Control Controller(MPC) in order to drive a simulated car around a virtual track. In order keep car on track, I fit polynomial to the waypoints,calculate initial cross track error and orientation error values,define the components of the cost function (state, actuators, etc),set the constraints for optimization.And I also acount for the 100ms latency for the MPC calculation.

## Project steps
1. Transform the points into the vehicle's coordinates
2. Fitting a line based on road waypoints and evaluating the current state like cte and epsi based on that polynomial line.
3. Implementing the MPC calculation, including setting variables, optimization costs and constraints.
4. Calculating actuator values delta and a from the MPC::solve from the current state.
5. In order to accounte for latency, I caculate the future state in 100ms to replace the actual state.
6. Calculating steering angle & throttle/brake.
7. Setting timestep length and duration and tuning them.

## Rubic
### The model description
The model starts out through the following values from the simulator:
* ptsx (x-position of waypoints in global coordinates)
* ptsy (y-position of waypoints in global coordinates)
* px (current x-position of the vehicle's position in global coordinates)
* py (current y-position of the vehicle's position in global coordinates)
* psi (current orientation angle of the vehicle, converted from the simulator's format to that expected in mathematical formulas)
* v (current velocity of the vehicle)
* delta (current steering angle of the car,not the actual orientation of the car in the simulator `psi`)
* a (current throttle)


### Polynomial Fitting and MPC Preprocessing

First, calculate the relative coordinates of the navigation points relative to the car, and then map the relative coordinates of the calculated navigation points through the following companies to the coordinate system of the car's angle of view.
* `ptsx_car[i] = x * cos(-psi) - y * sin(-psi)`
* `ptsy_car[i] = x * sin(-psi) + y * cos(-psi)`
Then use the `polyfit()` function to caculate the a third-degree polynomial line on the transformed waipoints in order to show the vehicle should go.Use the `polyeval()` to calculates the cross track error `cte` and the orientation error `epsi`.

### Model Predictive Control with Latency
I add a step (line 137 ~ 144) to predict state after 100ms' latency, to replace current state in order to get the effect that when the actuator commands reach the simulator and when the simulator is executed, it has been used for 100 milliseconds, which is exactly the result of the initial state optimization of the replacement states.

In MPC::Solve of `MPC.cpp` , I set 
1. the number of model variables
2. the initial variable values
3. the upper and lower bound for the delta,a
4. Lower and upper limits for the constraints

and in the `FG_eval class` of `MPC.cpp` I set 
1. Weights for how "important" each cost is
2. add costs of (cte, epsi, velocity, actuator<delta,a>,  gap between sequential actuations) to `fg[0]`
3. then setup Model Constraints 
4. and then use the `CppAD::ipopt::solve<Dvector, FG_eval>()` function to solve the optimization problem and return the first actuator values, along with predicted x and y values to plot in the simulator.


### Timestep Length and Elapsed Duration (N & dt)
The higher N (but not very high like 15,20 ) may cause the polynomial fitting slower(failed) and arise the cost, the higher dt (like 0.2 or 0.3) may slow down the speed fo the car and somtimes may cause the optimization failed (i.e. the predicted future positions are off the track).And finaly the N=10 and dt=0.1 make the car run prefectly up to speed 100mph at some points in track.
And I think that three order spline fitting of waypoints may be a better way for fitting.

### The vehicle must successfully drive a lap around the track.
[Here](http://v.youku.com/v_show/id_XMzQ0MTY4ODEyMA==.html) is the demo video of my MPC implmenting.


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `cmake .`
3. Compile: `make`
4. Run it: `./mpc`.

## Reference

https://github.com/mvirgo/MPC-Project
