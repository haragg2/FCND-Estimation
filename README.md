# FCND-P4-3D-Estimation
Udacity Flying Car Nanodegree - Project 4 - 3D Estimation

# Project description
In this project, we implement a 3D estimation technique for quadrotors using Extended Kalman Filter. This is build upon Project-3 [3D PID Controller](https://github.com/haragg2/FCND-Controls). All the maths used in the project can be found in [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/). This estimator uses a IMU, GPS and a magnetometer for the measurements of the state including position, velocity and attitude.

## Setup ##

This project will continue to use the C++ development environment you set up in the Controls C++ project.

 1. Clone the repository
 ```
 git clone https://github.com/haragg2/FCND-Estimation.git
 ```
2. Build

- Create a new directory for the build files:
```sh
cd FCND-Estimation-CPP
mkdir build
```

- Navigate to the build directory and run `cmake` and then compile and build the code:

```sh
cd build
cmake ..
make
```

3. You should now be able to run the simulator with `./CPPEstSim` and you should see a single quadcopter, falling down.

### Project Structure ###

For this project, you will be interacting with a few more files than before.

 - The Extended Kalman Filter for state estimation is implemented in `QuadEstimatorEKF.cpp`

 - Parameters for tuning the EKF are in the parameter file `QuadEstimatorEKF.txt`

 - The cascade PID control implemented in `QuadControl.cpp`
 
 - Parameters for the PID control code is in `QuadControlParams.txt`