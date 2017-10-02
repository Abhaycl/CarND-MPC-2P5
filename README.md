# MPC Controller Project Starter Code

A MPC (Model Predictive Control) the goal of this project is to drive the car around the track in Udacity-provided simulator, which communicates telemetry and track waypoint data via websocket, by sending steering and acceleration commands back to the simulator. Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

<!--more-->

[//]: # (Image References)

[image1]: /build/result.jpg "Sample final score"
[image2]: /build/result1.jpg "Sample final score"

#### How to run the program

```sh
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./mpc
6. and run the simulator and select Project 5: MPC Controller
```

The summary of the files and folders int repo is provided in the table below:

| File/Folder               | Definition                                                                                  |
| :------------------------ | :------------------------------------------------------------------------------------------ |
| src/json.hpp              | Various definitions.                                                                        |
| src/MPC.cpp               | Initializes variables, calculation of different parameters, using the angle of rotation     |
|                           | and speed.                                                                                  |
| src/MPC.h                 | Definition of the package of mpc.                                                           |
| src/main.cpp              | Has several functions within main(), communicates with the Term 2 Simulator receiving       |
|                           | data measurements, calls a function to run the MPC (Model Predictive Control), these        |
|                           | all handle the uWebsocketIO communication between the simulator and it's self.              |
|                           |                                                                                             |
| src                       | Folder where are all the source files of the project.                                       |
| build                     | Folder where the project executable has been compiled.                                      |
|                           |                                                                                             |


---

# The Model

A simple Kinematic model (ignores tire forces, gravity, mass, etc) was used for the Controller. It's essential to know parameters of the vehicle (such as law of response on the throttle, geometry of the car, drag model, tires properties, etc) to construct a reasonable dynamic model but such parameters are not derectly accessible from provided materials for the project. Position (x,y), heading (ψ) and velocity (v) form the vehicle state vector:

State: [x, y, ψ, v]

There are two actuators. Stearing angle (δ) is the first one, it should be in range [-25, 25] deg. For simplicity the throttle and brake represented as a singular actuator (a), with negative values signifying braking and positive values signifying acceleration. It should be in range [-1, 1]. Actuators: [δ, a]

The kinematic model can predict the state on the next time step by taking into account the current state and actuators as follows:

```sh
     x​t + 1 ​​= x​t ​​+ v​t ​​∗ cos(ψ​t​​) ∗ dt

     y​t + 1 ​​= y​t ​​+ v​t ​​∗ sin(ψ​t​​) ∗ dt

     ψ​t + 1 ​​= ψ​t ​​+ ​v​t/L​f ​​​​​​​∗ δ​t ​​∗ dt

     v​t + 1 ​​= v​t ​​+ a​t ​​∗ dt
```

Where Lf measures the distance between the front of the vehicle and its center of gravity. The parameter was provided by Udacity.

Errors: cross track error (cte) and orientation error (eψ) were used to build the cost function for the MPC. They could be updated on a new time step using the following equations:

```sh
     cte​t + 1 ​​= cte​t ​​+ v​t ​​∗ sin(eψ​t​​) ∗ dt

     eψ​t + 1 ​​= eψ​t ​​+ vt/​L​f ​​​​∗ δ​t ​​∗ dt
```

# Timestep Length and Elapsed Duration (N & dt)
While the PID controller is easy to implement, but it is not so easy to tune.

![Final score][image1]

![Final score][image2]

---

#### Discussion

Parameterization of the tuning could be improved to obtain better results.
