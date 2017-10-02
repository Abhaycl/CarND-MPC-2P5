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

Generally speaking, it requires to set Kd and Ki to 0 and gradually increase Kp before the car runs with stable and consistent oscillations. This value of Kp and the oscillation period can be used to calculate optimal pid controller parameters by the method. Parameters was able to drive car around the track but with a lot of wobbling, that is why parameters were further tuned manually after several rounds of trial and error.

The same process was applied for different speed, so different PID parameters were found for different speed. The results were linearized in order to make the parameters automatically tune with the car speed variation.

While the PID controller is easy to implement, but it is not so easy to tune.

![Final score][image1]

![Final score][image2]

---

#### Discussion

Parameterization of the tuning could be improved to obtain better results.
