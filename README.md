# Term2 - Project 4: PID Control
### Ajay Paidi

# Objective
The objective of this project is to implement a PID controller to control the movement of a virtual car in the Udacity simulator.

# File structure
- **ReadMe.md**: This file
- **main.cpp**: The main executable program that calls the PID controller to get a steering value and communicates with the virtual car in the simulator using uwebsockets.
- **PID.h** and **PID.cpp**: Contains the implementation of the PID controller and twiddle optimizer.

# Description

A PID controller stands for Proportional-Integral-Derivative controller. Given an error value (between measured and expected values), the PID controller tries to smoothly minimize the deviation by applying control factors (Kp - proportional gain, Kd - derivative gain, ki - Integral gain) to the error term before feeding it back to the control system. In our case the error term is the Cross Track Error (CTE) which is the difference between the measured and expected car positions in the simulator.

Final Error = -Kp * CTE - Kd * (CTE(t) - CTE(t-1)) - Ki * sum(CTE)   

A large Kp has the effect of 'over compensating' resulting in oscillations. The Kd term controls for these oscillations by factoring in the derivative. The Ki term takes into the account the duration of the error thereby compensating for any systemic errors that tend to build up over time.

# Parameter tuning

The implemented controller uses a twiddle (local hill climbing algorithm) algorithm to fine tune the Kp, Kd and Ki parameters. The initial values of these parameters were chosen empirically. It was found that range of Kp was around [0.01, 0.9], the range of Kd was around [1,7] and the range of Ki was around [0, 0.009]. The parameters were initialized with Kp = 0.01, Kd = 1.0 and Ki = 0.001.After applying twiddle (through multiple runs of the simulator) the best values converged at Kp = 0.341, Kd = 4.31 and Ki = 0.001.

# Results

[![PID](https://img.youtube.com/vi/t5JhI5EGqxo/0.jpg)](https://youtu.be/t5JhI5EGqxo)
