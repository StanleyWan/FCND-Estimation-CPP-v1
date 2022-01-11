# FCND-Estimation-CPP
![base_filter](./images/base_filter.png)  
This project is part of [Udacity](https://www.udacity.com "Udacity - Be in demand")'s [Flying Car Nanodegree](https://www.udacity.com/course/flying-car-nanodegree--nd787).  The project is to build the estimation portion for Drone Controller with C++ in a simulator. The project is using Visual Studio 2022.


---


# Project Description
The assignment of this project is mainly to develop key software functions for Drone Estimator and then work with the PID Controller that we did on the last project to form a Drone Controller. On this project, the technique of Sensor Fusion has been implement through Extended Kalman Filter and Complimentary Filter.  To ensure the project move smooth,  the project is developed step by step. And there are 6 steps:
<ul>
        <li>Sensors Noise</li>
        <li>Attitude Estimation</li>
        <li>Prediction Step</li>
        <li>Magnetometer Update</li>
        <li>Closed Loop + GPS Update</li>
        <li>Adding the PID Controller</li>        
</ul>        


# Project Setup
<ul>
<li>Download or clone the C++ simulator repository</li>  

    git clone https://github.com/udacity/FCND-Estimation-CPP.git

<li>Download and install Visual Studio.</li>
<li>Select Open Project/Solution and open <simulator>/Simulator.sln</li>
<li>From the Project menu, select the Retarget solution option and select the Windows SDK that is installed on your computer (this should have been installed when installing Visual Studio or upon opening of the project).</li>
<li>To compile and run the project/simulator, simply click on the green play button at the top of the screen.  When you run the simulator, you should see a single quadcopter, falling down.</li>
</ul>

# Implementation 
All the C++ codes are in the [./src](./src) directory. However, most of my work only focus on the below four files:  <ul>
        <li> [QuadEstimatorEKF.cpp](./src/QuadEstimatorEKF.cpp): it contains the software modules that need to build the Estimator.</li>
        <li> [QuadEstimatorEKF.txt](./src/QuadEstimatorEKF.txt): it contains the software modules that need to build the Estimator.</li>
        <li> [QuadControl.cpp](./src/QuadControl.cpp): it contains the software modules that need to build the cascaded 3D controller.</li>
        <li> [QuadControlParams.txt](./config/QuadControlParams.txt): the file contains the configuration data for the cascaded 3D controller.</li>
</ul>      
                
## Step 1: Sensor Noise
None of the sensors has no noise.  So we need to know how noisy of a sensor.  On this assignment,  the quad's sensor has been simulated to add with some noise. What we need to do is to record some sensor data on a static quad.  And then,  based on the normal distribution,  we the can calculated the standard deviation of a sensor data there.    
The assignment is dedicated to collect the data of GPS X data and accelerometer X data into the [Graph1.txt](./config/log/Graph1.txt) and [Graph1.txt](./config/log/Graph1.txt) respectively.  Through the standard deviation equation:
![sd_equation](./images/sd_equation.png)  
we get the standard deviation 0.68119 and 0.493118 and set on the MeasuredStdDev_GPSPosXY and MeasuredStdDev_AccelXY respectively.  
The following is the test result:
<p align="center">
<img src="images/scenario6.gif" width="500"/>
</p>
<p align="center">
<img src="images/result6"/>
</p>


## Scenario 2: Body rate and roll/pitch control (scenario 2)
### Body Rate Controller
The Body Rate Controller is a P Controller.  The responsibility of the controller is to generate the moments command.  Through the error between the body rate command and actual body rate that fed back from the drone, we could find out desired moments to the drone.

        pqrErr = pqrCmd - pqr
        momentCmd = I * kpPQR * pqrErr
        where pqrCmd is the body rate command for p,q,q
              pqr is the actual body rate fed back from drone 
              pqrErr is the difference between the pqrCmd and prq
              I is the moment inertia
              kpPQR is the parameter of the controller to the error
<p></p>

### Roll-Pitch Controller
The Roll-Pitch Controller is also a proportional controller.  The controller use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. It sets the desired rate of change of the given matrix elements (R13 and R23).  We thus get the error value by subtract the actual matrix element (R13, R23) with the command matrix element (R13, R23). 

![Equation1](./images/equation1.png)   

The codes are implemented on the function BodyRateControl() and RollPitchControl() in the file [QuadControl.cpp](./src/QuadControl.cpp)

The following is the testing result on scenario2.  It mainly tests the leveling capability of a drone.
<p align="center">
<img src="images/scenario2.gif" width="500"/>
</p>

![s2testresult](./images/s2testresult.png)

## Scenario 3: Position/velocity and yaw angle control(scenario 3)

### Altitude Controller  
Altitude controller in scenario 3 is a PD controller.  Based on the input of the position and velocity, the Altitude controller generates the desired acceleration which then be converted to thrust command to Roll-Pitch Controller as well as the drone. 

![Equation2](./images/equation2.png)   

We can based on the difference between the command position and actual position, multiply with the parameter of the Altitude controller to get a proportional control (p_term).  And the difference between the command of velocity and actual velocity , multiply with the parameter of the Altitude controller to get a Derivative control (d_term).

### Lateral Controller
Lateral Controller is a PD controller.  The drone generate lateral acceleration by changing the body orientation.  The equation have the following form:

![Equation3](./images/equation3.png)   

and xt is the target location, xa is the actual location, x_dot_t is the target velocity and x_dot_a is the actual velocity.  x_dot_dot is the feed forward acceleration.

### YawController
YawController is a Proportional controller.  We can get the command yaw rate by multiplying its parameter with the difference between the command psi and the actual psi.  

![Equation4](./images/equation4.png)  

The codes are implemented on the function of AltitudeControl() LateralPosition() YawControl() in the file [QuadControl.cpp](./src/QuadControl.cpp)

The following is the testing result on scenario3.  It mainly tests the rotating and moving capability of a drone.
<p align="center">
<img src="images/scenario3.gif" width="500"/>
</p>

![s3testresult](./images/s3testresult.png)

## Scenario 4: Non-idealities and robustness
Here, we enhanced integral control to the Altitude controller and make it as a PID controller. 

        posZErr = posZCmd - posZ;
        integratedAltitudeError += posZErr * dt;
        i_term = integratedAltitudeError * KiPosZ;
        where i_item is the integral control
              integratedAltitudeError is the integrated Altitude Error
              KiPosZ is the parameter
              posZCmd is the command Z position
              posZ is the actual Z position
              dt is the step of the easurements


This test is used to show how well the controller can control under some unexpected situation such as unexpected heavier in weight or shift of the gravity center.  We config 3 quads that are all are trying to move one meter forward.  However, this time, each drone has a bit different
<ul>
        <li> The green quad has its center of mass shifted back.</li>
        <li> The orange vehicle is an ideal quad </li>
        <li> The red vehicle is heavier than usual</li>
</ul>
The following is the result of the AltitudeController without integral control.  We can see the red drone is failed.
<p align="center">
<img src="images/scenario4_fail.gif" width="500"/>
</p>

![s4testfaul](./images/s2testfail.png)      

The following is the result of the Altitude Controller with integral control.  We can see the red drone is passed.
<p align="center">
<img src="images/scenario4_pass.gif" width="500"/>
</p>

![s4testpass](./images/s4testpass.png)    

We can see the integral control really can improve the performance of the PD controller.      

## scenario 5: Tracking trajectories
This test is to test the performance of the whole 3D Drone controller.  The scenario has two quadcopters:
<ul>
        <li> the orange one is following traj/FigureEight.txt</li>
        <li> the other one is following traj/FigureEightFF.txt which contain the data of feed forward acceleration.</li>
</ul>
The following is the result of the test:

<p align="center">
<img src="images/scenario5.gif" width="500"/>
</p>

![s5testresult](./images/s5testresult.png)

From the result, it is not hard to see that when the red trajectory dded with the feed forward acceleration, those overshoot, setup time and settle time become more controllable. And also, the drone can follow the trajectory consistently.

## The converter between the cascaded 3D controller and the rotors
In between the controller and the rotors, there is a converter that convert the thrust and moments to the appropriate 4 different desired thrust forces for the moments. The value of the thrust forces will then pass to the engins of the rotors. The following is the rotor layout in a 3D drone.

![3D Drone](./images/3D_Drone.png)
The relationship between the lifting force on axes and the thrusts on the four rotor is as follows:
<p></p>

       p_bar = momentCmd.x/l      =    F1 - F2 - F3 + F4          
       q_bar = momentCmd.y/l      =    F1 + F2 - F3 - F4          
       r_bar = momentCmd.z/kappa  =   -F1 + F2 - F3 + F4          
       c_bar = collThrustCmd      =    F1 + F2 + F3 + F4  

       where    p_bar is the total force on x axis, q_bar is the total force on y axis,
                r_bar is the total force on z axis, c_bar is the total lifting force
                momentCmd.x, momentCmd.y and momentCmd.z is the moment at x, y, z with distance l = L /sqrt(2)
                L is the distance between the force and the center. 
                F1, F2, F3 and F4 are the thrust of the rotor1, rotor2, rotor3 and rotor4 respectively.
                kapper is the thrust/drag ratio provided from the simulator
<p></p>
After Calculation, we get:

        F1 = ( p_bar + q_bar - r_bar + c_bar) / 4
        F2 = (-p_bar + q_bar + r_bar + c_bar) / 4         
        F3 = (-p_bar - q_bar - r_bar + c_bar) / 4
        F4 = ( p_bar - q_bar + r_bar + c_bar) / 4 
<p></p>

The code is implemented in the function GenerateMotorCommands() in [QuadControl.cpp](./src/QuadControl.cpp) 


## Conclusion
Since most of the principles on implementing the 3D Drone controller have been taught in the course,  then the most difficulty part that left on this project is the parameter tuning.  It almost exhaust me because the controllers always need to retune once and once again.  I believe, in this project, the parameters setting is only barely enough to pass the scenario test.  In the future, when I have more time,  I would like to come back to retune this parameters so it has the smallest settle time, low over shoot and can perfectly follow the trajectory. 

