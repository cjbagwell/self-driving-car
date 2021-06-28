# Self Driving Car
**Version 0.1.0**
This project aims to control a vehicle in the Carla Simulator from raw sensor inputs.  This is meant to be a learning opportunity for developing a large scale project using git.  Currently, the project only has the vehicle controller and localization code implemented.

## Controller
The controller of a self driving car is responsible for computing the Accelerator Pedal Position (APP), Brake Pedal Position (BPP), and Steering Angle for the vehicle to reach a desired state.  The control problem can be simplified by separating it into two separate components; longitudinal control and lateral control.

### Longitudinal Control
The longitudinal vehicle controller is responsible for computing the required APP and BPP for the ego vehicle to reach a desired speed.  The longitudinal controller implemented in this project uses a simple PID controller to calculate if the vehicle must accelerate or decelerate in order to reach the target speed.  This done by taking the difference between the desired speed and the current speed of the ego vehicle.  The high level error signal produced is then translated into APP and BPP setpoints using a low level controller which translates positive errors into positive APP values and negative errors into positive BPP values.

### Lateral Control
The lateral vehicle controller is responsible for computing the required steering angle to reach a desired location and orientation.  The lateral controller implemented in this project is the Stanley controller.  The stanley controller utilizes two sources of error: heading error and crosstrack error.  The heading error is simply the difference between the desired heading and the current heading of the ego vehicle.  The crosstrack error is the perpendicular distance from the front axle of the vehicle to the trajectory the vehicle is trying to follow.  These two forms of error are combinded by using <img src="https://render.githubusercontent.com/render/math?math=$\delta(t) = \psi_{error}(t) %2B \tan^{-1}({\frac{k_{cte}*cte(t)} {v_{soft} %2B v_f(t)}})$">.

## Localization
The localization module of a self driving car is responsible for approximating the ego vehicle's location using available sensor measurements.  Sensor measurements are fused together using an Error-state Extended Kalman Filter. 
### Error-State Extended Kalman Filter
An Error-State Extended Kalman Filter uses a nonlinear motion model that opperates on measurements from the Inertial Measurement Unit (IMU).  However, IMU measurements can be noisy and prone to sensor drift, so the motion estimate will accumulate errors at each time step.  To counteract this, Kalman Filters perform updates whenever alternate sensor measurements become available.  However, these measurements are also noisy.  The filter can accomadate this by propogating the covariance during each sensor event and computing the most likely prediction.  
The term error-state means that the filter is not predicting the actual state at each step, but rather the error of the current state prediction.  This is useful as the values are smaller and tend to be better better approximated than the full state.

