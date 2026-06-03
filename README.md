# Overview
The current smart walker design uses force torque sensors to measure intent of the user during walking. For the smart walker design that is used to help rehabilitate patients suffering from dementia, having a control system that only looks at the force being applied to the handles isn’t an accurate depiction of the user's intent since it’s not taking into account the users legs. In order to fix this problem with relatively cheap components, we’ve added a feed forward + feedback control system using a 2D RPLidar A1M8-R6 to perceive the users legs and an AK-10-9 V2.0 motor with magnetic encoders. 

![Hybrid Feedforward Feedback Control Loop](Docs/AFO_Control.drawio.svg)


# How It Works
It's difficult to create a control system that uses only 2D LiDAR scans due to the unpredictable gait of dementia patients, 10 Hz sampling rate, and noise from outside laser scans. Because of this, traditional frequency calculation methods like a Fast Fourier Transforms (FFT) rely on a fixed window, which means the latency is proportional to the window size, and resolution is also inversely proptional to window size. 

In order to solve this I implemented an Hopf Adaptive Frequency Oscillator (AFO) that uses a separate set of differential equations that react to every frame and converges to the frequency of any input signal over time. 

To make sure the signal that the AFO is processing doesn't have unpredictable noise and is filtered in real time I used a Kalman filter to predict the next data point. 

One caveat to the AFO is that it doesn't take into account how far the user is to the walker, so I also added a PD controller that takes the average distance of the users pelvis during calibration as the desired distance between the paitent and walker. This ensures the walker to constantly be at a safe distance between itself and the paitent by either accelerating or deccelerating to keep in pace with the user. 

## Components

### Kalman Filter

### Scissor Metric
The input signal I chose was the difference in position of the left leg relative to the right. 

$$
\begin{aligned}
\ x_{signal} &= x_{left} - x_{right}
\end{aligned}
$$

This is optimal for the AFO because it's tracking the frequency of the legs not the frequency relative to the walker. It also removes any offset from 0 that both legs will inherently have. 


### Hopf Adaptive Frequency Oscillator (AFO) 

$$
\begin{aligned}
\ r     &= \sqrt{y^2 + x^2} \\
\dot{x} &= (\mu - r^2)x - y + KF(t) \\
\dot{y} &= (\mu - r^2)y + x \\
\dot{z} &= \frac{\eta F(t) y}{r}
\end{aligned}
$$

### Calibration

### PD Controller 
$$
\begin{aligned}
\ Velocity &= k (x_{signal} - x_{calibrated}) - \beta (\dot{x}) \\
\end{aligned}
$$

# Functions

### Clustering & Filtering
* `scan_callback`: Returns LaserScan msg 
* `process_scan`: Returns x,y coordinates of laserscan values. 
* `cluster_find`: Identifies and returns centroids of clusters found.
* `kalman`: Predicts next centroid values based off of `cluster_find`

### Calibration
* `calibration`: Takes a specified window and interpolates the data between each stride to normalize each stride to be a 1x100 array. Calculates the standard deviation along axis = 0. Returns `True`, `average position`, and `velocity_gain` if `std_avg` < 0.5. 

### Velocity Calculation
