# Overview
The Smart Walker is an autonomous rehabilitation device used to help patients with dementia or other gait disabilities learn how to walk again. 

The current control system uses force torque sensors to measure conscious intent of the user during walking in order to walk with the patient without exerting much energy to move the walker. While this is useful, having a control system that only looks at the force being applied to the handles isn’t an accurate depiction of the user's actual intent since it’s not taking into account the users legs. In order to fix this problem with relatively cheap components, I've added a feed forward + feedback control system using a 2D RPLidar A1M8-R6 to perceive the users legs and an AK-10-9 V2.0 motor with magnetic encoders to control the wheels. 

<img src="Docs//Media/Smart_walker_diagram.png" alt="Control-system flowchart" width="800">

# Research Question
Can a non-wearable 2D-LiDAR-sensed walker provide accurate, comfortable, gait-synchronized feedforward assistance despite the sensing constraints of a cheaper (6-10 Hz sampling rate, occlusion, noise)? 

### Subclaims

  * The Hopf AFO tracks cadence with lower latency than windowed FFT/moving-average under this walker's actual noise and occlusion profile. (the estimator benchmark)
  * The occlusion-handling pipeline (DBSCAN + decision tree) preserves leg tracking through a stated fraction of synthetic occlusion events without losing calibration. (new result — doesn't exist yet)
  * The closed-loop controller tracks true user velocity within a stated error bound across trials/subjects.
  * Extracted gait metrics (stride length, symmetry, cadence variability) are usable outputs, ideally checked against a reference measurement. 

# Objectives
1. Design a control system that can measure the users intent through a 2D LiDAR scan of the patients legs in order to command motor speeds in rhythm with the user.
2. Extract gait metrics from sessions i.e (Velocity, Stride Length and Time Variability, Gait Symmetry, Lateral Step Length)
3. Execute Proof of Concept on different gait patterns.


# How It Works
It's difficult to create a real time control system that uses only 2D LiDAR scans due to the low 10 Hz sampling rate, occlusion, and noise from outside LiDAR scans. Because of this, traditional frequency calculation methods like a Fast Fourier Transforms (FFT) has built in latency proportional to it's window size, and resolution is also inversely proportional to the latency shown in the equations below. For a control system that needs to walk in rhythm with a patient that has irregular pacing i.e (changes in stride length and step timing) delayed motor control can cause discomfort and potential injuries when walking. 

In order to solve this I implemented an Hopf Adaptive Frequency Oscillator (AFO) that uses a coupled set of differential equations that converges to the frequency of an input frequency over time. To make sure the input signal to the AFO doesn't have unpredictable noise and is filtered in real time I used a simple low pass filter. In order to make sure the patient is within a comfortable distance, I added a corrective PID controller as well as a freezing gait detection system. 


## ROS2 Topics and sensor inputs

### Publishers
* `/shutdown`: Unlocks Safety switch to turn on motors 
* `/right_wheel_velocity`: Commanded Right Wheel Velocity
* `/left_wheel_velocity`: Commanded Left Wheel Velocity

### Subscribers 
* `/scan_legs_fitlered`: (x,y) coordinates of LiDAR scans
* `/encoder_data`: Wheel Velocities

## Leg Detection

LiDAR scans are grouped using DBSCAN, a density based clustering algorithin that identifies groups of points as clusters. Using the centroid of each point, we define each leg as a single (x,y) point. 

To learn more about cluster detection check [Clustering Validation](/Validation/Clustering)

<img src="Docs/Media/Occlusion_decision_tree.png" alt="Control-system flowchart" width="800">


## Feedforward Controller 

The Hopf Adaptive Frequency Oscillator is a coupled set of differential equations shown below. As you run this equation over many time steps the input signal F(t) forces $\dot{\omega}$ to either speed up or slow down to match the frequency of the input signal. 

[Feedforward Validation Documentation](/Validation/AFO/afo_validation.md)

$$
\begin{aligned}
\ r     &= \sqrt{y^2 + x^2} \\
\dot{x} &= (\mu - r^2)x - y + \epsilon * F(t) \\
\dot{y} &= (\mu - r^2)y + x \\
\dot{\omega} &= \frac{\eta F(t) y}{r} \\
\omega  &= \dot{\omega} * dt + \omega
\end{aligned}
$$


https://github.com/user-attachments/assets/7dd72901-bb62-4123-a32d-af2526f6bd0f

> [!NOTE]
> * $\eta$ - Changes the rate of convergence of the AFO frequency to the input signal frequency
> * $\epsilon$ - Changes sensitivity of $\dot{x}$ to the input signal
> * $\mu$ - Baseline radius with no input signal. Represents the amplitude of your AFO.


## Feedback Controller
A PID velocity controller was used as a corrective velocity when patients were outside of the desired zone. Further documentation on validation process and tests are here:

[Feedback Validation Documentation](/Validation/PID/PID_Validation_Document.md)

### Calibration
Initial gait metrics of the user is needed to calibrate the necessary speed / frequency of their gait patterns. During this period the user pushes the walker themselves. Average standard deviation is calculated to ensure smooth walking during calibration and the below equation is used to calculate necessary gain to compensate for the difference in leg movement vs needed speed of the walker. 

$$
\begin{aligned}
\ gain &= EncoderVelocity / (cycle_frequency * cycle_stride)
\end{aligned}
$$


# Startup Commands
* `cd ~/ros2_ws`
* `ros2_load`
* `ros2 launch rplidar_ros gait_lidar_launch.py`
* `docker run -it --rm -v /dev:/dev --privleged --net=host microros/micro-ros-agen-jazzy serial --dev /dev/ttyUSB1 =b 115200`
* `python3 ~/ros2_ws/src/control_system/control_system/AFO_control.py`


# Example Output
Calibration time: 14.833203554153442 s  
Mean Command: 2.453 rad/s  
Mean Encoder:      2.826 rad/s  
RMSE Predicted to True Velocity:       0.902 rad/s  
Mean error: -0.01763556725637624 --- Negative : missing low --- Positive : missing high ---  
Mean error absolute (MAE): 0.4707271453445335  
Standard Deviation of commanded velocity 1.0432527452144844  
Time in Active Assist: 70.43010752688173 %   
Time in Active Attenuation: 15.591397849462366 %  
Time in Boost: 0.0 %  
Time in 0 Velocity: 13.978494623655912 %  
Time detected Frozen Gait [35.55018329620361, 47.55037522315979]  

<img src="Docs/Media/Trial_data.png" alt="" width="800">


<img src="Docs/Media/Trial_vid.mp4" alt="" width="800">


https://github.com/user-attachments/assets/790466eb-0060-42ea-8940-d9f74e1ff06b


### References

1. https://pubmed.ncbi.nlm.nih.gov/18728766/
2. https://www.sciencedirect.com/science/article/pii/S2405896325032136


