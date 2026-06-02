import pandas as pd
from matplotlib import pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN, KMeans
from matplotlib.animation import FuncAnimation 
import time
import scipy
from scipy.signal import savgol_filter, find_peaks
from scipy.interpolate import interp1d
from scipy.signal import butter, filtfilt
import calibration
import os

files = [
    "/Users/michaelnawa/SmartWalker/Simulations/Data/Data/straight_line.csv",
    "/Users/michaelnawa/SmartWalker/Simulations/Data/Data/slow.csv",
    "/Users/michaelnawa/SmartWalker/Simulations/Data/Data/fast.csv",
    "/Users/michaelnawa/SmartWalker/Simulations/Data/Data/staggered.csv",

]
colors = ['blue', 'green', 'orange','purple']  # one per file

num_files=len(files)

class Gaitsensor:
    'Handles Signal Processing of LiDAR and Leg detection'
    def __init__(self,cal_window=None,scissor_window=None,right=None,left=None,encoder_velocity=None,true_timestamp=None,stride_window=None,avg_position_history=None,prev_scissor=0,prev_avg=0,prev_left=0,prev_right=0):
        self.cal_window = cal_window if cal_window is not None else []
        self.scissor_window=scissor_window if scissor_window is not None else []
        self.right=right if right is not None else []
        self.left=left if left is not None else []
        self.stride_window= stride_window if stride_window is not None else []
        self.encoder_velocity=encoder_velocity if encoder_velocity is not None else []
        self.true_timestamp=true_timestamp if true_timestamp is not None else []
        self.avg_position_history =avg_position_history if avg_position_history is not None else []

        self.prev_scissor=prev_scissor
        self.prev_avg=prev_avg
        self.prev_left=prev_left
        self.prev_right=prev_right

    

    def offline_data(self,left_current,right_current,time,encoder):

        if pd.isna(left_current) or pd.isna(right_current):
            left_raw = self.prev_left
            right_raw = self.prev_right
            scissor_signal = self.prev_scissor
            avg_position=self.prev_avg

            self.left.append(left_raw)
            self.right.append(right_raw)
            self.stride_window.append(scissor_signal)
            self.avg_position_history.append(avg_position)
        else:
            left_raw = left_current 
            right_raw = right_current
            scissor_signal=left_current-right_current
            avg_position=(left_current+right_current)/2

            self.left.append(left_current)
            self.right.append(right_current)
            self.stride_window.append(left_current-right_current)
            self.avg_position_history.append(avg_position)


        if encoder is not None:
            self.encoder_velocity.append(encoder)
        else:
            self.encoder_velocity.append(None)
        if time is not None:
            self.true_timestamp.append(time)
        else:
            self.true_timestamp.append(None)


        self.prev_scissor=scissor_signal
        self.prev_avg = avg_position
        self.prev_left=left_raw
        self.prev_right=right_raw

        return left_raw,right_raw,scissor_signal,avg_position


class AdaptiveFrequencyOscillator:
    'Calculate Frequency of signal'

    def __init__(self,sampling_frequency,eta=0.3,eps=1.5,mu=1):
        self.x=1.0
        self.y=0
        self.omega=2
        self.dt=1/sampling_frequency
        self.eta=eta
        self.eps=eps
        self.mu=mu

    def step_afo(self,signal):

        r=np.sqrt(self.x**2 +self.y**2) + 1e-6
        xdot=(self.mu-np.square(r))*self.x - self.omega * self.y + self.eps *signal
        ydot=(self.mu-np.square(r))*self.y + self.omega * self.x
        omegadot= -self.eta*signal*self.y/r

        self.x= xdot * self.dt + self.x
        self.y=ydot * self.dt + self.y

        self.omega= np.clip(self.omega + omegadot * self.dt,0.3,4.0)

        phase = ((np.arctan2(self.y,self.x)) /(2*np.pi)) % 1.0
        cadence=self.omega/(2*np.pi)

        return phase, cadence
        

class PDController:
    'Feedback controller'

    def __init__(self,sampling_frequency,x_d,k=2,beta=2,alpha=0.2,filtered_xdot=None,prev_pelvis=None,rsme_feedback=None,feedback=None):
        self.x_d=x_d
        self.k=k
        self.beta=beta
        self.alpha=alpha
        self.dt=1/sampling_frequency
        self.prev_pelvis=prev_pelvis
        self.rsme_feedback=rsme_feedback if rsme_feedback is not None else []
        self.feedback=feedback if feedback is not None else []
        self.filtered_xdot=filtered_xdot


    def pd_controller(self,pelvis):
            
        if self.prev_pelvis is not None:
            raw_xdot=(pelvis-self.prev_pelvis) / self.dt
            if self.filtered_xdot is None:
                self.filtered_xdot= raw_xdot
            else:
                self.filtered_xdot=self.alpha*raw_xdot + self.filtered_xdot * (1-self.alpha)

            self.rsme_feedback.append(pelvis-self.x_d)
            feedback_velocity=self.k*(pelvis-self.x_d)-(self.filtered_xdot*self.beta)
            self.prev_pelvis=pelvis
            return feedback_velocity
        
        else:
            self.prev_pelvis=pelvis
            return None
        

class WalkerController:
    'Velocity commands'

    def __init__(self,window_size=50,stride_window=None):
        self.window_size=window_size
        self.stride_window=stride_window if stride_window is not None else []
        self.stride_history = []


    def last_stride(self,stride_window):

        if len(stride_window)>self.window_size:
            stride_window.pop(0)
        if len(stride_window)==self.window_size:
            last_stride = np.ptp(stride_window)            #Need to change this sometime (PTP finds maxiself.mum and miniself.mum of window, but is there a better way to scale the stride length? Kalman maybe)
            last_stride = np.clip(last_stride,0.3,1.5)
            self.stride_history.append(last_stride)
            return last_stride
        
        return None
    
    def velocity_comamnd(self,feedback_velocity,cadence,last_stride,velocity_gain):
        velocity_command = cadence*last_stride*velocity_gain + feedback_velocity
        return velocity_command
    



def process_trial(filepath,sampling_frequency=10):
    df = pd.read_csv(filepath)

    sensor = Gaitsensor()
    oscillator = AdaptiveFrequencyOscillator(sampling_frequency)
    pd_controller = PDController(sampling_frequency, x_d=0)
    walker = WalkerController()

    velocity_history=[]
    commanded_timestamps = []
    calibrated=False
    velocity_gain = 1.0

    for _, data in df.iterrows():

        left,right,scissor_signal,pelvis = sensor.offline_data(data[0],data[2],data[4],data[5])
        phase,cadence=oscillator.step_afo(scissor_signal)
        feedback_velocity=pd_controller.pd_controller(pelvis)
        
        if calibrated== False:
            if len(sensor.scissor_window) == 100 and len(sensor.encoder_velocity) == 100:

                cal,x_d,velocity_gain= calibration.calibration(sensor.right,sensor.left,sensor.scissor_window,sampling_frequency,sensor.encoder_velocity,current_omega=oscillator.omega)
                
                if cal == True:
                    calibrated=True
                    print("Calibration complete")
                else:
                    sensor.left.clear()
                    sensor.right.clear()
                    sensor.scissor_window.clear()
                    sensor.encoder_velocity.clear()

        else:
            last_stride = walker.last_stride(sensor.stride_window)
            if last_stride is not None and feedback_velocity is not None:
                velocity_command = walker.velocity_comamnd(feedback_velocity,cadence,last_stride,velocity_gain)
                commanded_timestamps.append(data[4])
                velocity_history.append(velocity_command)




    encoder_velocity = np.array([v if v is not None else 0.0 for v in sensor.encoder_velocity])

    average_position = np.array(sensor.avg_position_history)

    rsme_feedback = np.array(pd_controller.rsme_feedback)   
    rsme_feedback = np.mean(np.square(rsme_feedback))

    true_timestamp = sensor.true_timestamp



    pelvis_velocity=-np.diff(average_position)* sampling_frequency
    pelvis_velocity=np.append(pelvis_velocity,pelvis_velocity[-1])
    true_velocity=pelvis_velocity+np.array(encoder_velocity)
    true_velocity=np.array(true_velocity)
    velocity_history=np.array(velocity_history)

    return commanded_timestamps, velocity_history, true_timestamp, true_velocity, rsme_feedback




fig, axes = plt.subplots(nrows=num_files, ncols=1, figsize=(10, 3 * num_files), sharex=True)

if num_files == 1:
    axes = [axes]

for ax, filepath, color in zip(axes, files, colors):
    label = os.path.basename(filepath)
    commanded_timestamps, velocity_history, true_timestamp, true_velocity,rsme_feedback = process_trial(filepath)
    
    true_velocity=np.clip(true_velocity,-1.3,1.3)
    velocity_history=np.clip(velocity_history,-1.3,1.3)

    #Velocity error
    print(f"\n--- {label} ---")
    print(f"Predicted mean: {np.mean(true_velocity):.3f} m/s")
    print(f"True mean:      {np.nanmean(true_velocity):.3f} m/s")
    print(f"True std:       {np.nanstd(true_velocity):.3f}")

    start_idx=len(true_velocity)-len(velocity_history)

    rsme=np.sqrt(np.mean(np.square(velocity_history-true_velocity[start_idx:])))

    print(f"RSME Predicted to True Velocity:       {rsme:.3f}")
    print(f"RSME X_avg Real to X desired:     {rsme_feedback:.3f}")
        

    # Plot directly onto 'ax' (the specific subplot for this loop iteration) instead of 'ax2'
    ax.plot(commanded_timestamps, velocity_history, color=color, label=f'{label} predicted')
    ax.plot(true_timestamp, true_velocity, color=color, linestyle='--', label=f'{label} true')

    
    # Y-label goes on every graph
    ax.set_ylabel("velocity (m/s)")
    ax.legend(loc="upper right")
    ax.set_ylim(-0.5, 2.0)


axes[-1].set_xlabel("time (s)")

# tight_layout prevents the labels of one graph from overlapping the graph above it
plt.tight_layout() 
plt.show()
