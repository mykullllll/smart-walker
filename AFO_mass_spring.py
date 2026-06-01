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
import os

files = [
    "/Users/michaelnawa/SmartWalker/Simulations/Data/Data/straight_line.csv",
    "/Users/michaelnawa/SmartWalker/Simulations/Data/Data/slow.csv",
    "/Users/michaelnawa/SmartWalker/Simulations/Data/Data/fast.csv",
    "/Users/michaelnawa/SmartWalker/Simulations/Data/Data/staggered.csv",

]
colors = ['blue', 'green', 'orange','purple']  # one per file

num_files=len(files)


def step_afo(signal,omega0,x0,y0,eps,mu,eta,dt):
    r=np.sqrt(x0**2 +y0**2) + 1e-6
    xdot=(mu-np.square(r))*x0 - omega0 * y0 + eps*signal
    ydot=(mu-np.square(r))*y0 + omega0 * x0
    omegadot= -eta*signal*y0/r

    x_new=xdot*dt+x0
    y_new=ydot*dt+y0

    omega= np.clip(omega0 + omegadot * dt,0.3,4.0)

    phase = ((np.arctan2(y_new,x_new)) /(2*np.pi)) % 1.0

    return phase, x_new, y_new, omega

#3 Median buffer for outliers
def median_buffer(raw_left, raw_right, right_buffer, left_buffer, clean_scissor, scissor_history):

    right_buffer.append(raw_right)
    left_buffer.append(raw_left)
    if len(right_buffer) > 3:
        right_buffer.pop(0)
        left_buffer.pop(0)

    if len(right_buffer) == 3:
        scissor_val = np.median([right_buffer[i] - left_buffer[i] for i in range(3)])
        clean_scissor.append(scissor_val)
        scissor_history.append(scissor_val)

    if len(clean_scissor)>50:
        scissor_arr=np.array(clean_scissor)
        scissor_smooth=savgol_filter(scissor_arr,window_length=5,polyorder=3)
        scissor_smooth=scissor_smooth-np.mean(scissor_smooth)
        peak_scissor,_=find_peaks(scissor_smooth,prominence=0.3 *np.ptp(scissor_smooth))
        valley_scissor,_=find_peaks(-scissor_smooth,prominence=0.3*np.ptp(scissor_smooth))
        clean_scissor.pop(0)

        return peak_scissor, valley_scissor, scissor_smooth, right_buffer, left_buffer, clean_scissor
    
    return np.array([], dtype=int), np.array([], dtype=int), np.zeros(50), right_buffer, left_buffer, clean_scissor


#Calibration Standard deviation
def calibration(peak_scissor,scissor_smooth):
    normalized_smooth=[]
    for index in range(len(peak_scissor)-1):
        start_idx=peak_scissor[index]
        end_idx=peak_scissor[index+1]
        step_data=scissor_smooth[start_idx:end_idx]
        raw_time=np.linspace(0,1,len(step_data))
        interp_r= interp1d(raw_time,step_data,kind="cubic")
        normalized_time=np.linspace(0,1,100)
        stretched_step1= interp_r(normalized_time)
        normalized_smooth.append(stretched_step1)

    normalized_smooth=np.array(normalized_smooth)
    calibrated_smooth=np.mean(normalized_smooth,axis=0)
    std=np.std(normalized_smooth,axis=0)           
    std_avg=np.mean(std)

    return std_avg,std,normalized_smooth,calibrated_smooth


def leg_frequency(leg_data,fs):

    signal=leg_data-np.mean(leg_data)
    tstep=1/fs  
    y=np.fft.fft(signal)
    y=np.abs(y)
    f=np.fft.fftfreq(len(y),1/fs)  
    y = y[:int(len(y)/2)]  
    f = f[:int(len(f)/2)]   
    #plt.scatter(f,y)
    sort_indices = np.argmax(y)
    max_freq=f[sort_indices]
    #print(f"Movement detected at {max_freq} Hz")
    return max_freq



def process_trial(filepath):

    df = pd.read_csv(filepath)

    #Initial variables 
    min_dist=0.25
    max_dist=2
    sampling_frequency=10
    radius=.1016




    #AFO initial variables
    x=1.0
    y=0
    mu=1
    eta=0.3
    eps=1.5
    omega0=2
    dt=1/sampling_frequency
    phi=[]
    signal=[]
    phase=[]



    #Mass spring
    x_d= -0.4
    k=2
    beta=0.3
    weight=[0.1,0.3,0.6]
    alpha=0.2
    filtered_xdot=None
    prev_pelvis=None



    #Filter Variables
    clean_scissor=[]
    right_buffer=[]
    left_buffer=[]

    #Kalman filter
    

    #Clustering data
    occlusion=0
    clump=0
    accurate=0
    noise=0
    width_history=[]

    #Calibration Variables
    gold_calibration=False
    commanded_timestamps=[]
    offset_smooth=0

    #Graphing
    log_time=[]
    log_x_left=[]
    log_x_right=[]
    log_occlusions=[]
    afo_freq_hist=[]
    fft_freq_hist=[]
    stride_window=[]
    fft_window_size=50
    velocity_history=[]
    scissor_history=[]
    stride_history=[]
    average_position=[]
    encoder_velocity=[]
    true_timestamp=[]
    prev_row=None
    prev_scissor=0
    lock_on_time = None
    feedback_velocity=0
    feedback_velocity_graph=[]
    rsme_feedback=[]



    for _, data in df.iterrows():

        if pd.isna(data[0]) or pd.isna(data[2]):
            scissor_raw = prev_scissor
        else:
            #Kalman Filter add
            scissor_raw=data[0]-data[2]

            prev_scissor=scissor_raw

        centered_signal = scissor_raw - offset_smooth

        phase, x, y, omega = step_afo(centered_signal, omega0, x, y, eps, mu, eta, dt)
        omega0 = omega
        cadence=(omega0/(2*np.pi))
        average_position.append((data[2]+data[0])/2)
        pelvis=(data[2]+data[0])/2

        #Mass spring damper

        if prev_pelvis is not None:
            raw_xdot=(pelvis-prev_pelvis) / dt
            filtered_xdot= raw_xdot
            if filtered_xdot is not None:
                filtered_xdot=alpha*raw_xdot + filtered_xdot * (1-alpha)
                rsme_feedback.append(((data[0]+data[2])/2)-x_d)
                feedback_velocity=k*(((data[0]+data[2])/2)-x_d)-(filtered_xdot*beta)
                
        else:
            feedback_velocity=0
            
        feedback_velocity_graph.append(feedback_velocity)
        prev_pelvis=pelvis

            


        if prev_row is not None:
            encoder_velocity.append(data[5])


        prev_row=data
        true_timestamp.append(data[4])
        log_x_right.append(data[2])
        log_x_left.append(data[0])
        afo_freq_hist.append(cadence)
        stride_window.append(scissor_raw)




        if not gold_calibration:
            if pd.isna(data[0]) or pd.isna(data[2]):
                continue
            else:
                peak_scissor, valley_scissor, scissor_smooth, right_buffer, left_buffer, clean_scissor = median_buffer(data[0], data[2], right_buffer, left_buffer, clean_scissor, scissor_history)

            if len(peak_scissor)<2:
                #print("Not enough data points found keep walking")
                continue
            else:
                normalized_smooth=[]
                std_avg,std,normalized_smooth,calibrated_smooth=calibration(peak_scissor,scissor_smooth)
            
            if std_avg> 0.5:
                clean_scissor=clean_scissor[-40:]
                #print("Standard Deviation too large")
                continue
            else:
                gold_calibration=True
                last_stride=np.ptp(calibrated_smooth)
                last_stride=np.clip(last_stride,0.1,1.5)
                fft_freq=leg_frequency(np.array(clean_scissor),sampling_frequency)
                fft_freq=np.clip(fft_freq,0.3,2.0)
                omega0=fft_freq*(2*np.pi)
                offset_smooth = np.mean(calibrated_smooth)
    

                # 1. Calculate the walker's average forward speed over the last 50 frames
                avg_walker_speed = np.mean(encoder_velocity[-50:])
                velocity_gain = avg_walker_speed / (fft_freq * last_stride + 1e-6)
                x_d = np.mean(average_position[-50:])
            

                continue

        #Velocity Calculation
        if pd.isna(data[0]) or pd.isna(data[2]):
            if not gold_calibration:
                continue
            else:
                cadence= (omega0/(2*np.pi)) 
                target_velocity= last_stride * cadence * velocity_gain
                velocity_history.append(target_velocity)
                commanded_timestamps.append(data[4])
        else:
            if not gold_calibration:
                continue
            if len(stride_window)>fft_window_size:
                stride_window.pop(0)
            if len(stride_window)==fft_window_size:
                last_stride = np.ptp(stride_window)            #Need to change this sometime (PTP finds maximum and minimum of window, but is there a better way to scale the stride length? Kalman maybe)
                last_stride = np.clip(last_stride,0.3,1.5)
                stride_history.append(last_stride)

                #Convergence
                fft_freq=leg_frequency(stride_window,sampling_frequency)
                if fft_freq - cadence <=0.2:
                    if lock_on_time is None:
                        lock_on_time = data[4]

            
            feed_forward_velocity=cadence * last_stride * velocity_gain
            velocity_history.append(np.clip(feed_forward_velocity + feedback_velocity,0.0,2.0))
            commanded_timestamps.append(data[4])


    encoder_velocity = [0.0] + encoder_velocity
    average_position_arr = pd.Series(average_position).ffill().to_numpy()


    nyquist = 0.5 * sampling_frequency  # 5.0 Hz for a 10 Hz system
    cutoff = 0.5                        # Cut off anything faster than 0.5 Hz (kills the stepping wobble)
    b, a = butter(2, cutoff / nyquist, btype='low')
    
    # 2. Apply zero-phase filtering to the position array
    smoothed_position = filtfilt(b, a, average_position_arr)



    pelvis_velocity=-np.diff(smoothed_position)*sampling_frequency
    pelvis_velocity=np.append(pelvis_velocity,pelvis_velocity[-1])
    true_velocity=pelvis_velocity+np.array(encoder_velocity)
    true_velocity=np.array(true_velocity)
    velocity_history=np.array(velocity_history)
    rsme_feedback=np.array(rsme_feedback)
    rsme_feedback=np.mean(np.square(rsme_feedback))

    return commanded_timestamps, velocity_history, true_timestamp, true_velocity, lock_on_time, feedback_velocity_graph, rsme_feedback




fig, axes = plt.subplots(nrows=num_files, ncols=1, figsize=(10, 3 * num_files), sharex=True)

if num_files == 1:
    axes = [axes]

for ax, filepath, color in zip(axes, files, colors):
    label = os.path.basename(filepath)
    commanded_timestamps, velocity_history, true_timestamp, true_velocity,lock_on_time,feedback_velocity_graph,rsme_feedback = process_trial(filepath)
    
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

    if lock_on_time is not None:
        # Draw a thick, dotted vertical line at the exact time of convergence
        ax.axvline(x=lock_on_time, color=color, linewidth=2.5, alpha=0.8, label="Lock-on")
    
    # Y-label goes on every graph
    ax.set_ylabel("velocity (m/s)")
    ax.legend(loc="upper right")
    ax.set_ylim(-0.5, 2.0)


axes[-1].set_xlabel("time (s)")

# tight_layout prevents the labels of one graph from overlapping the graph above it
plt.tight_layout() 
plt.show()
