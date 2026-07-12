import numpy as np
from scipy.signal import savgol_filter, find_peaks
from scipy.interpolate import interp1d



'''def leg_frequency(signal,fs):
    pad = 1024
    signal=signal-np.mean(signal)
    #tstep=1/fs  
    y=np.fft.fft(signal,n=pad)
    y=np.abs(y)
    f=np.fft.fftfreq(len(y),fs)  
    y = y[:int(len(y)/2)]  
    f = f[:int(len(f)/2)]   
    #plt.scatter(f,y)
    sort_indices = np.argmax(y)
    max_freq=f[sort_indices]
    #print(f"Movement detected at {max_freq} Hz")
    return max_freq'''


def calibration(right,left,signal,sampling_frequency,cal_encoder_velocity,current_omega,wheel_radius,timestamps):
    scissor_arr=np.array(signal)
    scissor_smooth=savgol_filter(scissor_arr,window_length=5,polyorder=3)

    print(f'Range of peaks {np.ptp(scissor_smooth)}')
    peak_scissor,_= find_peaks(scissor_smooth,prominence=0.4 *np.ptp(scissor_smooth))
    valley_scissor,_= find_peaks(-scissor_smooth,prominence=0.4*np.ptp(scissor_smooth))
    print(f'Found {len(peak_scissor)} peaks and {len(valley_scissor)} valleys')

    encoder_timestamps=timestamps
    cycle_gains=[]
    cycle_freqs=[]
    normalized_smooth=[]
    cal_encoder_velocity=np.asarray(cal_encoder_velocity,dtype=float)
    timestamps=np.asarray(timestamps,dtype=float)
    time_to_cal = timestamps[-1] - timestamps[0] if len(timestamps) > 0 else None

    for index in range(len(peak_scissor)-1):
        start_idx=peak_scissor[index]
        end_idx=peak_scissor[index+1]
        step_data=scissor_smooth[start_idx:end_idx]
        if len(step_data)<4:
            continue

        raw_time=np.linspace(0,1,len(step_data))
        interp_r= interp1d(raw_time,step_data,kind="cubic")
        normalized_time=np.linspace(0,1,100) #Need to change this to (start_time:end_time,100)
        stretched_step1= interp_r(normalized_time)
        normalized_smooth.append(stretched_step1)


        start_time=timestamps[start_idx]
        end_time=timestamps[end_idx]
        encoder_timestamps=timestamps

        cycle_period=end_time-start_time
        if cycle_period <= 0:
            continue
        cycle_frequency=1/cycle_period
        cycle_freqs.append(cycle_frequency)
        
        mask = (encoder_timestamps>=start_time) & (encoder_timestamps<end_time)
        if not np.any(mask):
            print(f'No encoder data found for step {index}')
            continue

        cycle_stride=np.ptp(step_data)
        cycle_encoder_velocity = np.mean(cal_encoder_velocity[mask]) * wheel_radius
        cycle_raw_speed = cycle_frequency * cycle_stride
        cycle_gain = cycle_encoder_velocity / (cycle_raw_speed + 1e-6)
        cycle_gains.append(cycle_gain)



    if len(cycle_gains)<2:
        return False, None, None, None, None, None
    velocity_gain = np.median(cycle_gains)
    raw_frequency=np.mean(cycle_freqs)



    normalized_smooth=np.array(normalized_smooth)
    gold_cycle=np.mean(normalized_smooth,axis=0)
    std=np.std(normalized_smooth,axis=0)           
    std_avg=np.mean(std)
    last_stride=np.ptp(gold_cycle)


    print('----- Calibration Results -----')

    if len(normalized_smooth) < 2:
        print(f'Failed Calibration Found {len(peak_scissor)} peaks need at least 2 peaks')
        return False, None, None, None, None, None #Calibration Failed
    
    if std_avg>0.5  or last_stride is None or last_stride<0.05:
        print(f'Standard deviation average {std_avg}')
        print(f'Current Omega {current_omega}')
        print("Failed Calibration")
        print(f' Current Velocity AFO {current_omega}')
        return False, None, None, None, None, None #Calibration Failed 
    
    if not np.isfinite(velocity_gain) or velocity_gain <= 0 or velocity_gain > 10:        
        print(f'Velocity Gain {velocity_gain} out of range')
        print(f'calibration failed')
        return False, None, None, None,None,None#Calibration Failed 
    else:
        # 1. Calculate the walker's average forward speed over the last 50 frames
        x_d = (np.mean(right[-50:])+np.mean(left[-50:]))/2
        #print(f'FFT Frequency {fft_freq}')
        print(f' Current Velocity AFO {current_omega}')
        print(f'Velocity Gain {velocity_gain}')
        print('calibration successful')
        return True, x_d, velocity_gain*0.85 ,last_stride, raw_frequency,time_to_cal  #Calibration Success
