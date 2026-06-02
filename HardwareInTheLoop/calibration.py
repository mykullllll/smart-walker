import numpy as np
from scipy.signal import savgol_filter, find_peaks
from scipy.interpolate import interp1d



def leg_frequency(signal,fs):

    signal=signal-np.mean(signal)
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


def calibration(right,left,signal,sampling_frequency,encoder_velocity,current_omega):

    scissor_arr=np.array(signal)
    scissor_smooth=savgol_filter(scissor_arr,window_length=5,polyorder=3)
    peak_scissor,_= find_peaks(scissor_smooth,prominence=0.3 *np.ptp(scissor_smooth))
    valley_scissor,_= find_peaks(-scissor_smooth,prominence=0.3*np.ptp(scissor_smooth))
    normalized_smooth=[]
    for index in range(len(peak_scissor)-1):
        start_idx=peak_scissor[index]
        end_idx=peak_scissor[index+1]
        step_data=scissor_smooth[start_idx:end_idx]
        raw_time=np.linspace(0,1,len(step_data))
        interp_r= interp1d(raw_time,step_data,kind="cubic")
        normalized_time=np.linspace(0,1,100) #Need to change this to (start_time:end_time,100)
        stretched_step1= interp_r(normalized_time)
        normalized_smooth.append(stretched_step1)
        
    if len(normalized_smooth) < 2:
        return False, None, None

    normalized_smooth=np.array(normalized_smooth)
    gold_cycle=np.mean(normalized_smooth,axis=0)
    std=np.std(normalized_smooth,axis=0)           
    std_avg=np.mean(std)

    last_stride=np.ptp(normalized_smooth)
    fft_freq_hz=np.clip(leg_frequency(scissor_smooth,sampling_frequency),0.3,2.0)
    fft_freq=fft_freq_hz*(2*np.pi)
    


    if std_avg>0.5 or np.abs(current_omega-fft_freq) > 0.5:
        return False, None, None #Calibration Failed 
    else:
        # 1. Calculate the walker's average forward speed over the last 50 frames
        avg_walker_speed = np.mean(encoder_velocity[-50:])
        velocity_gain = avg_walker_speed / (fft_freq_hz * last_stride + 1e-6)
        x_d = np.mean(np.mean(right[-50:])+np.mean(left[-50:]))
        return True, x_d, velocity_gain  #Calibration Success
    
