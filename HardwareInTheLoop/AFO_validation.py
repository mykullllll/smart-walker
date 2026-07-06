import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter, find_peaks
from Control.AFO import AdaptiveFrequencyOscillator

class sig():
    def __init__ (self,dt,cycle_count=0,prev_cycle_count=0,phase=0,phase_const=0,noise=0.05,frequencies=None,amplitude_options=None):

        self.dt=dt
        self.frequencies = frequencies if frequencies is not None else []
        self.amplitude_options = amplitude_options if amplitude_options is not None else []
        self.noise = noise

        self.frequency = np.random.choice(self.frequencies)
        self.amplitude = np.random.choice(self.amplitude_options)

        self.cycle_count=cycle_count
        self.prev_cycle_count=prev_cycle_count
        self.phase=phase
        self.phase_const = phase_const
        

    def freq_step_vary(self):
        signal = self.amplitude * np.sin(self.phase) + np.random.normal(0, self.noise)
        true_frequency = self.frequency
        self.phase+=2*np.pi*self.frequency*self.dt

        #Cycle to new Frequency check
        if self.phase >= 2 * np.pi:
            self.phase -= 2*np.pi
            self.cycle_count+=1

        #Check Cycle count 
        if self.cycle_count > 8 :
            self.frequency = np.random.choice(self.frequencies)
            self.amplitude = np.random.choice(self.amplitude_options)
            self.cycle_count = 0
            self.prev_cycle_count = 0

        return signal,true_frequency


    def freq_const(self,amplitude_const,frequency_const):

        self.phase_const+=2*np.pi*frequency_const*self.dt

        signal = amplitude_const * np.sin(self.phase_const) + np.random.normal(-self.noise, self.noise)
        return signal,frequency_const
   
class stride_validation():
    def __init__(self,stride=None):
        self.stride = stride if stride is not None else []


    def stride_ptp_validation(self,sig):
        scissor_arr=np.array(sig)
        scissor_smooth=savgol_filter(scissor_arr,window_length=5,polyorder=3)

        #print(f'Range of peaks {np.ptp(scissor_smooth)}')
        peak_scissor,_= find_peaks(scissor_smooth,prominence=0.4 *np.ptp(scissor_smooth)) 
        
        for index in range(len(peak_scissor)-1):
            self.stride.append(np.ptp(scissor_smooth[peak_scissor[index]:peak_scissor[index+1]]))
        
        return self.stride, len(self.stride)




fs = 10
dt = 1/fs
afo_variable = AdaptiveFrequencyOscillator(sampling_frequency=10,eta=2.5,eps=1.5)
afo_constant = AdaptiveFrequencyOscillator(sampling_frequency=10,eta=2.5,eps=1.5)


signal_gen = sig(dt=dt,frequencies = [0.5, 0.8, 1.1, 0.6],amplitude_options = [0.3,0.45,0.55,0.65,0.75],noise=0.05)
val_stride = stride_validation()


cadence_hist=[]
abs_freq_error=[]
true_freq_hist = []
input_signal=[]


cadence_hist_const=[]
abs_freq_error_const=[]
true_freq_hist_const = []
input_signal_const =[]

rate_convergence=None
threshold=0.05
convergence_tracker=0
convergence_tracker_const=0
rate_convergence_const=None




for i in range(900):
    t = i*dt
    scissor_signal,true_freq = signal_gen.freq_step_vary()
    scissor_signal_const, true_freq_const = signal_gen.freq_const(amplitude_const=0.5,frequency_const=0.8)

    _, cadence = afo_variable.step_afo(scissor_signal)
    _, cadence_const = afo_constant.step_afo(scissor_signal_const)

    abs_freq_error.append(np.abs(true_freq-cadence))
    cadence_hist.append(cadence)
    input_signal.append(scissor_signal)
    true_freq_hist.append(true_freq)

    abs_freq_error_const.append(np.abs(true_freq_const-cadence_const))
    cadence_hist_const.append(cadence_const)
    true_freq_hist_const.append(true_freq_const)
    input_signal_const.append(scissor_signal_const)

    #Convergence Check
    if abs(cadence - true_freq) < threshold:
        convergence_tracker+=1
        if convergence_tracker > 20 and rate_convergence is None:
            rate_convergence=(t-20) * dt
    else:
        convergence_tracker = 0

    #Convergence Check Constant, Need to change per cycle convergence
    if abs(cadence_const - true_freq_const) < threshold:
        convergence_tracker_const+=1
        if convergence_tracker_const > 20 and rate_convergence_const is None:
            rate_convergence_const=t
    else:
        convergence_tracker_const = 0


def post_calc(abs_freq_error,cadence_hist):

    abs_freq_error = np.asarray(abs_freq_error)
    cadence_hist = np.asarray(cadence_hist)
    max_error = np.max(abs_freq_error)
    max_error_time = np.argmax(abs_freq_error) * dt
    freq_error_avg = np.mean(abs_freq_error)
    std_cadence = np.std(cadence_hist)

    return freq_error_avg,max_error,max_error_time,std_cadence

val_stride_variable = stride_validation()
val_stride_constant = stride_validation()


stride_history, num_of_strides =val_stride_variable.stride_ptp_validation(input_signal)
stride_history_const, num_of_strides_const =val_stride_constant.stride_ptp_validation(input_signal_const)

freq_error_avg,max_error,max_error_time,std_cadence= post_calc(abs_freq_error,cadence_hist)
freq_error_avg_const,max_error_const,max_error_time_const,std_cadence_const = post_calc(abs_freq_error_const,cadence_hist_const)

print(f'----- Variable Frequency Signal Input Metrics ----')
print(f'Convergence Time: {rate_convergence}')
print(f'Absolute average Frequency Error: {freq_error_avg}')
print(f'Maximum Error: {max_error} at {max_error_time}')
print(f'Standard Deviation of Cadence (Smoothness): {std_cadence}')
print(f'Number of Strides: {num_of_strides}')
#print(f'Stride Lengths: {stride_history}')



print(f'----- Constant Frequency Signal Input Metrics ----')
print(f'Convergence Time: {rate_convergence_const}')
print(f'Absolute average Frequency Error: {freq_error_avg_const}')
print(f'Maximum Error: {max_error_const} at {max_error_time_const}')
print(f'Standard Deviation of Cadence (Smoothness): {std_cadence_const}')
print(f'Number of Strides: {num_of_strides_const}')
#print(f'Stride Lengths: {stride_history_const}')








'''Objectives:

mean absolute frequency error
percent frequency error
max error
convergence time
cadence smoothness/std

Realistic Noise Parameters:
Noise
Missing frames
Occlusion
irregular step timing

'''
