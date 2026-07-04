import pandas as pd
from matplotlib import pyplot as plt
import numpy as np
import os
import sys
sys.path.append("/Users/michaelnawa/Documents/GitHub/smart-walker")
from Control.AFO import main_loop


files = [
    "/Users/michaelnawa/Documents/GitHub/smart-walker/HardwareInTheLoop/data/normal.csv",
    "/Users/michaelnawa/Documents/GitHub/smart-walker/HardwareInTheLoop/data/fast.csv",
]
colors = ['blue', 'green']  # one per file

num_files=len(files)

'''class PDController:
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
            return 0
        
        else:
            self.prev_pelvis=pelvis
            return None
        '''

def process_trial(filepath):
    controller = main_loop()
    df = pd.read_csv(filepath)
    for _, data in df.iterrows():
        wheel_velocity =  controller.step_from_legs(data['time(s)'],data['Encoder'], data['Left_x'],data['Right_x'],isoccluded=False)
    return wheel_velocity,controller



fig, axes = plt.subplots(nrows=num_files, ncols=1, figsize=(10, 3 * num_files), sharex=True)
if num_files == 1:
    axes = [axes]

for ax, filepath, color in zip(axes, files, colors):
    label = os.path.basename(filepath)
    wheel_velocity,controller = process_trial(filepath)
    
    #encoder_velocity=np.clip(encoder_velocity,-1.3,1.3)
    #velocity_history=np.clip(velocity_history,-1.3,1.3)

    #Velocity error
    print(f"\n--- {label} ---")
    print(f"Predicted mean: {np.mean(controller.velocity_history):.3f} rad/s")
    print(f"True mean:      {np.nanmean(controller.encoder_data):.3f} rad/s")

    velocity_history = np.array(controller.velocity_history)
    encoder_velocity = np.array(controller.encoder_data)

    start_idx=len(controller.encoder_data)-len(controller.velocity_history)
    rmse = np.sqrt(np.mean(np.square(velocity_history - encoder_velocity[start_idx:])))

    print(f"RMSE Predicted to True Velocity:       {rmse:.3f}")
        

    # Plot directly onto 'ax' (the specific subplot for this loop iteration) instead of 'ax2'
    ax.plot(controller.commanded_timestamps, controller.velocity_history, color=color, label=f'{label} predicted')
    ax.plot(controller.encoder_time, controller.encoder_data, color=color, linestyle='--', label=f'{label} true')
    ax.plot()

    
    # Y-label goes on every graph
    ax.set_ylabel("velocity (rad/s)")
    ax.legend(loc="upper right")
    ax.set_ylim(-0.5, 10)


axes[-1].set_xlabel("time (s)")

# tight_layout prevents the labels of one graph from overlapping the graph above it
plt.tight_layout() 
plt.show()
