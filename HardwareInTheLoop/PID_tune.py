import numpy as np 
from matplotlib import pyplot as plt

class PID:
    def __init__(self,k_p,k_i,k_d,error_history=None,dt=0.1):
        self.k_p = k_p
        #self.k_i = k_i
        self.k_d = k_d
        self.error_history= error_history if error_history is not None else []
        self.dt=dt

    def pid_controller(self,pelvis,x_d):
        error= x_d - pelvis
        self.error_history.append(error)

        #i_term= self.k_i * np.trapz(self.error_history, np.arange(len(self.error_history)))
        p_term = self.k_p * error 
        if len(self.error_history) >= 2:
            d_term = ((error - self.error_history[-2]) / self.dt) * self.k_d
        else:
            d_term=0

        feedback_velocity = d_term + p_term
        return feedback_velocity

dt=0.1
controller = PID(k_p=1.5,k_i=0.01,k_d=0.3,dt=dt)

x_d = -0.3
pelvis=-1.2
length=1000
list=[]
time_history=[]

for i in range(length):
    velocity=controller.pid_controller(pelvis,x_d)
    pelvis += velocity * dt

    list.append(pelvis)
    time_history.append(i*dt)

plt.figure(1)
plt.plot(time_history,list,color='blue',label='Change in position')
plt.axhline(y=x_d, color='red', linestyle='--', label='Desired Position (x_d)')
plt.ylabel("m")
plt.xlabel("time (s)")
plt.show()





