import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
from collections import deque
from matplotlib.animation import FuncAnimation

#Initial Inputs
sampling_frequency=60
dt=1/sampling_frequency
init_mu=2
init_eps=1.5
init_eta=0.3
init_frequency=2
init_amplitude=1
init_noise=1
t=0


#AFO state Variables

x_state,y_state,omega_state = 1, 0, 2

#Signal Input
def f(t,amplitude,frequency,noise):
    return amplitude*np.sin(frequency*t)+np.random.normal(0,noise)


fig, (ax1,ax2,ax3) = plt.subplots(nrows=1,ncols=3,figsize=(15,6))

plt.subplots_adjust(
    left=0.05,    # left edge of plots (0=figure left edge)
    right=0.9,   # right edge of plots — everything right of 0.75 is empty margin
    top=0.90,     # top edge
    bottom=0.4,  # bottom edge — leaves room for sliders below
    wspace=0.35   # horizontal space *between* subplots
)

#Parameterized Function 

def step_afo(signal,omega0,x0,y0,eps,mu,eta):
    r=np.sqrt(x0**2 +y0**2) + 1e-9
    xdot=(mu-np.square(r))*x0 - omega0 * y0 + eps*signal
    ydot=(mu-np.square(r))*y0 + omega0 * x0
    omegadot= -eta*signal*y0/r

    x_new=xdot*dt+x0
    y_new=ydot*dt+y0


    omega= (omega0 + omegadot * dt)

    phase = (np.arctan2(y_new,x_new) + np.pi)/(2*np.pi)

    return phase, x_new, y_new, omega




#Plots
sig_buf = deque([np.nan] * 120, maxlen=120)
xs = list(range(120))
line, = ax1.plot(xs,sig_buf,lw=2,color='Blue')
ax1.set_ylim(-8,8)
ax1.set_title('Signal Input Plot')

afo_buf_x = deque([np.nan] * 120, maxlen=120)
afo_buf_y = deque([np.nan] * 120, maxlen=120)

line2, = ax2.plot(x_state,y_state,lw=2,color='Purple')
ax2.set_xlim(-5 ,5) 
ax2.set_ylim(-3,3)
ax2.set_title('AFO Signal Plot')

signal_freq_buf = deque([np.nan] * 120, maxlen=120)
afo_freq_buf = deque([np.nan] * 120, maxlen=120)
line3, = ax3.plot(xs,list(signal_freq_buf),color="Blue",label="Input Frequency")
line4, = ax3.plot(xs,list(afo_freq_buf),color="Red",label="AFO Frequency")
ax3.legend(loc="upper right")
ax3.set_ylim(0,6)
ax3.set_title('Frequency Convergence')




#Sliders 
ax_amp = fig.add_axes([0.25, 0, 0.65, 0.03])
ax_mu=fig.add_axes([0.25, 0.1, 0.65, 0.03])
ax_eps=fig.add_axes([0.25, 0.15, 0.65, 0.03])
ax_eta=fig.add_axes([0.25, 0.2, 0.65, 0.03])
ax_freq = fig.add_axes([0.25, 0.25, 0.65, 0.03])
ax_noise=fig.add_axes([0.25, 0.3, 0.65, 0.03])


amp_slider   = Slider(ax=ax_amp,   label='Input Amplitude',   valmin=0, valmax=8.0, valstep=0.1, valinit=init_amplitude)
mu_slider    = Slider(ax=ax_mu,    label='mu',                valmin=0, valmax=4.0, valstep=0.1, valinit=init_mu)
eps_slider   = Slider(ax=ax_eps,   label='eps',               valmin=0, valmax=3.0, valstep=0.1, valinit=init_eps)
eta_slider   = Slider(ax=ax_eta,   label='eta',               valmin=0, valmax=1.0, valstep=0.05,valinit=init_eta)
freq_slider  = Slider(ax=ax_freq,  label='Input Frequency',   valmin=0, valmax=4.0, valstep=0.1, valinit=init_frequency)
noise_slider = Slider(ax=ax_noise, label='Input noise',       valmin=0, valmax=3.0, valstep=0.1, valinit=init_noise)


#Animation

def update_animation(frame):

    global x_state, y_state, omega_state, t

    current_amp= amp_slider.val
    current_mu = mu_slider.val
    current_eps = eps_slider.val
    current_eta = eta_slider.val
    current_freq = freq_slider.val
    current_noise = noise_slider.val

    sig_input = f(t,current_amp,current_freq,current_noise)
    sig_buf.append(sig_input)
    line.set_ydata(sig_buf)

    phase, x_state, y_state, omega_state = step_afo(sig_input,omega_state,x_state,y_state,current_eps,current_mu,current_eta)
    afo_buf_x.append(x_state)
    afo_buf_y.append(y_state)
    line2.set_xdata(afo_buf_x)
    line2.set_ydata(afo_buf_y)


    signal_freq_buf.append(current_freq)
    afo_freq_buf.append(omega_state)
    line3.set_ydata(signal_freq_buf)
    line4.set_ydata(afo_freq_buf)
    
    t+=dt
    

ani = FuncAnimation(fig, update_animation, interval=int(dt*1000), blit=False, cache_frame_data=False)
plt.show()


        



   