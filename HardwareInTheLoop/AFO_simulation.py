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
    return amplitude*np.sin(2*np.pi*frequency*t)+np.random.normal(0,noise)


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
time_history = deque(maxlen=100)
sig_buf = deque(maxlen=100)
afo_buf_x = deque(maxlen=100)
afo_buf_y = deque(maxlen=100)
signal_freq_buf=deque(maxlen=100)
afo_freq_buf=deque(maxlen=100)


signal_line, = ax1.plot([],[],lw=2,color='Blue')
afo_line, =ax1.plot([],[],lw=2,color='Orange')
ax1.set_ylim(-4,4)
ax1.set_title('Signal Input Plot')


afo, = ax2.plot([],[],lw=2,color='Purple')
ax2.set_xlim(-5 ,5) 
ax2.set_ylim(-3,3)
ax2.set_title('AFO Signal Plot')


input_freq, = ax3.plot([],[],color="Blue",label="Input Frequency")
afo_freq, = ax3.plot([],[],color="Red",label="AFO Frequency")
ax3.legend(loc="upper right")
ax3.set_ylim(0,6)
ax3.set_title('Frequency Convergence')






#Sliders 
ax_amp = fig.add_axes([0.25, 0.05, 0.65, 0.03])
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
    t+=dt
    time_history.append(t)

    current_amp= amp_slider.val
    current_mu = mu_slider.val
    current_eps = eps_slider.val
    current_eta = eta_slider.val
    current_freq = freq_slider.val
    current_noise = noise_slider.val

    sig_input = f(t,current_amp,current_freq,current_noise)
    sig_buf.append(sig_input)


    phase, x_state, y_state, omega_state = step_afo(sig_input,omega_state,x_state,y_state,current_eps,current_mu,current_eta)
    afo_buf_x.append(x_state)
    afo_buf_y.append(y_state)
    afo.set_data(afo_buf_x,afo_buf_y)

    signal_freq_buf.append(current_freq)
    afo_freq_buf.append(omega_state)

    input_freq.set_data(time_history,signal_freq_buf)
    afo_freq.set_data(time_history, afo_freq_buf)
    signal_line.set_data(time_history,sig_buf)
    afo_line.set_data(time_history,afo_buf_x)


    ax1.set_xlim(time_history[0],time_history[-1]+dt)
    ax1.set_ylim(-10,10)

    ax2.set_xlim(-5,5)
    ax2.set_ylim(-5,5)

    ax3.set_xlim(time_history[0],time_history[-1]+dt)
    ax3.set_ylim(0,10)
    

ani = FuncAnimation(fig, update_animation, interval=10, blit=False, cache_frame_data=False,save_count=900)
ani.save('afo_animation.mp4', writer='ffmpeg', fps=30, dpi=150)
plt.show()


        



   