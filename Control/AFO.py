from matplotlib import pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN, KMeans
from matplotlib.animation import FuncAnimation 
import time
import rospy
import scipy
from scipy.signal import savgol_filter, find_peaks
from scipy.interpolate import interp1d
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool, Float64
from geometry_msgs.msg import Twist





def step_afo(signal,omega0,x0,y0,eps,mu,eta):
    r=np.sqrt(x0**2 +y0**2)
    xdot=(mu-np.square(r))*x0 - omega0 * y0 + eps*signal
    ydot=(mu-np.square(r))*y0 + omega0 * x0
    omegadot= -eta*signal*y0/r

    x_new=xdot*dt+x0
    y_new=ydot*dt+y0


    omega= (omega0 + omegadot * dt)

    phase = (np.arctan2(y_new,x_new) + np.pi)/(2*np.pi)

    return phase, x_new, y_new, omega





def cluster_find(collisions,prev_leg_r=None,prev_leg_l=None):
    isoccluded=False
    if len(collisions)==0:
        return None, None
    
    cluster=DBSCAN(eps=4e-2,min_samples=3).fit(collisions)
    labels=cluster.labels_
    unique_labels = [l for l in np.unique(labels) if l != -1]
    print(f"Number of clusters found: {len(unique_labels)}")
    centroids=[]

    #Ideal Case (2 Clusters)
    if len(unique_labels)==2:   
        for index in unique_labels:
            leg_points= collisions[labels==index]
            centroid= (np.mean(leg_points,axis=0))
            centroids.append(centroid)
            global accurate
            accurate+=1


    elif len(unique_labels)==1:
        leg_points = collisions[labels==unique_labels[0]]

        width = np.max(leg_points[:,1])-np.min(leg_points[:,1])

        if width>0.15:
            kmeans= KMeans(n_clusters=2,n_init=10).fit(leg_points)
            centroids = kmeans.cluster_centers_
            print("Clumped Cluster Detected")
            global clump
            clump+=1

        
        else:
            single_centroid=np.mean(leg_points,axis=0)
            print("Occlusion Detected")
            global occlusion
            width_history.append(width)
            occlusion+=1
            isoccluded=True
            
            
            #Check which leg current cluster corresponds to
            if prev_leg_l is not None and prev_leg_r is not None:

                dist_center_r= np.linalg.norm(single_centroid-prev_leg_l)
                dist_center_l=np.linalg.norm(single_centroid-prev_leg_r)

                if dist_center_r > dist_center_l:
                    print(f"Right Leg {dist_center_r} Left Leg {dist_center_l}")
                    return prev_leg_r, single_centroid, isoccluded
                
                else:
                    print(f"Right Leg {single_centroid} Left Leg {prev_leg_l}")
                    return  single_centroid,prev_leg_l, isoccluded
                
            #If no history drop frame
            else:
                return [1,0], [1,0], isoccluded

        

    if len (unique_labels)>2:
        global noise
        noise+=1
        return prev_leg_r,prev_leg_l,isoccluded

    
    if centroids[0][1]<centroids[1][1]:
        left_leg=(centroids[1])
        right_leg=(centroids[0])

        #plt.scatter(left_leg[0],left_leg[1])
        #plt.scatter(right_leg[0],right_leg[1])
    if centroids[0][1]>centroids[1][1]:
        left_leg=(centroids[0])
        right_leg=(centroids[1])


    print(f"Right Leg {right_leg} Left Leg {left_leg}")
    return left_leg,right_leg,isoccluded

#Collision Scanner
def process_scan(angle_min,angle_max,angle_increment,range_min,range_max,ranges,angle_offset=0): 
        collisions = []
        for i in range(0,200):
            #print(position)
            #print(walker_angle)
            if ranges[i] == float('Inf'):
                continue
            if ranges[i] > range_min and ranges[i] < range_max:
                angle = angle_min + i * angle_increment + angle_offset
                dx = ranges[i] * np.cos(angle)
                dy = ranges[i] * np.sin(angle)
                collisions.append((dx, dy))
        collisions=np.array(collisions)
        return collisions


def leg_frequency(leg_data):

    signal=leg_data-np.mean(leg_data)
    fs=8  
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



current_scan = None
def scan_callback(msg:LaserScan):
        global current_scan
        current_scan=msg


if __name__== "__main__":
    rospy.init_node('Adaptive_Frequency_Oscillator')


    #Initial Variables 
    min_dist=0.25
    max_dist=2
    sampling_frequency=10
    mu=1
    omega0=2
    eta=6
    eps=4
    dt=1/sampling_frequency
    t=0
    x_r,y_r=1,0
    x_l,y_l=-1,0
    phi=[]
    signal=[]
    phase=[]


    #Clustering
    occlusion=0
    clump=0
    accurate=0
    noise=0
    width_history=[]

    prev_right=None
    prev_left=None

    #Graphing
    log_time=[]
    log_x_left=[]
    log_x_right=[]
    log_occlusions=[]
    start_time=rospy.get_time()
    current_time=0




    #Filter Variables
    right_leg_centroid=[]
    left_leg_centroid=[]
    left_buffer=[]
    right_buffer=[]



    #Calibration Variables
    gold_calibration=False
    peak_x_l=[]
    peak_x_r=[]
    calibrated_x_r=[]
    calibrated_x_l=[]
    std_r=[]
    std_l=[]
    all_timestamps=[]

    #Motor Control
    prev_phase_r=0
    prev_phase_l=0
    wheel_radius=0.3
    initial_velocity=0.0

    #Spring Damping
    beta=2.0
    k=5.0
    m=4
    x_target=0.4
    x_current=0.0


    freq_r_hist=[]
    freq_l_hist=[]
    target_velocity=0




    rate=rospy.Rate(sampling_frequency)
    laser_scan_sub = rospy.Subscriber("/scan_legs_filtered",LaserScan,scan_callback,queue_size=10)
    pub_signal=rospy.Publisher('leg_data', Float32, queue_size=10)
    pub_phi=rospy.Publisher('phase_data',Float32,queue_size=10)

    #Initialization of motors
    pub_shutdown = rospy.Publisher('/shutdown',Bool,queue_size=10)
    rospy.sleep(1.0)
    rospy.loginfo('Motors Enabled.')

    pub_right_motor = rospy.Publisher('/right_wheel_velocity', Float64, queue_size=10)
    pub_left_motor = rospy.Publisher('/left_wheel_velocity', Float64, queue_size=10)
    


    
    

    while not rospy.is_shutdown():

        if current_scan == None:
            rate.sleep()
            continue

        collisions = process_scan(current_scan.angle_min,current_scan.angle_max,current_scan.angle_increment,min_dist,max_dist,current_scan.ranges)
        raw_left,raw_right, isoccluded= cluster_find(collisions,prev_right,prev_left)

        log_x_right.append(raw_right[0])
        log_x_left.append(raw_left[0])
        log_occlusions.append(isoccluded)

        prev_right=raw_right
        prev_left=raw_left

        if not gold_calibration:

            #3 Median Buffer for Outliers
            if raw_left is None or raw_right is None:
                

                rate.sleep()
                continue
            else:
                right_buffer.append(raw_right)
                left_buffer.append(raw_left)

                if len(right_buffer)> 3:
                    left_buffer.pop(0)
                    right_buffer.pop(0)

                if len(right_buffer) == 3: 
                    clean_right=np.median(right_buffer,axis=0)
                    clean_left=np.median(left_buffer,axis=0)

                    right_leg_centroid.append(clean_right)
                    left_leg_centroid.append(clean_left)

                    all_timestamps.append(rospy.get_time())

            if len(right_leg_centroid)>50:

                right_arr = np.array(right_leg_centroid)
                left_arr= np.array(left_leg_centroid)

                right_leg_smooth= savgol_filter(right_arr[:,0],window_length=5,polyorder=3)
                left_leg_smooth= savgol_filter(left_arr[:,0],window_length=5,polyorder=3)


                peak_x_r,_= find_peaks(right_leg_smooth,prominence=0.05)
                valley_x_r,_= find_peaks(-right_leg_smooth,prominence=0.05)

                peak_x_l,_= find_peaks(left_leg_smooth,prominence=0.05)
                valley_x_l,_= find_peaks(-left_leg_smooth,prominence=0.05)


                if len(peak_x_r)<3 or len(peak_x_l)<3:
                    print("Not enough data points found keep walking")
                    right_leg_centroid=[]
                    left_leg_centroid=[]
                    rate.sleep()
                    continue

                else:
                    normalized_x_l=[]
                    normalized_x_r=[]


                for index in range(len(peak_x_r)-1):
                    start_idx=peak_x_r[index]
                    end_idx=peak_x_r[index+1]

                    step_data=right_leg_smooth[start_idx:end_idx]

                    raw_time=np.linspace(0,1,len(step_data))

                    interp_r= interp1d(raw_time,step_data,kind="cubic")
                    normalized_time=np.linspace(0,1,100)
                    stretched_step1= interp_r(normalized_time)
                    
                    normalized_x_r.append(stretched_step1)


                for index in range(len(peak_x_l)-1):
                    start_idx=peak_x_l[index]
                    end_idx=peak_x_l[index+1]

                    step_data=left_leg_smooth[start_idx:end_idx]

                    raw_time=np.linspace(0,1,len(step_data))

                    interp_l= interp1d(raw_time,step_data,kind="cubic")
                    normalized_time=np.linspace(0,1,100)
                    stretched_step2= interp_l(normalized_time)
                    
                    normalized_x_l.append(stretched_step2)

                normalized_x_r=np.array(normalized_x_r)
                normalized_x_l=np.array(normalized_x_l)

                std_r= np.std(normalized_x_r,axis=0)
                std_l= np.std(normalized_x_l,axis=0)            
                
                std_avg1=np.mean(std_r)
                std_avg2=np.mean(std_l)

                if std_avg1> 0.5 or std_avg2>0.5:
                    #Clear data 
                    right_leg_centroid=[]
                    left_leg_centroid=[]
                    continue
                else:
                        gold_calibration=True

                        calibrated_x_r= np.mean(normalized_x_r,axis=0)
                        calibrated_x_l= np.mean(normalized_x_l,axis=0)
                        offset_r= np.mean(right_leg_smooth)
                        offset_l=np.mean(left_leg_smooth)
                        input("Calibration Complete, press ENTER to begin session")

                        target_hz_r=leg_frequency(right_leg_smooth)
                        target_hz_l=leg_frequency(left_leg_smooth)
                        omega0_r= 2*np.pi*target_hz_r
                        omega0_l=2*np.pi*target_hz_l
                        
                        pub_shutdown.publish(False)
                        rospy.loginfo("Handshake Complete, motors engaged for assistance")




        elif raw_right is None or raw_left is None:
            target_velocity= 0
            pub_left_motor.publish(Float64(target_velocity))
            pub_right_motor.publish(Float64(target_velocity))
            rate.sleep()
            continue

        else:
            centered_signal_r=raw_right[0]-offset_r
            centered_signal_l=raw_left[0]-offset_l
            phase_r,x_r,y_r,omega_r=step_afo(centered_signal_r,omega0_r,x_r,y_r,eps,mu,eta)
            phase_l,x_l,y_l,omega_l=step_afo(centered_signal_l,omega0_l,x_l,y_l,eps,mu,eta)


            if min(raw_right[0],raw_left[0])/2 == raw_right[0]:
                target_velocity=((k*(x_target-raw_right[0])*dt)+m*initial_velocity)/(m+beta*dt)

                if target_velocity > 10:
                    target_velocity = 10
                if target_velocity < 0:
                    target_velocity = 0

                pub_left_motor.publish(Float64(target_velocity))
                pub_right_motor.publish(Float64(target_velocity))
                initial_velocity=target_velocity
            else:
                target_velocity=((k*(x_target-raw_left[0])*dt)+m*initial_velocity)/(m+beta*dt)
                pub_left_motor.publish(Float64(target_velocity))
                pub_right_motor.publish(Float64(target_velocity))
                initial_velocity=target_velocity

                


            prev_phase_r=phase_r
            prev_phase_l=phase_l   
            omega0_r=omega_r
            omega0_l=omega_l

            freq_r_hist.append(omega0_r)
            freq_l_hist.append(omega0_l)

        rate.sleep()
    pub_shutdown.publish(True)
    rospy.loginfo("Safety Shutdown Sent.")

    plt.figure(1)
    plt.scatter(np.arange(len(freq_r_hist))/sampling_frequency,freq_r_hist)
    plt.ylabel("Right")

    plt.figure(2)
    plt.scatter(np.arange(len(freq_l_hist))/sampling_frequency,freq_l_hist)
    plt.ylabel('Left')

    plt.figure(3)
    plt.scatter(np.arange(len(width_history)),width_history)

    plt.figure(4)
    plt.plot(np.arange(len(log_x_left)),log_x_left,label="Left Leg x")
    plt.plot(np.arange(len(log_x_right)),log_x_right,label="Right Leg x")
