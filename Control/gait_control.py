from matplotlib import pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN, KMeans
from matplotlib.animation import FuncAnimation 
import rospy
from scipy.interpolate import interp1d
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from HardwareInTheLoop import calibration 
import os

colors = ['blue', 'green']  # one per file


class Cluster:

    def __init__(self,min_dist= 0.25,max_dist=2,prev_leg_r=None,prev_leg_l=None,width_history=None,accurate=0,clump=0,occlusion=0,noise=0):
        self.min_dist=min_dist
        self.max_dist=max_dist
        self.prev_leg_r=prev_leg_r
        self.prev_leg_l=prev_leg_l


        self.accurate=accurate
        self.clump=clump
        self.occlusion=occlusion
        self.noise=noise
        
        self.width_history = width_history if width_history is not None else []



    def cluster_find(self,collisions):
        isoccluded=False
        if len(collisions)==0:
            return None, None, isoccluded
        
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

            self.accurate+=1


        elif len(unique_labels)==1:
            leg_points = collisions[labels==unique_labels[0]]

            width = np.max(leg_points[:,1])-np.min(leg_points[:,1])

            if width>0.15:
                kmeans= KMeans(n_clusters=2,n_init=10).fit(leg_points)
                centroids = kmeans.cluster_centers_
                print("Clumped Cluster Detected")
                self.clump+=1

            
            else:
                single_centroid=np.mean(leg_points,axis=0)
                print("Occlusion Detected")
                self.width_history.append(width)
                self.occlusion+=1
                isoccluded=True
                
                
                #Check which leg current cluster corresponds to
                if self.prev_leg_l is not None and self.prev_leg_r is not None:

                    dist_center_r= np.linalg.norm(single_centroid-self.prev_leg_l)
                    dist_center_l=np.linalg.norm(single_centroid-self.prev_leg_r)

                    if dist_center_r > dist_center_l:
                        print(f"Right Leg {dist_center_r} Left Leg {dist_center_l}")
                        return self.prev_leg_r, single_centroid, isoccluded
                    
                    else:
                        print(f"Right Leg {single_centroid} Left Leg {self.prev_leg_l}")
                        return  single_centroid,self.prev_leg_l, isoccluded
                    
                #If no history drop frame
                else:
                    return [1,0], [1,0], isoccluded

            

        if len (unique_labels)>2:

            if self.prev_leg_l is not None and self.prev_leg_r is not None:
                self.noise+=1
                return self.prev_leg_r,self.prev_leg_l,isoccluded
            else:
                return [1,0], [1,0], isoccluded

        
        if centroids[0][1]<centroids[1][1]:
            left_leg=(centroids[1])
            right_leg=(centroids[0])
            self.prev_leg_l=left_leg
            self.prev_leg_r=right_leg

            #plt.scatter(left_leg[0],left_leg[1])
            #plt.scatter(right_leg[0],right_leg[1])
        else:
            
            left_leg=(centroids[0])
            right_leg=(centroids[1])
            self.prev_leg_l=left_leg
            self.prev_leg_r=right_leg


        print(f"Right Leg {right_leg} Left Leg {left_leg}")
        return left_leg,right_leg,isoccluded
        

    #Collision Scanner
    def process_scan(self,angle_min,angle_max,angle_increment,ranges,angle_offset=0): 
            collisions = []
            for i in range(0,200):
                #print(position)
                #print(walker_angle)
                if ranges[i] == float('Inf'):
                    continue
                if ranges[i] > self.min_dist and ranges[i] < self.max_dist:
                    angle = angle_min + i * angle_increment + angle_offset
                    dx = ranges[i] * np.cos(angle)
                    dy = ranges[i] * np.sin(angle)
                    collisions.append((dx, dy))
            collisions=np.array(collisions)
            return collisions



class SignalProcessor:
    'Handles Signal Processing of LiDAR and Leg detection'
    def __init__(self,cal_window=None,scissor_window=None,right=None,left=None,encoder_velocity=None,true_timestamp=None,stride_window=None,avg_position_history=None,cal_encoder_velocity=None,prev_scissor=0,prev_avg=0,prev_left=0,prev_right=0):
        self.cal_window = cal_window if cal_window is not None else []
        self.scissor_window=scissor_window if scissor_window is not None else []
        self.right=right if right is not None else []
        self.left=left if left is not None else []
        self.stride_window= stride_window if stride_window is not None else []
        self.encoder_velocity=encoder_velocity if encoder_velocity is not None else []
        self.true_timestamp=true_timestamp if true_timestamp is not None else []
        self.avg_position_history =avg_position_history if avg_position_history is not None else []
        self.cal_encoder_velocity = cal_encoder_velocity if cal_encoder_velocity is not None else []

        self.prev_scissor=prev_scissor
        self.prev_avg=prev_avg
        self.prev_left=prev_left
        self.prev_right=prev_right

    

    def offline_data(self,left_current,right_current,time,encoder):

        if left_current is None or right_current is None:
            left_raw = self.prev_left
            right_raw = self.prev_right
            scissor_signal = self.prev_scissor
            avg_position=self.prev_avg

            self.left.append(left_raw)
            self.right.append(right_raw)
            self.scissor_window.append(scissor_signal)
            self.avg_position_history.append(avg_position)
        else:
            left_raw = left_current 
            right_raw = right_current
            scissor_signal=left_current-right_current
            avg_position=(left_current+right_current)/2

            self.left.append(left_current)
            self.right.append(right_current)
            self.scissor_window.append(left_current-right_current)
            self.avg_position_history.append(avg_position)


        if encoder is not None:
            self.encoder_velocity.append(encoder)
            self.cal_encoder_velocity.append(encoder)
        else:
            self.encoder_velocity.append(None)
            self.cal_encoder_velocity.append(None)
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
        omegadot= (-self.eta*signal*self.y)/r

        self.x= xdot * self.dt + self.x
        self.y=ydot * self.dt + self.y

        self.omega= np.clip(self.omega + omegadot * self.dt,0.3,10.0)


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
            return 0
        
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
            #print(last_stride)
            #last_stride = np.clip(last_stride,0.3,1.5)
            self.stride_history.append(last_stride)
            return last_stride
        
        return None
    
    def velocity_command(self,feedback_velocity,cadence,last_stride,velocity_gain):
        velocity_command = cadence*last_stride*velocity_gain + feedback_velocity
        return velocity_command
   

encoder = None
def odom_callback(msg:Odometry):
    global encoder
    encoder=msg 

current_scan=None
def scan_callback(msg:LaserScan):
        global current_scan
        current_scan=msg


if __name__== "__main__":
    rospy.init_node('Adaptive_Frequency_Oscillator')

    sampling_frequency= 10
    rate=rospy.Rate(sampling_frequency)

    #Initialization of Motors,Encoder and LiDAR
    pub_shutdown = rospy.Publisher('/shutdown',Bool,queue_size=1)
    encoder_sub=rospy.Subscriber('/odom',Odometry,odom_callback,queue_size=1)
    laser_scan_sub = rospy.Subscriber("/scan_legs_filtered",LaserScan,scan_callback,queue_size=1)
    rospy.sleep(1.0)
    rospy.loginfo('Motors, LiDAR, and Encoder Enabled.')

    pub_right_motor = rospy.Publisher('/right_wheel_velocity', Float64, queue_size=10)
    pub_left_motor = rospy.Publisher('/left_wheel_velocity', Float64, queue_size=10)

    start_time = rospy.Time.now().to_sec()


    sensor=Cluster()
    signal = SignalProcessor()
    oscillator = AdaptiveFrequencyOscillator(sampling_frequency)
    pd_controller = PDController(sampling_frequency, x_d=0)
    walker = WalkerController()


    print("=" * 50)
    input("Press enter to begin session")
    print("Controller engaged. Please Begin Walking Calibrating Your Walking Style.")
    print("=" * 50)

    cal_time=[]
    cal_velocity=[]
    calibrated=False
    velocity_gain = 1.0

    commanded_timestamps=[]
    velocity_history=[]

    
    while not rospy.is_shutdown():
        if current_scan == None:
            rate.sleep()
            continue

        if encoder == None:
            rate.sleep()
            continue


        current_time = rospy.Time.now().to_sec()- start_time
        encoder_velocity=encoder.twist.twist.linear.x


        collisions = sensor.process_scan(current_scan.angle_min,current_scan.angle_max,current_scan.angle_increment,current_scan.ranges)
        raw_left,raw_right, isoccluded= sensor.cluster_find(collisions)
        if raw_left is not None and raw_right is not None and encoder_velocity is not None:
            
            left,right,scissor_signal,pelvis = signal.offline_data(raw_left[0],raw_right[0],current_time,encoder_velocity)

            phase,cadence=oscillator.step_afo(scissor_signal)
            feedback_velocity=pd_controller.pd_controller(pelvis)
            walker.stride_window.append(scissor_signal)
            last_stride = walker.last_stride(walker.stride_window)


            if calibrated== False:
                if len(signal.scissor_window) == 100:

                    cal,x_d,velocity_gain= calibration.calibration(signal.right,signal.left,signal.scissor_window,sampling_frequency,signal.cal_encoder_velocity,current_omega=oscillator.omega)
                    
                    if cal == True: 
                        calibrated=True
                        print("Calibration complete")
                        input("Press any key to continue")
                    else:
                        signal.left.clear()
                        signal.right.clear()
                        signal.scissor_window.clear()
                        signal.cal_encoder_velocity.clear()
                        signal.true_timestamp.clear()
                        cal_velocity.clear()
                        cal_time.clear()
                        cal_velocity = list(signal.cal_encoder_velocity)
                        cal_time = list(signal.true_timestamp)

            else:
                if last_stride is not None and feedback_velocity is not None:
                    velocity_command = walker.velocity_command(feedback_velocity,cadence,last_stride,velocity_gain)

                    commanded_timestamps.append(current_time)
                    velocity_history.append(velocity_command)

        else:
            rate.sleep()
            continue

        rate.sleep()

    pub_shutdown.publish(True)



    commanded_timestamps=np.array(commanded_timestamps)
    true_velocity=np.array(cal_velocity)
    cal_time=np.array(cal_time)
    velocity_history=np.array(velocity_history)

    #Velocity error
    print(f"\n---Control System Gait Analysis ---")
    print(f"Predicted mean: {np.mean(true_velocity):.3f} m/s")
    print(f"True mean:      {np.nanmean(true_velocity):.3f} m/s")
    print(f"True std:       {np.nanstd(true_velocity):.3f}")

    start_idx=len(true_velocity)-len(velocity_history)

    rmse=np.sqrt(np.mean(np.square(velocity_history-true_velocity[start_idx:])))

    print(f"RMSE Predicted to True Velocity:       {rmse:.3f}")
        
    plt.plot(commanded_timestamps, velocity_history, color='blue', label=f'Calculated Velocity')
    plt.plot(cal_time, true_velocity, color='red', linestyle='--', label=f' True Velocity During Calibration')
    plt.plot()

    # Y-label goes on every graph
    plt.ylabel("velocity (m/s)")
    plt.legend(loc="upper right")
    plt.ylim(-0.5, 2.0)

    plt.xlabel("time (s)")

    plt.tight_layout() 
    plt.show()
