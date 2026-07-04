import numpy as np
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN, KMeans
from Control import calibration


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

    '''def lowpass_filter(self,data,cutoff,fs,order=4):
            nyq = 0.5 * fs
            normal_cutoff = cutoff / nyq
            b, a = butter(order, normal_cutoff, btype='low')
            return filtfilt(b, a, data)'''


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
    #Calculate Frequency of signal

    def __init__(self,sampling_frequency,eta=0.3,eps=1.5,mu=1):
        self.sampling_frequency=sampling_frequency
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
    
    def velocity_command(self,cadence,last_stride,velocity_gain):
        velocity_command = cadence*last_stride*velocity_gain
        velocity_command = np.clip(velocity_command,0,1.2)
        return velocity_command
    
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


        #Thighs too close together
        elif len(unique_labels)==1:
            leg_points = collisions[labels==unique_labels[0]]
            width = np.max(leg_points[:,1])-np.min(leg_points[:,1])

            if width>0.30:
                '''kmeans= KMeans(n_clusters=2,n_init=10).fit(leg_points)
                centroids = kmeans.cluster_centers_'''            
                left_leg = leg_points[:len(leg_points)//2]
                right_leg = leg_points[len(leg_points)//2:]

                centroids.append(np.mean(left_leg,axis=0))
                centroids.append(np.mean(right_leg,axis=0))

                print(f"Clumped Cluster Detected Right leg:{right_leg} Left Leg:{left_leg}")


        #Occlusion
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
                        print(f"Right leg occluded: Right Leg {dist_center_r} Left Leg {dist_center_l}")
                        return  single_centroid, self.prev_leg_r, isoccluded
                    
                    else:
                        print(f"Left Leg Occluded: Right Leg {single_centroid} Left Leg {self.prev_leg_l}")
                        return  self.prev_leg_l,single_centroid, isoccluded
                    
                #If no history drop frame
                else:
                    return None, None, isoccluded


        if len (unique_labels)>2:

            if self.prev_leg_l is not None and self.prev_leg_r is not None:
                self.noise+=1
                return self.prev_leg_l,self.prev_leg_r,isoccluded
            else:
                return [-1,0], [-1,0], isoccluded
            
        if len(unique_labels) == 0:
            return self.prev_leg_l, self.prev_leg_r, isoccluded

        else:
            if centroids[0][1]<centroids[1][1]:
                left_leg=(centroids[1])
                right_leg=(centroids[0])
                self.prev_leg_l=left_leg
                self.prev_leg_r=right_leg

            else:  
                left_leg=(centroids[0])
                right_leg=(centroids[1])
                self.prev_leg_l=left_leg
                self.prev_leg_r=right_leg


        print(f"Right Leg {right_leg} Left Leg {left_leg}")
        return left_leg,right_leg,isoccluded
        

    #Collision Scanner
    def process_scan(self, angle_min, angle_increment, ranges, angle_offset=0):
        """
        Processes LiDAR scan data to extract collision points within a specified distance range.

        Args:
            angle_min (float): The starting angle of the scan.
            angle_increment (float): The angular distance between measurements.
            ranges (list or np.ndarray): The distance data from the LiDAR.
            angle_offset (float, optional): An additional angle offset to apply. Defaults to 0.

        Returns:
            np.ndarray: Array of (x, y) collision points.
        """
        collisions = []
        for i in range(0, 200):
            #if ranges[i] == float('Inf'):
                #continue
            if ranges[i] > self.min_dist and ranges[i] < self.max_dist:
                angle = angle_min + i * angle_increment + angle_offset
                dx = ranges[i] * np.cos(angle)
                dy = ranges[i] * np.sin(angle)
                collisions.append((dx, dy))
        collisions = np.array(collisions)
        return collisions
    
class main_loop:
    def __init__(self,fs=10, wheel_radius=0.1143):

        #1 Physics Parameters
        self.fs = fs
        self.wheel_radius = wheel_radius
        self.delta_v = 0.3 / wheel_radius
        self.current_velocity = 0        
        self.cadence = None
        self.prev_cadence = 1.0

        #Calibration
        self.calibrated = False
        self.velocity_gain = 1.0
        self.cal_stride = None

        #Classes
        self.signal = SignalProcessor()
        self.oscillator = AdaptiveFrequencyOscillator(fs)
        self.walker = WalkerController()
        self.cluster = Cluster()


        #Data Arrays
        self.commanded_timestamps = []
        self.velocity_history = []
        self.encoder_data = []
        self.encoder_time = []
        self.control_state = []  

    def step_from_legs(self, current_time, encoder_velocity, left_x, right_x, isoccluded):
        if left_x is not None and right_x is not None and encoder_velocity is not None:
            left,right,scissor_signal,pelvis = self.signal.offline_data(left_x,right_x,current_time,encoder_velocity)

            if isoccluded == True:
                self.phase,self.prev_cadence= self.oscillator.step_afo(scissor_signal)
            else:
                self.phase,self.cadence= self.oscillator.step_afo(scissor_signal)

            if self.cadence is not None:
                self.prev_cadence = self.cadence

            self.walker.stride_window.append(scissor_signal)
            self.last_stride = self.walker.last_stride(self.walker.stride_window)

            if not self.calibrated:
                if len(self.signal.scissor_window) == 150:
                    self.cal,self.x_d,self.velocity_gain,self.cal_stride= calibration.calibration(self.signal.right,self.signal.left,self.signal.scissor_window,self.fs,self.signal.cal_encoder_velocity,current_omega=self.oscillator.omega)
                    
                    if self.cal == True: 
                        self.calibrated=True
                        print("Calibration complete")
                        
                    else:
                        self.cal_velocity = list(self.signal.cal_encoder_velocity)
                        self.cal_time = list(self.signal.true_timestamp)
                        self.signal.left.clear()
                        self.signal.right.clear()
                        self.signal.scissor_window.clear()
                        self.signal.cal_encoder_velocity.clear()
                        self.signal.true_timestamp.clear()
                        self.cal_velocity.clear()
                        self.cal_time.clear()
                return None
            else:
                if self.last_stride == None:
                    feedforward_velocity = self.walker.velocity_command(self.cadence,self.cal_stride,self.velocity_gain)
                else:
                    feedforward_velocity = self.walker.velocity_command(self.cadence,self.last_stride,self.velocity_gain) 

                if -1.0 < pelvis < -0.558:
                    attenuation_factor=attenuation(pelvis,-1.0,-0.558)
                    linear_velocity =  attenuation_factor * feedforward_velocity
                    self.control_state.append(1)

                elif -0.254 < pelvis < 0:
                    attenuation_factor=attenuation(pelvis,-0.254,0) 

                    linear_velocity =  attenuation_factor * feedforward_velocity
                    self.control_state.append(1)

                elif -0.558 < pelvis < -0.254:
                    linear_velocity = feedforward_velocity
                    self.control_state.append(2)

                else:
                    linear_velocity = 0 
                    self.control_state.append(3)  

                wheel_velocity = linear_velocity/self.wheel_radius
                wheel_velocity = np.clip(wheel_velocity,0,(3/self.wheel_radius))
                if (wheel_velocity - self.current_velocity) > self.delta_v :
                    wheel_velocity = self.current_velocity + self.delta_v

                elif (wheel_velocity - self.current_velocity) < -self.delta_v :
                    wheel_velocity = self.current_velocity-self.delta_v

                self.current_velocity = wheel_velocity
                self.velocity_history.append(wheel_velocity)
                self.commanded_timestamps.append(current_time)
                self.encoder_data.append(encoder_velocity/self.wheel_radius)
                self.encoder_time.append(current_time)

                return wheel_velocity
                
# --- Returns a value between 0 and 1 given a pelvis value between error max and error min ---
def attenuation(pelvis,error_max,error_min):
    return float(np.interp(pelvis,[error_max,error_min],[0,1.0]))



