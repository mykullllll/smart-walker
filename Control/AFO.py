import numpy as np
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN, KMeans
try:
    from . import calibration
except ImportError:
    import calibration
from collections import deque


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


    def offline_data(self, left_current, right_current, time, encoder, alpha_scissor=0.35, alpha_pelvis=0.6):
        max_leg_jump = 0.20

        if left_current is None or right_current is None:
            left_raw = self.prev_left
            right_raw = self.prev_right
            scissor_signal = self.prev_scissor
            avg_position = self.prev_avg
        else:
            if len(self.left) > 0:
                if abs(left_current - self.prev_left) > max_leg_jump:
                    left_current = self.prev_left + np.sign(left_current - self.prev_left) * max_leg_jump
                if abs(right_current - self.prev_right) > max_leg_jump:
                    right_current = self.prev_right + np.sign(right_current - self.prev_right) * max_leg_jump

            left_raw = left_current
            right_raw = right_current

            raw_scissor = left_raw - right_raw
            raw_avg = (left_raw + right_raw) / 2

            if len(self.scissor_window) == 0:
                scissor_signal = raw_scissor
                avg_position = raw_avg
            else:
                scissor_signal = self.prev_scissor + alpha_scissor * (raw_scissor - self.prev_scissor)
                avg_position = self.prev_avg + alpha_pelvis * (raw_avg - self.prev_avg)

        self.left.append(left_raw)
        self.right.append(right_raw)
        self.scissor_window.append(scissor_signal)
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

        self.prev_scissor = scissor_signal
        self.prev_avg = avg_position
        self.prev_left = left_raw
        self.prev_right = right_raw

        return left_raw,right_raw,scissor_signal,avg_position

class AdaptiveFrequencyOscillator:
    #Calculate Frequency of signal

    def __init__(self,sampling_frequency,eta,eps,mu=1):
        self.sampling_frequency=sampling_frequency
        self.x=0
        self.y=1.0
        self.omega=2*np.pi*1.0
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

        self.omega= np.clip(self.omega + omegadot * self.dt,0.3,20.0)

        phase = ((np.arctan2(self.y,self.x)) /(2*np.pi)) % 1.0
        cadence=self.omega/(2*np.pi)

        return phase, cadence, omegadot
        
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
        self.occlusions_length=0
        
        self.width_history = width_history if width_history is not None else []



    def cluster_find(self,collisions):
        isoccluded=False
        
        if len(collisions)==0:
            isoccluded = True
            self.occlusions_length += 1
            if self.occlusions_length >= 20:
                return None, None, isoccluded,True
            return None, None, isoccluded,False
        
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
            self.occlusions_length = 0
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

        #Occlusion
            else:
                single_centroid=np.mean(leg_points,axis=0)
                self.width_history.append(width)
                self.occlusion+=1
                isoccluded=True

                if isoccluded:
                    self.occlusions_length+=1
                else:
                    self.occlusions_length=0

                if self.occlusions_length >= 20:
                    return None, None, isoccluded,True
                
                #Check which leg current cluster corresponds to
                if self.prev_leg_l is not None and self.prev_leg_r is not None:

                    dist_center_r= np.linalg.norm(single_centroid-self.prev_leg_l)
                    dist_center_l=np.linalg.norm(single_centroid-self.prev_leg_r)

                    if dist_center_r > dist_center_l:
                        return  self.prev_leg_l, single_centroid, isoccluded,False
                    
                    else:
                        return  single_centroid,self.prev_leg_r, isoccluded,False
                    
                #If no history drop frame
                else:
                    return None, None, isoccluded,False


        if len (unique_labels)>2:

            if self.prev_leg_l is not None and self.prev_leg_r is not None:
                self.noise+=1
                return self.prev_leg_l,self.prev_leg_r,isoccluded,False
            else:
                return None, None, isoccluded, False
            
        if len(unique_labels) == 0:
            isoccluded = True
            self.occlusions_length += 1
            if self.occlusions_length >= 20:
                return None, None, isoccluded,True
            return self.prev_leg_l, self.prev_leg_r, isoccluded, False

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


        return left_leg,right_leg,isoccluded,False
        

    #Collision Scanner
    def process_scan(self, angle_min, angle_increment, ranges, angle_offset=0):
        """
        Processes LiDAR scan data to extract collision points within a specified distance range.
        Vectorized for high-speed robotics execution.
        """
        # 1. Convert to numpy array dynamically if it isn't already
        ranges = np.asarray(ranges)
        
        # 2. Dynamically calculate all angles across the entire array (No hardcoding!)
        angles = angle_min + np.arange(len(ranges)) * angle_increment + angle_offset
        
        # 3. Create a strict boolean mask to isolate valid leg points
        # Strips out NaNs, Infs, and enforces your distance boundaries
        valid_mask = (
            np.isfinite(ranges) & 
            (~np.isnan(ranges)) & 
            (ranges > self.min_dist) & 
            (ranges < self.max_dist)
        )
        
        # 4. Extract only the valid data payloads
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # If no points match (e.g. empty room or out of box), return empty array safely
        if len(valid_ranges) == 0:
            return np.empty((0, 2))
            
        # 5. Vectorized Trigonometry (Calculates all X and Y coordinates instantly)
        dx = valid_ranges * np.cos(valid_angles)
        dy = valid_ranges * np.sin(valid_angles)
        
        # 6. Stack into a clean (N, 2) array of XY coordinates
        collisions = np.column_stack((dx, dy))
        
        return collisions
    
class main_loop:
    def __init__(self,fs=10, wheel_radius=0.1143):

        #1 Physics Parameters
        self.fs = fs
        self.wheel_radius = wheel_radius
        self.max_accel = 0.4  #m/s^2
        self.max_decel = 0.8  #m/s^2
        self.stop_accel = 1.2
        self.attenuation_decel = 1.0  #m/s^2 -- faster than max_decel, slower than stop_accel
        self.pos_delta_v = (self.max_accel/self.fs) / self.wheel_radius
        self.neg_delta_v = (self.max_decel/self.fs) / self.wheel_radius
        self.stop_delta_v = (self.stop_accel/self.fs) / self.wheel_radius
        self.attenuation_delta_v = (self.attenuation_decel/self.fs) / self.wheel_radius
        self.current_velocity = 0
        self.cadence = None
        self.prev_cadence = 1.0


        #Freeze Detection
        self.freeze_window = deque()
        self.freeze_detection_time = 1.2
        self.freeze_window_duration = (
            self.freeze_detection_time + 1.0 / self.fs
        )

        self.freeze_motion_threshold = 0.025
        self.freeze_detection_armed = False
        self.ramp_complete_time = None
        self.freeze_arm_delay = 2.0

        #Calibration
        self.calibrated = False
        self.velocity_gain = 1.0
        self.cal_stride = None
        self.time_to_cal = 0

        #Classes
        self.signal = SignalProcessor()
        self.oscillator = AdaptiveFrequencyOscillator(fs,eta=5.5,eps=8.5)
        self.walker = WalkerController()
        self.cluster = Cluster()


        #Data Arrays
        self.commanded_timestamps = []
        self.velocity_history = []
        self.encoder_data = []
        self.encoder_time = []
        self.control_state = []  
        self.cadence_history=[]
        self.pelvis_history=[]
        self.freeze_detected_time_history= []
        self.stride_used_history = []
        self.target_wheel_history = []

        #Ramp of AFO
        self.afo_enabled = False
        self.assist_ramping = False

        #Frozen Gait Metrics
        self.prev_freeze_detected = False

    def step_from_legs(self, current_time, encoder_velocity, left_x, right_x, isoccluded):
        if left_x is not None and right_x is not None and encoder_velocity is not None:
            left,right,scissor_signal,pelvis = self.signal.offline_data(left_x,right_x,current_time,encoder_velocity)
            self.pelvis_history.append(pelvis)
            self.encoder_data.append(encoder_velocity)
            self.encoder_time.append(current_time)

            if not self.calibrated:
                calibration_samples = int(self.fs * 15)
                if len(self.signal.scissor_window) >= calibration_samples and not self.calibrated:
                    self.cal,self.x_d,self.velocity_gain,self.cal_stride,self.raw_frequency,self.time_to_cal= calibration.calibration(self.signal.right,self.signal.left,self.signal.scissor_window,self.fs,self.signal.cal_encoder_velocity,current_omega=self.oscillator.omega, wheel_radius=self.wheel_radius,timestamps=self.signal.true_timestamp)
                    
                    if self.cal == True: 
                        self.calibrated=True
                        print("Calibration complete")
                        self.cadence = self.raw_frequency
                        self.prev_cadence = self.raw_frequency
                        self.oscillator.omega = 2 * np.pi * self.raw_frequency
                        self.cal_raw=self.raw_frequency*self.cal_stride
                        self.previous_stride = self.cal_stride


                        self.assist_ramping = True
                        self.afo_enabled = False
                        self.current_velocity = 0
                        
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
                        self.walker.stride_window.clear()
                        self.walker.stride_history.clear()
                        self.freeze_window.clear()
                return None,None
            



            # before AFO update
            cadence_update_zone = -0.4556 < pelvis < -0.3556

            if (
                not self.assist_ramping
                and self.ramp_complete_time is not None
                and current_time - self.ramp_complete_time >= self.freeze_arm_delay
            ):
                self.freeze_detection_armed = True

            pelvis_safe = -0.60 < pelvis < -0.28

            # Use the unsmoothed leg positions for responsive freeze detection.
            if not self.freeze_detection_armed or isoccluded or not pelvis_safe:
                self.freeze_window.clear()
                freeze_detected = False

            else:
                raw_scissor = left - right
                self.freeze_window.append((current_time, raw_scissor))

                #Remove Measurements older than 0.35 sec
                while (
                    self.freeze_window
                    and current_time - self.freeze_window[0][0]
                    > self.freeze_window_duration
                ):
                    self.freeze_window.popleft()

                window_age = current_time - self.freeze_window[0][0]

                #Measure how much the legs moved in the past 0.35 sec
                motion_range = np.ptp([
                    value for _, value in self.freeze_window
                ])

                enough_history = window_age >= self.freeze_detection_time

                #Freeze Detection Conditions - 0.3 secondds of data frozen, legs moved less than 2.5 cm relative to each other, The person is inside pelvis range, Both legs are visible. 
                freeze_detected = (
                    enough_history
                    and motion_range < self.freeze_motion_threshold
                )


            
            if self.afo_enabled:
                if isoccluded or not cadence_update_zone:
                    self.cadence = self.prev_cadence
                else:
                    self.phase, measured_cadence, _ = self.oscillator.step_afo(scissor_signal)

                    cadence_floor = self.raw_frequency  # or 0.95 * self.raw_frequency
                    measured_cadence = max(measured_cadence, cadence_floor)

                    if measured_cadence > self.prev_cadence:
                        alpha = 0.35
                    else:
                        alpha = 0.30

                    self.cadence = (1 - alpha) * self.prev_cadence + alpha * measured_cadence
                    self.prev_cadence = self.cadence
            else:
                self.cadence = self.raw_frequency
            
            self.cadence_history.append(self.cadence)



            #Check if user in zone to input signal into AFO 

            stride_used = self.previous_stride
            if cadence_update_zone and not isoccluded:
                self.walker.stride_window.append(scissor_signal)
                self.candidate_stride = self.walker.last_stride(self.walker.stride_window)

                lower_bound = 0.80 * self.cal_stride
                upper_bound = 1.20 * self.cal_stride

                if (self.candidate_stride is not None and 
                    lower_bound <= self.candidate_stride <= upper_bound):
                        alpha = 0.10

                        self.previous_stride = (
                            (1 - alpha) * self.previous_stride
                            + alpha * self.candidate_stride
                        )
                        stride_used=self.previous_stride

                

                
            #Velocity Calculation / Commands
            feedforward_velocity = self.walker.velocity_command(
                self.cadence,
                stride_used,
                self.velocity_gain)
            

            #Freeze Detection
            if freeze_detected:
                linear_velocity = 0
                self.control_state.append(4)
                if not self.prev_freeze_detected:
                    self.freeze_detected_time_history.append(current_time)
                    print("-------- Freeze Detected -------")
                    print(f"Start of Freeze Detection: {self.freeze_window[0][0]} s")
                    print(f"Time of Detection: {current_time} s")
                    print(f"Range of Motion Detected: {motion_range} m")


            elif -0.60 < pelvis < -0.5:
                attenuation_factor=attenuation(pelvis,-0.60,-0.50)
                linear_velocity =  attenuation_factor * feedforward_velocity
                self.control_state.append(2) 

            elif -0.280 < pelvis < 0:
                boost_factor=boost(pelvis,-0.280,0)
                linear_velocity =  boost_factor * feedforward_velocity
                self.control_state.append(3)

            elif -0.60 < pelvis < -0.280:
                linear_velocity = feedforward_velocity
                self.control_state.append(1)

            else:
                linear_velocity = 0 
                self.control_state.append(4)  

            self.prev_freeze_detected = freeze_detected

            #Wheel Velcoity Ramping/Attenuation/Smoothing
            target_wheel_velocity = linear_velocity / self.wheel_radius
            target_wheel_velocity = np.clip(target_wheel_velocity, 0, 3 / self.wheel_radius) #Cap velocity at 3 m/s

            if self.control_state[-1] == 4:
                delta_limit = self.stop_delta_v
            elif self.control_state[-1] == 2:
                delta_limit = self.attenuation_delta_v
            elif (target_wheel_velocity - self.current_velocity) > self.pos_delta_v:
                delta_limit = self.pos_delta_v
            else:
                delta_limit= self.neg_delta_v

            if target_wheel_velocity - self.current_velocity > delta_limit:
                wheel_velocity = self.current_velocity + delta_limit
            elif target_wheel_velocity - self.current_velocity < -delta_limit:
                wheel_velocity = self.current_velocity - delta_limit
            else:
                wheel_velocity = target_wheel_velocity
                            

            if self.assist_ramping and target_wheel_velocity > 0.1 and abs(wheel_velocity - target_wheel_velocity) < self.pos_delta_v:
                self.assist_ramping = False
                self.afo_enabled = True
                self.oscillator.omega = 2 * np.pi * self.cadence

                self.ramp_complete_time = current_time
                self.freeze_detection_armed = False
                self.freeze_window.clear()

                print("Assist ramp complete. AFO tracking enabled.")


            wheel_velocity = np.clip(wheel_velocity,0,6) #Cap at 6 rad/s
            self.current_velocity = wheel_velocity
            self.velocity_history.append(wheel_velocity)
            self.commanded_timestamps.append(current_time)
            

            self.stride_used_history.append(stride_used)
            self.target_wheel_history.append(target_wheel_velocity)
            
            return wheel_velocity,self.time_to_cal
                
# --- Returns a value between 0 and 1 given a pelvis value between error max and error min ---
def attenuation(pelvis,error_max,error_min):
    return float(np.interp(pelvis,[error_max,error_min],[0,1.0]))

def boost (pelvis,error_max,error_min):
    return float(np.interp(pelvis,[error_max,error_min],[1.0,1.2]))


