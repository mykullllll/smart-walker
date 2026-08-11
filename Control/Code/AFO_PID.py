import numpy as np
from sklearn.cluster import DBSCAN

try:
    from Control.Code import calibration
except ImportError:
    import calibration


class SignalProcessor:
    "Handles Signal Processing of LiDAR and Leg detection"

    def __init__(self, scissor_window=None, right=None, left=None, true_timestamp=None, cal_encoder_velocity=None, prev_scissor=0, prev_avg=0, prev_left=0, prev_right=0):
        self.scissor_window = scissor_window if scissor_window is not None else []
        self.right = right if right is not None else []
        self.left = left if left is not None else []
        self.true_timestamp = true_timestamp if true_timestamp is not None else []
        self.cal_encoder_velocity = cal_encoder_velocity if cal_encoder_velocity is not None else []

        self.prev_scissor = prev_scissor
        self.prev_avg = prev_avg
        self.prev_left = prev_left
        self.prev_right = prev_right

    """def lowpass_filter(self,data,cutoff,fs,order=4):
            nyq = 0.5 * fs
            normal_cutoff = cutoff / nyq
            b, a = butter(order, normal_cutoff, btype='low')
            return filtfilt(b, a, data)"""

    def offline_data(self, left_current, right_current, time, encoder, alpha_scissor=0.35, alpha_pelvis=1.0):
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

        if encoder is not None:
            self.cal_encoder_velocity.append(encoder)
        else:
            self.cal_encoder_velocity.append(None)

        if time is not None:
            self.true_timestamp.append(time)
        else:
            self.true_timestamp.append(None)

        self.prev_scissor = scissor_signal
        self.prev_avg = avg_position
        self.prev_left = left_raw
        self.prev_right = right_raw

        return left_raw, right_raw, scissor_signal, avg_position

class AdaptiveFrequencyOscillator:
    # Calculate Frequency of signal

    def __init__(self, sampling_frequency, eta, eps, mu=1):
        self.x = 0
        self.y = 1.0
        self.omega = 2 * np.pi * 1.0
        self.dt = 1 / sampling_frequency
        self.eta = eta
        self.eps = eps
        self.mu = mu

    def step_afo(self, signal):

        r = np.sqrt(self.x ** 2 + self.y ** 2) + 1e-6
        xdot = (self.mu - np.square(r)) * self.x - self.omega * self.y + self.eps * signal
        ydot = (self.mu - np.square(r)) * self.y + self.omega * self.x
        omegadot = (-self.eta * signal * self.y) / r

        self.x = xdot * self.dt + self.x
        self.y = ydot * self.dt + self.y

        self.omega = np.clip(self.omega + omegadot * self.dt, 0.3, 20.0)

        cadence = self.omega / (2 * np.pi)

        return cadence

class WalkerController:
    "Velocity commands"

    def __init__(self, window_size=50, stride_window=None, k_p=3.0,k_i =0.0,k_d=0.0, position_deadband=0.02 , max_linear_velocity=0.684,integral_limit = 0.25):
        self.window_size = window_size
        self.stride_window = stride_window if stride_window is not None else []

        #Gains
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        #History
        self.feedforward_velocity_history = []
        self.feedback_velocity_history = []
        self.error_history = []
        self.unclipped_velocity_command = []

        #Limits
        self.integral_limit = integral_limit
        self.position_deadband = position_deadband
        self.max_linear_velocity = max_linear_velocity

        #Calculations
        self.integral_error = 0.0
        self.prev_error = None
        self.filtered_derivative = 0.0
        self.alpha = 0.2

    def last_stride(self, stride_window):
        if len(stride_window) > self.window_size:
            stride_window.pop(0)
        if len(stride_window) == self.window_size:
            last_stride = np.ptp(stride_window)  # Need to change this sometime (PTP finds maxiself.mum and miniself.mum of window, but is there a better way to scale the stride length? Kalman maybe)
            # print(last_stride)
            # last_stride = np.clip(last_stride,0.3,1.5)
            return last_stride

        return None

    def velocity_command(self, cadence, last_stride, velocity_gain, pelvis, desired_pelvis,wheel_radius,dt):
        raw_error = pelvis - desired_pelvis
        self.error_history.append(raw_error)

        #Turn off PID if in deadband
        if abs(raw_error) < self.position_deadband:
            self.error = 0.0
            self.integral_error=0
            self.filtered_derivative = 0.0

        else:
            self.error = raw_error
            if self.prev_error is not None:

                #Integral Calculation
                self.integral_error += self.error * dt

                #Derivative Calculation
                raw_derivative = (self.error - self.prev_error) / dt
                self.filtered_derivative += self.alpha * (raw_derivative - self.filtered_derivative)
            else:
                self.filtered_derivative = 0
                self.integral_error += self.error * dt
        

            
        #PID Calculation 
        feedback = self.k_p * (self.error) + self.k_i * self.integral_error + self.k_d * self.filtered_derivative
        self.prev_error = self.error

        #AFO/Feedforward Calculation
        feedforward = cadence * last_stride * velocity_gain

        #History
        self.unclipped_velocity_command.append((feedback + feedforward)/wheel_radius)
        velocity_command = np.clip(feedback + feedforward, -self.max_linear_velocity, self.max_linear_velocity)
        feedforward = feedforward / wheel_radius
        feedback = feedback / wheel_radius
        self.feedforward_velocity_history.append(feedforward)
        self.feedback_velocity_history.append(feedback)

        return velocity_command

class Cluster:
    def __init__(self, min_dist=0.25, max_dist=2, prev_leg_r=None, prev_leg_l=None):
        self.min_dist = min_dist
        self.max_dist = max_dist
        self.prev_leg_r = prev_leg_r
        self.prev_leg_l = prev_leg_l

        self.occlusions_length = 0

    def cluster_find(self, collisions):
        isoccluded = False

        if len(collisions) == 0:
            isoccluded = True
            self.occlusions_length += 1
            if self.occlusions_length >= 20:
                return None, None, isoccluded, True
            return None, None, isoccluded, False

        cluster = DBSCAN(eps=4e-2, min_samples=3).fit(collisions)
        labels = cluster.labels_
        unique_labels = [l for l in np.unique(labels) if l != -1]
        print(f"Number of clusters found: {len(unique_labels)}")
        centroids = []

        # Ideal Case (2 Clusters)
        if len(unique_labels) == 2:
            for index in unique_labels:
                leg_points = collisions[labels == index]
                centroid = np.mean(leg_points, axis=0)
                centroids.append(centroid)
            self.occlusions_length = 0

        # Thighs too close together
        elif len(unique_labels) == 1:
            leg_points = collisions[labels == unique_labels[0]]
            width = np.max(leg_points[:, 1]) - np.min(leg_points[:, 1])

            if width > 0.30:
                """kmeans= KMeans(n_clusters=2,n_init=10).fit(leg_points)
                centroids = kmeans.cluster_centers_"""
                left_leg = leg_points[: len(leg_points) // 2]
                right_leg = leg_points[len(leg_points) // 2 :]

                centroids.append(np.mean(left_leg, axis=0))
                centroids.append(np.mean(right_leg, axis=0))

            # Occlusion
            else:
                single_centroid = np.mean(leg_points, axis=0)
                isoccluded = True

                if isoccluded:
                    self.occlusions_length += 1
                else:
                    self.occlusions_length = 0

                if self.occlusions_length >= 20:
                    return None, None, isoccluded, True

                # Check which leg current cluster corresponds to
                if self.prev_leg_l is not None and self.prev_leg_r is not None:

                    dist_center_r = np.linalg.norm(single_centroid - self.prev_leg_l)
                    dist_center_l = np.linalg.norm(single_centroid - self.prev_leg_r)

                    if dist_center_r > dist_center_l:
                        return self.prev_leg_l, single_centroid, isoccluded, False

                    else:
                        return single_centroid, self.prev_leg_r, isoccluded, False

                # If no history drop frame
                else:
                    return None, None, isoccluded, False

        if len(unique_labels) > 2:

            if self.prev_leg_l is not None and self.prev_leg_r is not None:
                return self.prev_leg_l, self.prev_leg_r, isoccluded, False
            else:
                return None, None, isoccluded, False

        if len(unique_labels) == 0:
            isoccluded = True
            self.occlusions_length += 1
            if self.occlusions_length >= 20:
                return None, None, isoccluded, True
            return self.prev_leg_l, self.prev_leg_r, isoccluded, False

        else:
            if centroids[0][1] < centroids[1][1]:
                left_leg = centroids[1]
                right_leg = centroids[0]
                self.prev_leg_l = left_leg
                self.prev_leg_r = right_leg

            else:
                left_leg = centroids[0]
                right_leg = centroids[1]
                self.prev_leg_l = left_leg
                self.prev_leg_r = right_leg

        return left_leg, right_leg, isoccluded, False

    # Collision Scanner
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
        valid_mask = np.isfinite(ranges) & (~np.isnan(ranges)) & (ranges > self.min_dist) & (ranges < self.max_dist)

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
    def __init__(self, fs=10, wheel_radius=0.1143):

        self.fs = fs
        self.wheel_radius = wheel_radius
        self.max_accel = 0.4  # m/s^2
        self.max_decel = 0.8  # m/s^2
        self.pos_delta_v = (self.max_accel / self.fs) / self.wheel_radius
        self.neg_delta_v = (self.max_decel / self.fs) / self.wheel_radius
        self.current_velocity = 0
        self.cadence = None

        self.freeze_window = []
        self.freeze_window_times = []
        self.freeze_event_history = []

        self.freeze_detection_time = 1.2
        self.freeze_motion_threshold = 0.025
        self.freeze_detection_armed = False
        self.freeze_arm_delay = 2.0
        self.ramp_complete_time = None
        self.prev_freeze_detected = False

        self.calibrated = False
        self.velocity_gain = 1.0
        self.cal_stride = None
        self.time_to_cal = 0
        self.afo_enabled = False
        self.assist_ramping = False

        self.signal = SignalProcessor()
        self.oscillator = AdaptiveFrequencyOscillator(fs, eta=5.5, eps=8.5)
        self.walker = WalkerController()

        self.commanded_timestamps = []
        self.commanded_velocity_history = []
        self.encoder_data = []
        self.encoder_time = []
        self.control_state = []
        self.cadence_history = []
        self.pelvis_history = []
        self.stride_used_history = []
        self.target_wheel_history = []
        self.controller_timestamps = []

    def step_from_legs(self, current_time, encoder_velocity, left_x, right_x, isoccluded):
        if left_x is not None and right_x is not None and encoder_velocity is not None:
            _, _, scissor_signal, pelvis = self.signal.offline_data(left_x, right_x, current_time, encoder_velocity)
            self.pelvis_history.append(pelvis)
            self.encoder_data.append(encoder_velocity)
            self.encoder_time.append(current_time)

            if not self.calibrated:
                calibration_samples = int(self.fs * 15)
                if len(self.signal.scissor_window) >= calibration_samples and not self.calibrated:
                    (calibration_success, self.x_d, self.velocity_gain, self.cal_stride, self.raw_frequency, self.time_to_cal) = calibration.calibration(self.signal.right, self.signal.left, self.signal.scissor_window, self.fs, self.signal.cal_encoder_velocity, current_omega=self.oscillator.omega, wheel_radius=self.wheel_radius, timestamps=self.signal.true_timestamp)

                    if calibration_success:
                        self.calibrated = True
                        print("Calibration complete")
                        self.cadence = self.raw_frequency
                        self.prev_cadence = self.raw_frequency
                        self.oscillator.omega = 2 * np.pi * self.raw_frequency
                        self.previous_stride = self.cal_stride

                        self.assist_ramping = True
                        self.afo_enabled = False
                        self.current_velocity = 0

                    else:
                        self.signal.left.clear()
                        self.signal.right.clear()
                        self.signal.scissor_window.clear()
                        self.signal.cal_encoder_velocity.clear()
                        self.signal.true_timestamp.clear()
                        self.walker.stride_window.clear()
                        self.freeze_window.clear()
                return None, None

            if isoccluded:
                state = 3
            elif self.assist_ramping:
                state = 0
            else:
                state = 1

            if not self.assist_ramping and self.ramp_complete_time is not None and current_time - self.ramp_complete_time >= self.freeze_arm_delay:
                self.freeze_detection_armed = True

            freeze_detected = False
            if self.freeze_detection_armed and not isoccluded:
                self.freeze_window.append(scissor_signal)
                self.freeze_window_times.append(current_time)
                motion_range = np.ptp(self.freeze_window)
                time_range = self.freeze_window_times[-1] - self.freeze_window_times[0]

                freeze_detected = motion_range < self.freeze_motion_threshold and time_range > self.freeze_detection_time
                if freeze_detected:
                    state = 2
                    if not self.prev_freeze_detected:
                        self.freeze_event_history.append(current_time)
                        # Freeze Detection
                        print("-------- Freeze Detected -------")
                        print(f"Start of Freeze Detection: {self.freeze_window_times[0]} s")
                        print(f"Time of Detection: {current_time} s")
                        print(f"Range of Motion Detected: {motion_range} m")


                if time_range > self.freeze_detection_time:
                    self.freeze_window_times.pop(0)
                    self.freeze_window.pop(0)
            else:
                self.freeze_window.clear()
                self.freeze_window_times.clear()

            self.prev_freeze_detected = freeze_detected

            self.control_state.append(state)

            stride_used = self.previous_stride
            if not isoccluded:
                stride_used = self.cal_stride
                # self.walker.stride_window.append(scissor_signal)
                # candidate_stride = self.walker.last_stride(self.walker.stride_window)

                # lower_bound = 0.80 * self.cal_stride
                # upper_bound = 1.20 * self.cal_stride

                # if candidate_stride is not None and lower_bound <= candidate_stride <= upper_bound:
                #     alpha = 0.10

                #     self.previous_stride = (1 - alpha) * self.previous_stride + alpha * candidate_stride
                #     stride_used = self.previous_stride

            if state == 0:
                self.assist_ramping = True
                command_cadence = self.raw_frequency
                target_linear_velocity = self.walker.velocity_command(cadence=command_cadence, last_stride=self.cal_stride, velocity_gain=self.velocity_gain, pelvis=pelvis, desired_pelvis=self.x_d, wheel_radius=self.wheel_radius,dt = 1/self.fs)
                self.controller_timestamps.append(current_time)
            elif state == 1:
                self.controller_timestamps.append(current_time)
                # if self.afo_enabled:
                #     if isoccluded:
                #         self.cadence = self.prev_cadence
                #     else:
                #         measured_cadence = self.oscillator.step_afo(scissor_signal)

                #         cadence_floor = self.raw_frequency  # or 0.95 * self.raw_frequency
                #         measured_cadence = max(measured_cadence, cadence_floor)

                #         if measured_cadence > self.prev_cadence:
                #             alpha = 0.35
                #         else:
                #             alpha = 0.30

                #         self.cadence = (1 - alpha) * self.prev_cadence + alpha * measured_cadence
                #         self.prev_cadence = self.cadence
                # else:
                self.cadence = self.raw_frequency
                
                target_linear_velocity = self.walker.velocity_command(self.cadence, stride_used, self.velocity_gain, pelvis=pelvis, desired_pelvis=self.x_d,wheel_radius=self.wheel_radius,dt=1/self.fs)

            elif state in (2,3):
                target_linear_velocity = 0.0

            target_wheel_velocity = target_linear_velocity / self.wheel_radius

            if target_wheel_velocity > 0.1:
                if state == 0 and self.assist_ramping and abs(target_wheel_velocity - self.current_velocity) < 0.1:
                    self.assist_ramping = False
                    self.afo_enabled = True
                    self.oscillator.omega = 2 * np.pi * self.cadence

                    self.ramp_complete_time = current_time
                    self.freeze_detection_armed = False
                    self.freeze_window.clear()

                    print("Assist ramp complete. AFO tracking enabled.")

            self.cadence_history.append(self.cadence)

            if (target_wheel_velocity - self.current_velocity) > self.pos_delta_v:
                delta_limit = self.pos_delta_v
            else:
                delta_limit = self.neg_delta_v

            if target_wheel_velocity - self.current_velocity > delta_limit:
                commanded_velocity = self.current_velocity + delta_limit
            elif target_wheel_velocity - self.current_velocity < -delta_limit:
                commanded_velocity = self.current_velocity - delta_limit
            else:
                commanded_velocity = target_wheel_velocity

            self.current_velocity = commanded_velocity

            self.commanded_velocity_history.append(commanded_velocity)
            self.commanded_timestamps.append(current_time)

            self.stride_used_history.append(stride_used)
            self.target_wheel_history.append(target_wheel_velocity)

            return commanded_velocity, self.time_to_cal
