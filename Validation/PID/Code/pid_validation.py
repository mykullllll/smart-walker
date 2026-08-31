import sys
from pathlib import Path
from matplotlib import pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import csv
from itertools import product
import random
import math
from collections import deque

project_root = Path(__file__).resolve().parents[3]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))
    
from Control.Code.AFO_PID import main_loop



k_p_values = [0,0.25,0.5,1.0,1.5,2.0,2.5,3.0,4.0,5.0,6.0]
k_i_values = [0.0, 0.05, 0.10, 0.20,0.3,0.4,0.5]
k_d_values = [0.0, 0.02, 0.05, 0.10,0.15,0.20,0.25,0.4,0.5,0.6,0.7,0.8,2.0,3.0,4.0,5.0,9.0]
noise_seeds = [0]


def make_controller(k_p,k_i,k_d,fs=6,wheel_radius=0.1143):
    controller = main_loop(
        fs=fs,
        wheel_radius=wheel_radius,
    )

    # Keep freeze detection out of the position-controller test.
    controller.ramp_complete_time = None
    controller.freeze_detection_armed = False

    #Gains
    controller.walker.k_p = k_p
    controller.walker.k_i = k_i
    controller.walker.k_d = k_d



    # Skip the real 15-second calibration for this test.
    controller.calibrated = True
    controller.x_d = -0.40
    controller.velocity_gain = 0
    controller.cal_stride = 0.30
    controller.previous_stride = 0.30
    controller.raw_frequency = 0.60
    controller.cadence = 0.60
    controller.prev_cadence = 0.60

    # Begin directly in fixed-gait tracking mode.
    controller.assist_ramping = False
    controller.afo_enabled = False

    return controller


def run_trial(controller,simulation_steps=100,position_noise_std_m=0.0,noise_seed=1234):


    current_time_history=[]
    average_position_history=[]
    position_error_history=[]
    commanded_velocity_linear_history =[]
    position_error_history_square=[]
    acceleration_window = []
    jerk_window=[]
    wheel_command=[]
    position_queue = []

    #Latency
    k_dynamic = 0.968507993
    tau = 0.686005288
    prev_latent_velocity = 0
    prev_time = 0
    initial_position = -0.50
    position_queue = deque([initial_position])


    #Metrics
    settle = False
    overshoot_status = False
    prev_state=False
    deadband_samples = 0
    counter=0
    oscillation = 0
    integral_abs_error=0
    peak_error = 0.0
    patient_velocity=0.0
    acceleration=0.0
    rng = random.Random(noise_seed) #Set Random sequence about gaussian distribution 


    #Simulated Velocity Changes Variables
    disturbance_velocities = [0,-0.2, 0.2, -0.16, 0.12, -0.20]
    disturbance_index = 0
    disturbance_interval_s = 2.0

    disturbance_interval_steps = round(disturbance_interval_s * controller.fs)
    dt = 1.0 / controller.fs

    #Latency 2 steps delayed

    
    average_position = initial_position


    for index in range(simulation_steps):
        position_noise = rng.gauss(0.0,position_noise_std_m,)
        noisy_position = average_position + position_noise
        disturbance_index = (index // disturbance_interval_steps) % len(disturbance_velocities)
        patient_velocity = disturbance_velocities[disturbance_index]
        current_time = index * dt
        position_queue.append(noisy_position)
        measured_position = position_queue.popleft()



        left_x = measured_position
        right_x = measured_position

        result = controller.step_from_legs(
            current_time=current_time,
            encoder_velocity=0.0,
            left_x=left_x,
            right_x=right_x,
            isoccluded=False,
        )

        if result is None or result[0] is None: 
            continue
        position_error = (controller.walker.error_history[-1])
        commanded_velocity_linear = result[0] * controller.wheel_radius
         
        latent_velocity = k_dynamic * commanded_velocity_linear + (prev_latent_velocity - k_dynamic*commanded_velocity_linear) * math.e ** (-(current_time - prev_time)/tau)

        average_position+= (patient_velocity * dt) - (latent_velocity * dt)
        wheel_command.append(latent_velocity)
       
        if len(wheel_command) > 1:
            previous_acceleration = acceleration
            acceleration = (wheel_command[-1] - wheel_command[-2]) / dt
            acceleration_window.append(acceleration)
            if len(acceleration_window)>1:
                jerk = (acceleration - previous_acceleration) / dt
                jerk_window.append(jerk)
                

        simulation_error = (average_position - controller.x_d)
        peak_error = max(peak_error,abs(simulation_error))
        integral_abs_error += (abs(simulation_error) * dt)

        # print(
        #     f"time={current_time:.5f}, "
        #     f"disturbance={disturbance_index}, "
        #     f"patient_velocity={patient_velocity:.5f}, "
        #     f"position={average_position:.5f}, "
        #     f"command={commanded_velocity_linear:.5f} m/s"
        # )

        # Change velocity after completing the 2-second interval.




        current_time_history.append(current_time)
        average_position_history.append(average_position)
        position_error_history_square.append(simulation_error**2)
        position_error_history.append(position_error)
        commanded_velocity_linear_history.append(commanded_velocity_linear)

        if abs(controller.x_d - average_position) <= controller.walker.position_deadband:
            prev_state=True
            counter +=1
            deadband_samples += 1
            if counter > 30:
                settle = True
                settling_time = current_time + dt - (counter - 1) * dt

        else:
            counter=0
            if prev_state is True:
                oscillation+=1
            prev_state = False

        if average_position > controller.x_d :
            overshoot_status=True

        prev_latent_velocity = latent_velocity
        prev_time = current_time


    #Post Calculations  
    rmse = math.sqrt(sum(position_error_history_square) / len(position_error_history_square))
    rms_acceleration = math.sqrt(sum(value**2 for value in acceleration_window) / len(acceleration_window)) if acceleration_window else 0.0
    rms_jerk = math.sqrt(sum(value**2 for value in jerk_window) / len(jerk_window)) if jerk_window else 0.0

    metrics = {
        "k_p": controller.walker.k_p,
        "k_i": controller.walker.k_i,
        "k_d": controller.walker.k_d,
        "Integral Absolute Error": integral_abs_error,
        "Root Mean Square Error (RMSE)": rmse,
        "Root Mean Square Acceleration (RMS)": rms_acceleration,
        "Root Mean Square Jerk (RMS)": rms_jerk,
        "Max error (m)": peak_error,
        "Overshoot": overshoot_status,
        "Position Noise Standard Deviation": position_noise_std_m,
        "Motor Gain": k_dynamic,
        "Motor Time Constant (s)": tau,
    }

    print(
        f"k_p={controller.walker.k_p}, "
        f"k_i={controller.walker.k_i}, "
        f"k_d={controller.walker.k_d}, "
        f"Integral Absolute Error = {integral_abs_error},"
        f"Root Mean Square Error (RMSE)= {rmse},"
        f"Root Mean Square Acceleration (RMS)= {rms_acceleration},"
        f"Root Mean Square Jerk (RMS)= {rms_jerk} "
        f"Max error (m) = {peak_error} m "
        f"Overshoot = {overshoot_status}], "
        f"Positional Noise {position_noise_std_m} "
        f"Motor Gain {k_dynamic}",
        f"Motor Time Constant (s) {tau}"
    )

    return metrics


def sweep(k_p_values,k_i_values,k_d_values):
    gain_results=[]

    for k_p,k_i,k_d in product(k_p_values,k_i_values,k_d_values):
        for noise_seed in noise_seeds:
            controller = make_controller(k_p,k_i,k_d)
            metrics = run_trial(controller,position_noise_std_m=0.00,noise_seed=noise_seed)
            gain_results.append(metrics)

    return gain_results


def save_csv(gain_results,csv_path):
    fieldnames = [
        "k_p",
        "k_i",
        "k_d",
        "Integral Absolute Error",
        "Root Mean Square Error (RMSE)",
        "Root Mean Square Acceleration (RMS)",
        "Root Mean Square Jerk (RMS)",
        "Max error (m)",
        "Overshoot",
        "Position Noise Standard Deviation",
        "Motor Gain",
        "Motor Time Constant (s)",
        
    ]

    with csv_path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=fieldnames,
        )
        writer.writeheader()
        writer.writerows(gain_results)

    print(f"Saved {len(gain_results)} results to {csv_path}")


def save_markdown(gain_results,markdown_path):
    #Saving to gait_baseline_data.md file
    with markdown_path.open("w") as md:
        print("# Gain Results\n", file=md)
        print(
            "| k_p| k_i | k_d | Integral Absolute Error | Root Mean Square Error (RMSE) | Root Mean Square Acceleration (RMS) | Root Mean Square Jerk (RMS) | Max error (m) | Overshoot | Position Noise Standard Deviation| Motor Gain | Motor Time Constant (s)",
            file=md,
        )
        print("|---:|---:|---:|---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|", file=md)

        for result in gain_results:
            print(
                f"| {result['k_p']} "
                f"| {result['k_i']} "
                f"| {result['k_d']} "
                f"| {result['Integral Absolute Error']} "
                f"| {result['Root Mean Square Error (RMSE)']}"
                f"| {result['Root Mean Square Acceleration (RMS)']}"
                f"| {result['Root Mean Square Jerk (RMS)']}"
                f"| {result['Max error (m)']} "
                f"| {result['Overshoot']} | ",
                f"| {result['Position Noise Standard Deviation']} |",
                f"| {result['Motor Gain']} |",
                f"| {result['Motor Time Constant (s)']} |",

                file=md,
            )

    print(f"Saved to {markdown_path}")


def main():
    gain_results = sweep(k_p_values,k_i_values,k_d_values)

    csv_path = (
        Path(__file__).resolve().parents[1]
        / "Data"
        / "latency_gain_data.csv"
    )
    save_csv(gain_results,csv_path)

    markdown_path = Path(__file__).with_name("latency_gain_data.md")
    save_markdown(gain_results,markdown_path)


if __name__ == "__main__":
    main()
