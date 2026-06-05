import pandas as pd
from matplotlib import pyplot as plt
import numpy as np


def pd_control(x_d, x, Kp, Kd, dt):
    error = x_d - x
    derivative = error / dt
    control_signal = Kp * error + Kd * derivative
    return control_signal


files = [
    "/Users/michaelnawa/Documents/GitHub/smart-walker/HardwareInTheLoop/data/normal.csv",
    "/Users/michaelnawa/Documents/GitHub/smart-walker/HardwareInTheLoop/data/fast.csv",
]
colors = ['blue', 'green']  # one per file

num_files=len(files)


