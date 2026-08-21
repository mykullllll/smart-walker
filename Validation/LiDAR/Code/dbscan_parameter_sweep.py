import csv
import pandas as pd
from pathlib import Path
from Control.Code.AFO_PID import Cluster
import glob




path = "LiDAR/Data/*.csv"
all_files=glob.glob(path)
data_frames_list=[]
for filename in all_files:
    df=pd.read_csv(filename)
    data_frames_list.append(df)

    




