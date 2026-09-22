import csv
import pandas as pd

from pathlib import Path
import numpy as np
import json


import sys
from pathlib import Path

project_root = Path(__file__).resolve().parents[3]

if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from Control.Code.AFO_PID import Cluster

columns = [
    "epsilon",
    "min_samples",
    "scan_index",
    "time_s",
    "segment",
    "left_x",
    "left_y",
    "right_x",
    "right_y",
    "trial_id",
    "pelvis",
]

epsilon = np.arange(0.01,1,0.01)
n_step = np.arange(1,10,1)
lidar_directory = Path(__file__).resolve().parents[1]
input_folder = lidar_directory / "Data" / "Trial"
NOMINAL_DELTA_M = 0.60
MIN_DETECTION_RATE = 0.95

def export_csv(results):
    results_table = pd.DataFrame(results,columns=columns)
    output_directory = Path(__file__).resolve().parents[1]/ "Data"/ "Results"/"sweep_results.csv"
    results_table.to_csv(output_directory,index=False)

    group = ["epsilon","min_samples","trial_id","segment"]
    parameter_group=["epsilon","min_samples"]
    #RMSE
    segment_results = results_table.groupby(group,as_index=False).agg(mean_pelvis = ('pelvis',"mean"))
    segment_results["delta"] = segment_results.groupby(["epsilon","min_samples","trial_id"])["mean_pelvis"].diff() 
    segment_results["error"] = (segment_results["delta"] - NOMINAL_DELTA_M).abs()
    segment_results["squared_error"] = segment_results["error"].pow(2)
    segment_results["sum_squared_error"] = segment_results.groupby(parameter_group)["squared_error"].transform("sum")
    segment_results["n_trials"] = segment_results.assign(valid_error=segment_results["error"].notna()).groupby(parameter_group)["valid_error"].transform("sum")
    segment_results["rmse"] = np.sqrt(segment_results["sum_squared_error"]/segment_results["n_trials"])


    #Detection Rate
    results_table["detected"]= results_table["pelvis"].notna().astype(int)
    detection_results = results_table.groupby(parameter_group,as_index=False).agg(detected_length=("detected","sum"),scan_length=("scan_index","count"))
    detection_results["detection_rate"] = detection_results["detected_length"]/detection_results["scan_length"]
    detection_results["miss_rate"] = 1-detection_results["detection_rate"]

    trial_results = results_table.groupby(parameter_group+["trial_id"],as_index=False).agg(trial_detection_rate=("detected","mean"))
    trial_results["trial_miss_rate"] = 1- trial_results["trial_detection_rate"]
    trial_balanced_results = trial_results.groupby(parameter_group,as_index=False).agg(detection_rate=("trial_detection_rate","mean"),miss_rate=("trial_miss_rate","mean"))

    #MAE
    segment_results["mae"] = segment_results.groupby(parameter_group,as_index=False)["error"].transform("mean")

    #Standard Deviation
    std_results = results_table.groupby(group,as_index=False).agg(std_left_x=("left_x","std"),std_right_x=("right_x","std"))

    all_results = segment_results.merge(std_results,on=group,how="left").merge(detection_results,on=parameter_group,how="left").merge(trial_balanced_results,on=parameter_group,how="left")
    results_directory = Path(__file__).resolve().parents[1]/ "Data"/ "Results"
    results_directory.mkdir(parents=True, exist_ok=True)
    all_results.to_csv(results_directory/"segment_metrics.csv",index=False)




results=[]
for excel_file in input_folder.glob("*.csv"):
    print(f"Processing {excel_file.name}")
    df = pd.read_csv(excel_file)
    trial_id = excel_file.stem

    #Distinguish between first scan and second scan for each trial. 
    df["Segment"] = (
    df["Time (s)"]
    .diff()
    .lt(0)
    .cumsum()
    + 1
)

    for eps in epsilon:
        for n in n_step:
            cluster= Cluster()

            for scan_index, row in df.iterrows():
                cell = row["Collision values"]
                time_s = float(row["Time (s)"])
                segment = int(row["Segment"])
                

                coordinate_pairs = np.asarray(json.loads(cell),dtype=float)
                left_leg, right_leg, _, _ = cluster.cluster_find(coordinate_pairs,eps,n)

                left_x, left_y = left_leg if left_leg is not None else (np.nan, np.nan)
                right_x, right_y = right_leg if right_leg is not None else (np.nan, np.nan)

                if pd.notna(left_x) and pd.notna(right_x):
                    pelvis = (left_x + right_x)/2
                else:
                    pelvis = np.nan
                

                results.append((
                    eps,
                    n,
                    scan_index,
                    time_s,
                    segment,
                    left_x,
                    left_y,
                    right_x,
                    right_y,
                    trial_id,
                    pelvis,
                ))

export_csv(results)
print("Export complete")
            

        





                







    




