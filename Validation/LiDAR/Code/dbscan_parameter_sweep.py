import csv
import pandas as pd
from pathlib import Path
from Control.Code.AFO_PID import Cluster
import glob
import numpy as np
import ast
import json
import select

columns = [
    "epsilon",
    "min_samples",
    "left_x",
    "left_y",
    "right_x",
    "right_y",
    "trial_id",
]

epsilon = np.arange(0.01,3,0.01)
n_step = np.arange(1,10,1)

input_folder = Path("data")
def export_csv(results):

    output_directory = Path(__file__).resolve().parents[1] / "Data"
    output_directory.mkdir(parents=True, exist_ok=True)
    output_path = output_directory / f"sweep_results.csv"

    results_table = pd.DataFrame(results, columns=columns)
    results_table.to_csv(output_path, index=False)


results=[]
for excel_file in input_folder.glob("*.csv"):
    cluster= Cluster()
    print(f"Processing {excel_file.name}")
    df = pd.read_csv(excel_file)

    if df["Trial ID"].nunique() != 1:
        raise ValueError(f"{excel_file.name} contains multiple Trial IDs")
    
    trial_id = df["Trial ID"].iloc[0]

    for eps in epsilon:
        for n in n_step:
            for cell in df["Collision values"]:

                coordinate_pairs = np.asarray(json.loads(cell),dtype=float)
                left_leg, right_leg, _, _ = cluster.cluster_find(coordinate_pairs,eps,n)

                left_x, left_y = left_leg if left_leg is not None else (np.nan, np.nan)
                right_x, right_y = right_leg if right_leg is not None else (np.nan, np.nan)

                results.append((eps,n,left_x,left_y,right_x,right_y,trial_id))

export_csv(results)
print("Export complete")
            

        





                







    




