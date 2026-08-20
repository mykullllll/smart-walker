"""Sweep DBSCAN parameters across labeled LiDAR recording files.

Ground truth is supplied through a manifest with one row per recorded trial:

    file,trial_index,condition,ground_truth_position_change_m,ground_truth_initial_separation_m
    Data/bare_01.csv,1,bare,0.10,0.30
    Data/pants_01.csv,1,pants,0.10,0.25

``ground_truth_position_change_m`` is the non-negative forward/backward change
magnitude. ``ground_truth_initial_separation_m`` is the center-to-center leg
separation during the initial interval. Paths are relative to the manifest.

The script writes:
  * an aggregate CSV containing position-change and separation RMSE for every
    DBSCAN configuration, both overall and by condition;
  * a detailed CSV containing interval medians and errors for every trial.

Example:
    python3 Validation/LiDAR/dbscan_parameter_sweep.py trial_manifest.csv \
        --output dbscan_sweep_results.csv \
        --eps 0.02 0.03 0.04 0.05 0.06 \
        --min-samples 2 3 4 5
"""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
from sklearn.cluster import DBSCAN


MANIFEST_FIELDS = {
    "file",
    "trial_index",
    "condition",
    "ground_truth_position_change_m",
    "ground_truth_initial_separation_m",
}

RECORDER_FIELDS = {
    "trial_index",
    "phase",
    "angle_min",
    "angle_increment",
    "range_min",
    "range_max",
    "ranges",
}

SUMMARY_FIELDS = (
    "eps_m",
    "min_samples",
    "condition",
    "manifest_trials",
    "evaluable_position_trials",
    "position_change_rmse_m",
    "evaluable_separation_trials",
    "initial_separation_rmse_m",
    "total_scans",
    "valid_two_cluster_scans",
    "two_cluster_detection_rate",
)

DETAIL_FIELDS = (
    "source_file",
    "trial_index",
    "condition",
    "eps_m",
    "min_samples",
    "initial_total_scans",
    "initial_valid_two_cluster_scans",
    "initial_detection_rate",
    "final_total_scans",
    "final_valid_two_cluster_scans",
    "final_detection_rate",
    "median_initial_left_x_m",
    "median_initial_left_y_m",
    "median_initial_right_x_m",
    "median_initial_right_y_m",
    "median_initial_pelvis_x_m",
    "median_final_left_x_m",
    "median_final_left_y_m",
    "median_final_right_x_m",
    "median_final_right_y_m",
    "median_final_pelvis_x_m",
    "estimated_position_change_m",
    "ground_truth_position_change_m",
    "position_change_error_m",
    "median_initial_separation_m",
    "ground_truth_initial_separation_m",
    "initial_separation_error_m",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sweep DBSCAN parameters across every recording in a ground-truth "
            "manifest and calculate position-change and separation RMSE."
        )
    )
    parser.add_argument(
        "manifest",
        type=Path,
        help="CSV listing recording paths and trial ground truth",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("dbscan_sweep_results.csv"),
        help="Aggregate results CSV (default: dbscan_sweep_results.csv)",
    )
    parser.add_argument(
        "--details-output",
        type=Path,
        default=None,
        help="Per-trial results CSV (default: <output_stem>_trials.csv)",
    )
    parser.add_argument(
        "--eps",
        type=float,
        nargs="+",
        default=[0.02, 0.03, 0.04, 0.05, 0.06],
        help="DBSCAN eps values in metres",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        nargs="+",
        default=[2, 3, 4, 5],
        help="DBSCAN min_samples values",
    )
    parser.add_argument(
        "--min-distance",
        type=float,
        default=0.25,
        help="Minimum retained LiDAR range in metres (default: 0.25)",
    )
    parser.add_argument(
        "--max-distance",
        type=float,
        default=2.0,
        help="Maximum retained LiDAR range in metres (default: 2.0)",
    )
    return parser.parse_args()


def load_manifest(path: Path) -> list[dict]:
    """Load trial metadata and resolve recording paths relative to the manifest."""
    manifest_path = path.resolve()
    with manifest_path.open("r", newline="", encoding="utf-8") as manifest_file:
        reader = csv.DictReader(manifest_file)
        missing = MANIFEST_FIELDS.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                "Manifest is missing required columns: " + ", ".join(sorted(missing))
            )

        entries = []
        seen_trials = set()
        for row_number, row in enumerate(reader, start=2):
            try:
                recording_path = Path(row["file"])
                if not recording_path.is_absolute():
                    recording_path = manifest_path.parent / recording_path
                recording_path = recording_path.resolve()
                trial_index = int(row["trial_index"])
                position_change = float(row["ground_truth_position_change_m"])
                initial_separation = float(
                    row["ground_truth_initial_separation_m"]
                )
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f"Could not parse manifest row {row_number}: {error}"
                ) from error

            if position_change < 0 or initial_separation < 0:
                raise ValueError(
                    f"Manifest row {row_number} ground-truth values must be non-negative"
                )
            if not recording_path.is_file():
                raise FileNotFoundError(
                    f"Manifest row {row_number} recording not found: {recording_path}"
                )

            trial_key = (recording_path, trial_index)
            if trial_key in seen_trials:
                raise ValueError(
                    f"Manifest contains duplicate file/trial pair on row {row_number}"
                )
            seen_trials.add(trial_key)
            entries.append(
                {
                    "source_file": recording_path,
                    "trial_index": trial_index,
                    "condition": row["condition"].strip() or "unspecified",
                    "ground_truth_position_change_m": position_change,
                    "ground_truth_initial_separation_m": initial_separation,
                }
            )

    if not entries:
        raise ValueError("Manifest contains no trial rows")
    return entries


def load_recorded_scans(path: Path) -> list[dict]:
    """Load one recorder CSV."""
    with path.open("r", newline="", encoding="utf-8") as input_file:
        reader = csv.DictReader(input_file)
        missing = RECORDER_FIELDS.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                f"{path} is missing required columns: " + ", ".join(sorted(missing))
            )

        scans = []
        for row_number, row in enumerate(reader, start=2):
            try:
                scans.append(
                    {
                        "trial_index": int(row["trial_index"]),
                        "phase": row["phase"].strip().lower(),
                        "angle_min": float(row["angle_min"]),
                        "angle_increment": float(row["angle_increment"]),
                        "sensor_range_min": float(row["range_min"]),
                        "sensor_range_max": float(row["range_max"]),
                        "ranges": np.asarray(json.loads(row["ranges"]), dtype=float),
                    }
                )
            except (TypeError, ValueError, json.JSONDecodeError) as error:
                raise ValueError(f"Could not parse {path} row {row_number}: {error}") from error
    return scans


def scan_to_points(scan: dict, min_distance: float, max_distance: float) -> np.ndarray:
    """Convert valid polar measurements into Cartesian points."""
    ranges = scan["ranges"]
    angles = scan["angle_min"] + np.arange(ranges.size) * scan["angle_increment"]
    lower_bound = max(min_distance, scan["sensor_range_min"])
    upper_bound = min(max_distance, scan["sensor_range_max"])
    valid = np.isfinite(ranges) & (ranges > lower_bound) & (ranges < upper_bound)

    valid_ranges = ranges[valid]
    valid_angles = angles[valid]
    if valid_ranges.size == 0:
        return np.empty((0, 2), dtype=float)
    return np.column_stack(
        (valid_ranges * np.cos(valid_angles), valid_ranges * np.sin(valid_angles))
    )


def detect_two_legs(points: np.ndarray, eps: float, min_samples: int):
    """Return left/right centroids when DBSCAN finds exactly two clusters."""
    if points.shape[0] < min_samples:
        return None, None

    labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(points)
    cluster_labels = [label for label in np.unique(labels) if label != -1]
    if len(cluster_labels) != 2:
        return None, None

    centroids = [np.mean(points[labels == label], axis=0) for label in cluster_labels]
    centroids.sort(key=lambda centroid: centroid[1], reverse=True)
    return centroids[0], centroids[1]


def summarize_phase(
    scans: list[dict], eps: float, min_samples: int, min_distance: float, max_distance: float
) -> dict:
    """Calculate median positions and detection coverage for one phase."""
    left_positions = []
    right_positions = []
    separations = []

    for scan in scans:
        points = scan_to_points(scan, min_distance, max_distance)
        left, right = detect_two_legs(points, eps, min_samples)
        if left is None or right is None:
            continue
        left_positions.append(left)
        right_positions.append(right)
        separations.append(float(np.linalg.norm(left - right)))

    total_scans = len(scans)
    valid_scans = len(separations)
    if valid_scans:
        median_left = np.median(np.asarray(left_positions), axis=0)
        median_right = np.median(np.asarray(right_positions), axis=0)
        median_pelvis_x = float((median_left[0] + median_right[0]) / 2.0)
        median_separation = float(np.median(separations))
    else:
        median_left = None
        median_right = None
        median_pelvis_x = None
        median_separation = None

    return {
        "total_scans": total_scans,
        "valid_scans": valid_scans,
        "detection_rate": valid_scans / total_scans if total_scans else 0.0,
        "median_left": median_left,
        "median_right": median_right,
        "median_pelvis_x": median_pelvis_x,
        "median_separation": median_separation,
    }


def numeric_or_blank(value):
    if value is None:
        return ""
    return f"{float(value):.9f}"


def coordinate_or_blank(position, index: int):
    if position is None:
        return ""
    return numeric_or_blank(position[index])


def evaluate_trial(
    entry: dict,
    scans: list[dict],
    eps: float,
    min_samples: int,
    min_distance: float,
    max_distance: float,
) -> dict:
    """Pair initial/final medians and calculate one trial's errors."""
    trial_scans = [scan for scan in scans if scan["trial_index"] == entry["trial_index"]]
    initial = summarize_phase(
        [scan for scan in trial_scans if scan["phase"] == "initial"],
        eps,
        min_samples,
        min_distance,
        max_distance,
    )
    final = summarize_phase(
        [scan for scan in trial_scans if scan["phase"] == "final"],
        eps,
        min_samples,
        min_distance,
        max_distance,
    )

    estimated_change = None
    position_error = None
    if initial["median_pelvis_x"] is not None and final["median_pelvis_x"] is not None:
        estimated_change = abs(final["median_pelvis_x"] - initial["median_pelvis_x"])
        position_error = estimated_change - entry["ground_truth_position_change_m"]

    separation_error = None
    if initial["median_separation"] is not None:
        separation_error = (
            initial["median_separation"]
            - entry["ground_truth_initial_separation_m"]
        )

    return {
        "source_file": str(entry["source_file"]),
        "trial_index": entry["trial_index"],
        "condition": entry["condition"],
        "eps_m": f"{eps:.9f}",
        "min_samples": min_samples,
        "initial_total_scans": initial["total_scans"],
        "initial_valid_two_cluster_scans": initial["valid_scans"],
        "initial_detection_rate": numeric_or_blank(initial["detection_rate"]),
        "final_total_scans": final["total_scans"],
        "final_valid_two_cluster_scans": final["valid_scans"],
        "final_detection_rate": numeric_or_blank(final["detection_rate"]),
        "median_initial_left_x_m": coordinate_or_blank(initial["median_left"], 0),
        "median_initial_left_y_m": coordinate_or_blank(initial["median_left"], 1),
        "median_initial_right_x_m": coordinate_or_blank(initial["median_right"], 0),
        "median_initial_right_y_m": coordinate_or_blank(initial["median_right"], 1),
        "median_initial_pelvis_x_m": numeric_or_blank(initial["median_pelvis_x"]),
        "median_final_left_x_m": coordinate_or_blank(final["median_left"], 0),
        "median_final_left_y_m": coordinate_or_blank(final["median_left"], 1),
        "median_final_right_x_m": coordinate_or_blank(final["median_right"], 0),
        "median_final_right_y_m": coordinate_or_blank(final["median_right"], 1),
        "median_final_pelvis_x_m": numeric_or_blank(final["median_pelvis_x"]),
        "estimated_position_change_m": numeric_or_blank(estimated_change),
        "ground_truth_position_change_m": numeric_or_blank(
            entry["ground_truth_position_change_m"]
        ),
        "position_change_error_m": numeric_or_blank(position_error),
        "median_initial_separation_m": numeric_or_blank(initial["median_separation"]),
        "ground_truth_initial_separation_m": numeric_or_blank(
            entry["ground_truth_initial_separation_m"]
        ),
        "initial_separation_error_m": numeric_or_blank(separation_error),
        "_position_error": position_error,
        "_separation_error": separation_error,
    }


def rmse(errors: list[float]):
    if not errors:
        return None
    return float(np.sqrt(np.mean(np.square(errors))))


def summarize_configuration(rows: list[dict], eps: float, min_samples: int) -> list[dict]:
    """Create overall and per-condition RMSE rows."""
    conditions = sorted({row["condition"] for row in rows})
    summaries = []
    for condition in ["ALL", *conditions]:
        selected = rows if condition == "ALL" else [
            row for row in rows if row["condition"] == condition
        ]
        position_errors = [
            row["_position_error"] for row in selected if row["_position_error"] is not None
        ]
        separation_errors = [
            row["_separation_error"]
            for row in selected
            if row["_separation_error"] is not None
        ]
        total_scans = sum(
            row["initial_total_scans"] + row["final_total_scans"] for row in selected
        )
        valid_scans = sum(
            row["initial_valid_two_cluster_scans"]
            + row["final_valid_two_cluster_scans"]
            for row in selected
        )
        summaries.append(
            {
                "eps_m": f"{eps:.9f}",
                "min_samples": min_samples,
                "condition": condition,
                "manifest_trials": len(selected),
                "evaluable_position_trials": len(position_errors),
                "position_change_rmse_m": numeric_or_blank(rmse(position_errors)),
                "evaluable_separation_trials": len(separation_errors),
                "initial_separation_rmse_m": numeric_or_blank(rmse(separation_errors)),
                "total_scans": total_scans,
                "valid_two_cluster_scans": valid_scans,
                "two_cluster_detection_rate": numeric_or_blank(
                    valid_scans / total_scans if total_scans else 0.0
                ),
            }
        )
    return summaries


def write_csv(path: Path, fields: tuple, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    if args.min_distance >= args.max_distance:
        raise ValueError("--min-distance must be smaller than --max-distance")
    if any(eps <= 0 for eps in args.eps):
        raise ValueError("All --eps values must be greater than zero")
    if any(value < 1 for value in args.min_samples):
        raise ValueError("All --min-samples values must be at least one")

    manifest = load_manifest(args.manifest)
    recordings = {
        path: load_recorded_scans(path)
        for path in sorted({entry["source_file"] for entry in manifest})
    }

    detail_rows = []
    summary_rows = []
    for eps in args.eps:
        for min_samples in args.min_samples:
            configuration_rows = [
                evaluate_trial(
                    entry,
                    recordings[entry["source_file"]],
                    eps,
                    min_samples,
                    args.min_distance,
                    args.max_distance,
                )
                for entry in manifest
            ]
            detail_rows.extend(configuration_rows)
            summary_rows.extend(
                summarize_configuration(configuration_rows, eps, min_samples)
            )

    details_output = args.details_output
    if details_output is None:
        details_output = args.output.with_name(f"{args.output.stem}_trials.csv")

    write_csv(args.output, SUMMARY_FIELDS, summary_rows)
    write_csv(details_output, DETAIL_FIELDS, detail_rows)

    print(
        f"Processed {len(recordings)} recording files and {len(manifest)} trials "
        f"with {len(args.eps) * len(args.min_samples)} configurations."
    )
    print(f"Aggregate RMSE results: {args.output.resolve()}")
    print(f"Per-trial median results: {details_output.resolve()}")


if __name__ == "__main__":
    main()
