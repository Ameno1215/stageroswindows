#!/usr/bin/env python3
"""
Compute robot motion accuracy statistics from CSV log(s).
Generates two distinct reports:
  1. Cartesian report (x, y, z, roll, pitch, yaw)
  2. Joint report (individual joints and Euclidean joint distance)

Velocity and Acceleration are now kept as separate columns.
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

POS = ["x", "y", "z"]
ROT = ["roll", "pitch", "yaw"]

# ----------------------------------------------------------------------
# Error math
# ----------------------------------------------------------------------

def wrap(a):
    """Wrap angle(s) to [-pi, pi]."""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def geodesic_angle(rpy_d, rpy_m):
    """Smallest rotation (rad) bringing the desired orientation onto the measured one."""
    rpy_d = np.asarray(rpy_d, dtype=float)
    rpy_m = np.asarray(rpy_m, dtype=float)
    out = np.full(len(rpy_d), np.nan)
    valid = ~(np.isnan(rpy_d).any(axis=1) | np.isnan(rpy_m).any(axis=1))
    if valid.any():
        Rd = R.from_euler("xyz", rpy_d[valid])
        Rm = R.from_euler("xyz", rpy_m[valid])
        out[valid] = np.linalg.norm((Rm * Rd.inv()).as_rotvec(), axis=-1)
    return out


def add_error_columns(df):
    """Append per-axis errors, Euclidean position, geodesic rotation, and joint errors."""
    df = df.copy()
    
    # Cartesian Errors
    for a in POS:
        if f"measured_{a}" in df.columns and f"desired_{a}" in df.columns:
            df[f"err_{a}"] = df[f"measured_{a}"] - df[f"desired_{a}"]
        else:
            df[f"err_{a}"] = np.nan
            
    for a in ROT:
        if f"measured_{a}" in df.columns and f"desired_{a}" in df.columns:
            df[f"err_{a}"] = wrap(df[f"measured_{a}"] - df[f"desired_{a}"])
        else:
            df[f"err_{a}"] = np.nan
            
    df["err_pos"] = np.sqrt(df["err_x"] ** 2 + df["err_y"] ** 2 + df["err_z"] ** 2)
    
    if all(f"desired_{a}" in df.columns for a in ROT) and all(f"measured_{a}" in df.columns for a in ROT):
        df["err_rot"] = geodesic_angle(
            df[[f"desired_{a}" for a in ROT]].to_numpy(),
            df[[f"measured_{a}" for a in ROT]].to_numpy(),
        )
    else:
        df["err_rot"] = np.nan

    # Joint Errors (dynamically find joint columns like desired_j1, desired_joint_1, etc.)
    joint_cols = [c.replace("desired_", "") for c in df.columns 
                  if c.startswith("desired_") and c.replace("desired_", "") not in POS + ROT]
    
    for j in joint_cols:
        df[f"err_{j}"] = df[f"measured_{j}"] - df[f"desired_{j}"]
        
    if joint_cols:
        # Euclidean distance across all joints
        df["err_joint_eucl"] = np.sqrt(sum(df[f"err_{j}"] ** 2 for j in joint_cols))
        
    return df, joint_cols

# ----------------------------------------------------------------------
# Column normalization
# ----------------------------------------------------------------------

def _to_bool_or_none(v):
    if v is None or (isinstance(v, float) and pd.isna(v)): return None
    if isinstance(v, bool): return v
    if isinstance(v, (int, np.integer)): return bool(v)
    s = str(v).strip().lower()
    if s in ("true", "1", "t", "yes"): return True
    if s in ("false", "0", "f", "no"): return False
    return None

def _to_frame_or_none(v):
    if v is None or (isinstance(v, float) and pd.isna(v)): return None
    s = str(v).strip().upper()
    if s in ("WORLD", "TOOL"): return s
    return None

def _to_scaling_or_none(v):
    if v is None or (isinstance(v, float) and pd.isna(v)): return None
    s = str(v).strip()
    if s == "": return None
    try:
        return int(round(float(s)))
    except (TypeError, ValueError):
        return None

def normalize_columns(df):
    """Coerce flag columns into a consistent form."""
    df = df.copy()
    defaults = {
        "cartesian_path": None, "is_relative": None, 
        "reference_frame": None, "velocity": None, "acceleration": None
    }
    for col, default_val in defaults.items():
        if col not in df.columns:
            df[col] = default_val

    df["cartesian_path"] = df["cartesian_path"].map(_to_bool_or_none)
    df["is_relative"] = df["is_relative"].map(_to_bool_or_none)
    df["reference_frame"] = df["reference_frame"].map(_to_frame_or_none)
    df["velocity"] = df["velocity"].map(_to_scaling_or_none)
    df["acceleration"] = df["acceleration"].map(_to_scaling_or_none)
    df["movement"] = df["movement"].astype(str)
    return df

# ----------------------------------------------------------------------
# Reporting
# ----------------------------------------------------------------------

def summarize_cartesian(sub, meta_dict):
    """Generate stats dict for cartesian parameters."""
    out = dict(meta_dict)
    out["n"] = int(len(sub))
    
    for a in POS:
        e = sub[f"err_{a}"].abs() * 1000.0  # m -> mm
        out[f"{a}_mean_mm"] = e.mean()
        out[f"{a}_min_mm"] = e.min()
        out[f"{a}_max_mm"] = e.max()
        
    for a in ROT:
        e = np.rad2deg(sub[f"err_{a}"].abs())
        out[f"{a}_mean_deg"] = e.mean()
        out[f"{a}_min_deg"] = e.min()
        out[f"{a}_max_deg"] = e.max()
        
    p = sub["err_pos"] * 1000.0
    out["pos_eucl_mean_mm"] = p.mean()
    out["pos_eucl_min_mm"] = p.min()
    out["pos_eucl_max_mm"] = p.max()
    
    r = np.rad2deg(sub["err_rot"])
    out["rot_geo_mean_deg"] = r.mean()
    out["rot_geo_min_deg"] = r.min()
    out["rot_geo_max_deg"] = r.max()
    
    return out

def summarize_joints(sub, meta_dict, joint_cols):
    """Generate stats dict for joint parameters."""
    out = dict(meta_dict)
    out["n"] = int(len(sub))
    
    for j in joint_cols:
        e = np.rad2deg(sub[f"err_{j}"].abs()) # Assuming logged in rad, output in deg
        out[f"{j}_mean_deg"] = e.mean()
        out[f"{j}_min_deg"] = e.min()
        out[f"{j}_max_deg"] = e.max()
        
    if joint_cols:
        je = np.rad2deg(sub["err_joint_eucl"].abs())
        out["joint_eucl_mean_deg"] = je.mean()
        out["joint_eucl_min_deg"] = je.min()
        out["joint_eucl_max_deg"] = je.max()
        
    return out

def build_cartesian_report(df):
    """Builds the Cartesian specific report."""
    rows = []
    df_cart = df[df["movement"] != "move_joints"].copy()
    if df_cart.empty:
        return pd.DataFrame()

    keys = ["movement", "cartesian_path", "reference_frame", "is_relative", "velocity", "acceleration"]
    grouped = df_cart.groupby(keys, dropna=False, sort=True)

    # 1. Per specific parameter combination
    for group_keys, sub in grouped:
        meta = dict(zip(keys, group_keys))
        rows.append(summarize_cartesian(sub, meta))

    # 2. Per movement type aggregate (ALL)
    for function, sub in df_cart.groupby("movement", sort=True):
        meta = {k: "ALL" for k in keys}
        meta["movement"] = function
        rows.append(summarize_cartesian(sub, meta))

    # 3. Global aggregate (ALL movements)
    meta_global = {k: "ALL" for k in keys}
    rows.append(summarize_cartesian(df_cart, meta_global))

    return pd.DataFrame(rows)

def build_joint_report(df, joint_cols):
    """Builds the Joint specific report."""
    rows = []
    df_joints = df[df["movement"] == "move_joints"].copy()
    if df_joints.empty:
        return pd.DataFrame()

    keys = ["movement", "is_relative", "velocity", "acceleration"]
    grouped = df_joints.groupby(keys, dropna=False, sort=True)

    # 1. Per specific parameter combination
    for group_keys, sub in grouped:
        meta = dict(zip(keys, group_keys))
        rows.append(summarize_joints(sub, meta, joint_cols))

    # 2. Global aggregate (ALL parameters mixed)
    meta_global = {k: "ALL" for k in keys}
    meta_global["movement"] = "move_joints"
    rows.append(summarize_joints(df_joints, meta_global, joint_cols))

    return pd.DataFrame(rows)

# ----------------------------------------------------------------------
# IO
# ----------------------------------------------------------------------

def load(paths):
    frames = []
    for p in paths:
        p = Path(p)
        files = sorted(p.glob("*.csv")) if p.is_dir() else [p]
        for f in files:
            d = pd.read_csv(f)
            d["__source"] = f.name
            frames.append(d)
    if not frames:
        raise SystemExit("no CSV files found")
    return pd.concat(frames, ignore_index=True)

def default_output_paths(paths):
    first = Path(paths[0])
    stem = first.name if first.is_dir() else first.stem
    base_dir = Path("accuracy/reports")
    return base_dir / f"report_cartesian_{stem}.csv", base_dir / f"report_joints_{stem}.csv"

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="CSV file(s) or folder(s) of CSVs to analyze")
    ap.add_argument("-oc", "--out-cartesian", help="Override path for Cartesian report CSV")
    ap.add_argument("-oj", "--out-joints", help="Override path for Joints report CSV")
    ap.add_argument("--no-save", action="store_true", help="Print to stdout but do not write the CSV")
    args = ap.parse_args()

    df = load(args.paths)
    df = normalize_columns(df)
    df, joint_cols = add_error_columns(df)
    
    report_cartesian = build_cartesian_report(df)
    report_joints = build_joint_report(df, joint_cols)

    if not report_cartesian.empty:
        report_cartesian = report_cartesian.round(4)
    if not report_joints.empty:
        report_joints = report_joints.round(4)

    print("\n=== Cartesian Report (Pos: mm, Rot: deg) ===")
    with pd.option_context("display.max_columns", None, "display.width", 260, "display.float_format", lambda x: f"{x:9.4f}"):
        print(report_cartesian.to_string(index=False) if not report_cartesian.empty else "No Cartesian data found.")

    print("\n=== Joints Report (Errors: deg) ===")
    with pd.option_context("display.max_columns", None, "display.width", 260, "display.float_format", lambda x: f"{x:9.4f}"):
        print(report_joints.to_string(index=False) if not report_joints.empty else "No Joint data found.")

    if not args.no_save:
        def_cart_path, def_joints_path = default_output_paths(args.paths)
        
        cart_path = Path(args.out_cartesian) if args.out_cartesian else def_cart_path
        joints_path = Path(args.out_joints) if args.out_joints else def_joints_path
        
        if not report_cartesian.empty:
            cart_path.parent.mkdir(parents=True, exist_ok=True)
            report_cartesian.to_csv(cart_path, index=False)
            print(f"\nCartesian report saved to {cart_path}")
            
        if not report_joints.empty:
            joints_path.parent.mkdir(parents=True, exist_ok=True)
            report_joints.to_csv(joints_path, index=False)
            print(f"Joints report saved to {joints_path}")

if __name__ == "__main__":
    main()