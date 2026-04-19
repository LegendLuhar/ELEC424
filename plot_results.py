#!/usr/bin/env python3
"""
plot_results.py – Generate the two rubric-required plots from a run_log.csv
produced by lane_keeping.py.

Plot 1: error, steering duty cycle (%), and speed duty cycle (%) vs frame number
Plot 2: error, proportional response, derivative response vs frame number

Run after a track run:
    python3 plot_results.py              # uses run_log.csv in same directory
    python3 plot_results.py my_log.csv  # use a specific file
"""

import sys
import os
import csv
import matplotlib.pyplot as plt

# ── Load Data ────────────────────────────────────────────────────────────────

def load_log(path):
    # Read CSV into column lists for easy plotting
    frames, errors, steer_duty, throttle_duty, props, derivs = [], [], [], [], [], []

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frames.append(int(row["frame"]))
            errors.append(float(row["error"]))
            steer_duty.append(float(row["steering_pct"]))
            throttle_duty.append(float(row["throttle_pct"]))
            props.append(float(row["proportional"]))
            derivs.append(float(row["derivative"]))

    return frames, errors, steer_duty, throttle_duty, props, derivs


# ── Plot 1: error, steering %, speed % vs frame ──────────────────────────────

def plot_performance(frames, errors, steer_duty, throttle_duty, out_path):
    fig, ax1 = plt.subplots(figsize=(12, 5))

    # Error on the left y-axis (degrees)
    color_err = "tab:blue"
    ax1.set_xlabel("Frame number")
    ax1.set_ylabel("Steering error (degrees)", color=color_err)
    ax1.plot(frames, errors, color=color_err, linewidth=1, label="Error (deg)")
    ax1.tick_params(axis="y", labelcolor=color_err)
    ax1.axhline(0, color="grey", linestyle="--", linewidth=0.8)

    # Duty cycles on the right y-axis (percent)
    ax2 = ax1.twinx()
    ax2.set_ylabel("Duty cycle (%)")
    ax2.plot(frames, steer_duty,   color="tab:orange", linewidth=1, label="Steering duty %")
    ax2.plot(frames, throttle_duty, color="tab:green",  linewidth=1, label="Throttle duty %")
    ax2.set_ylim(5, 10)   # expected range: 6–9 % for steering, ~7–8 % throttle

    # Combine legends from both axes
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

    plt.title("Plot 1 – Error, Steering Duty Cycle, and Speed Duty Cycle vs Frame")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"Plot 1 saved to {out_path}")
    plt.show()


# ── Plot 2: error, P response, D response vs frame ───────────────────────────

def plot_pid(frames, errors, props, derivs, out_path):
    fig, ax = plt.subplots(figsize=(12, 5))

    ax.plot(frames, errors, color="tab:blue",   linewidth=1, label="Error (deg)")
    ax.plot(frames, props,  color="tab:orange", linewidth=1, label="Proportional response (Kp × e)")
    ax.plot(frames, derivs, color="tab:red",    linewidth=1, label="Derivative response (Kd × Δe/Δt)")
    ax.axhline(0, color="grey", linestyle="--", linewidth=0.8)

    ax.set_xlabel("Frame number")
    ax.set_ylabel("Value (degrees or scaled)")
    ax.set_title("Plot 2 – Error, Proportional Response, and Derivative Response vs Frame")
    ax.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"Plot 2 saved to {out_path}")
    plt.show()


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    log_path = sys.argv[1] if len(sys.argv) > 1 else \
               os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_log.csv")

    if not os.path.exists(log_path):
        print(f"ERROR: Log file not found: {log_path}")
        print("Run lane_keeping.py first to generate run_log.csv")
        sys.exit(1)

    print(f"Loading {log_path}…")
    frames, errors, steer_duty, throttle_duty, props, derivs = load_log(log_path)
    print(f"  {len(frames)} frames loaded")

    base = os.path.dirname(log_path)
    plot_performance(frames, errors, steer_duty, throttle_duty,
                     os.path.join(base, "plot1_performance.png"))
    plot_pid(frames, errors, props, derivs,
             os.path.join(base, "plot2_pid.png"))


if __name__ == "__main__":
    main()
