#!/usr/bin/env python3
"""
ELEC 424 / COMP 424 Final Project - Autonomous Lane-Keeping Car
Raspberry Pi 5 Implementation

Citation: User raja_961, "Autonomous Lane-Keeping Car Using Raspberry Pi and OpenCV"
Instructables. URL:
https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/

Adapted for RPi 5 with:
  - RPi.GPIO library for PWM control on GPIO 18 (throttle) and GPIO 19 (steering)
  - ESC + servo PWM control at 50 Hz (7.5% duty = neutral/straight)
  - Optical speed encoder feedback via kernel module sysfs
  - Red stop-box detection
  - CSV data logging for PID analysis plots
"""

import os
import cv2
import numpy as np
import math
import time
import lgpio   # direct lgpio for RPi 5 (gpiochip4)
import csv
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

# ── GPIO Pin Configuration (BCM numbering) ──────────────────────────────────
# These match the Project 3 spec: pin 18 = ESC/throttle, pin 19 = steering servo
THROTTLE_PIN = 18   # ESC signal wire (white) → GPIO 18
STEERING_PIN = 19   # Servo signal wire (white) → GPIO 19

# ── PWM Duty Cycle Settings (50 Hz, period = 20 ms) ─────────────────────────
# 7.5% = 1.5 ms pulse = neutral/straight for both ESC and servo
DUTY_NEUTRAL  = 7.5    # ESC neutral / servo straight
DUTY_FORWARD  = 8.0    # Forward speed
DUTY_MAX      = 8.2    # Hard cap
DUTY_LEFT     = 6.0    # Full left steering
DUTY_RIGHT    = 9.0    # Full right steering
PWM_FREQ      = 50    # Hz

# ── PD Controller Gains ──────────────────────────────────────────────────────
# Start: Kp_steer small, Kd_steer = 0; increase Kp until oscillation, then add Kd
Kp_steer = 0.8    # Proportional gain: steering error (degrees) → duty cycle change
Kd_steer = 0.0    # Derivative gain: set to 0 until P-only is stable
Kp_speed = 0.02   # Proportional gain: RPM error → duty cycle change

# ── Optical Encoder / Speed Settings ────────────────────────────────────────
TARGET_RPM    = 60    # Desired wheel RPM (tune to match comfortable track speed)
ENCODER_SYSFS = "/sys/module/encoder_driver/parameters/speed_rpm"

# ── Camera / Frame Settings ──────────────────────────────────────────────────
CAM_INDEX    = 0      # Webcam index – try 1 or 2 if 0 fails
FRAME_WIDTH  = 320    # Lower resolution = faster processing
FRAME_HEIGHT = 240
MAX_FRAMES   = 500    # Hard loop limit so motors always get neutralised on exit

# ── Lane Color (Blue tape) HSV Bounds ───────────────────────────────────────
BLUE_LOWER = np.array([100,  80,  20], dtype="uint8")   # dark blue tape
BLUE_UPPER = np.array([130, 255, 150], dtype="uint8")

# ── Stop-Box Color (Red paper) HSV Bounds ───────────────────────────────────
# Red wraps around hue=0/180 in HSV so two ranges are needed
RED_LOWER1 = np.array([0,   120,  70], dtype="uint8")
RED_UPPER1 = np.array([10,  255, 255], dtype="uint8")
RED_LOWER2 = np.array([160, 120,  70], dtype="uint8")
RED_UPPER2 = np.array([180, 255, 255], dtype="uint8")
STOP_AREA_THRESHOLD = 0.25   # 25% of ROI pixels red → stop detected
STOP_HOLD_FRAMES    = 30     # Frames to hold at stop before resuming
STOP_COUNT_MAX      = 2      # Permanently stop after the 2nd stop box


# ── GPIO / PWM Setup ─────────────────────────────────────────────────────────

GPIOCHIP = 4   # RPi 5 main user GPIO is on gpiochip4

def setup_gpio():
    # Open gpiochip4 and claim throttle + steering pins as outputs
    h = lgpio.gpiochip_open(GPIOCHIP)
    lgpio.gpio_claim_output(h, THROTTLE_PIN, 0)
    lgpio.gpio_claim_output(h, STEERING_PIN, 0)

    # Start both at neutral duty cycle
    lgpio.tx_pwm(h, THROTTLE_PIN, PWM_FREQ, DUTY_NEUTRAL)
    lgpio.tx_pwm(h, STEERING_PIN, PWM_FREQ, DUTY_NEUTRAL)

    return h


def stop_motors(h):
    # Return both channels to neutral
    lgpio.tx_pwm(h, THROTTLE_PIN, PWM_FREQ, DUTY_NEUTRAL)
    lgpio.tx_pwm(h, STEERING_PIN, PWM_FREQ, DUTY_NEUTRAL)
    time.sleep(0.1)


def cleanup_gpio(h):
    # Stop PWM and close GPIO chip
    lgpio.tx_pwm(h, THROTTLE_PIN, 0, 0)
    lgpio.tx_pwm(h, STEERING_PIN, 0, 0)
    lgpio.gpiochip_close(h)


# ── Lane-Detection Functions (adapted from raja_961 Instructable) ────────────

def convert_to_HSV(frame):
    # Convert BGR camera frame to HSV for colour-based masking
    return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


def detect_edges(hsv):
    # Isolate blue lane tape then find edges with Canny (50:100 = 1:2 ratio)
    mask  = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
    edges = cv2.Canny(mask, 50, 100)
    return edges


def region_of_interest(edges):
    # Bottom half — covers more of frame in case camera is mounted higher
    h, w   = edges.shape
    mask   = np.zeros_like(edges)
    poly   = np.array([[(0, h), (0, h // 2), (w, h // 2), (w, h)]], np.int32)
    cv2.fillPoly(mask, poly, 255)
    return cv2.bitwise_and(edges, mask)


def detect_line_segments(cropped):
    # Probabilistic Hough transform to find line-segment endpoints
    return cv2.HoughLinesP(
        cropped, rho=1, theta=np.pi / 180, threshold=10,
        lines=np.array([]), minLineLength=2, maxLineGap=5
    )


def get_steering_from_mask(hsv):
    """Centroid-based steering — reliable when tape is close to camera and nearly horizontal."""
    h, w = hsv.shape[:2]
    roi   = hsv[h // 2:, :]              # bottom half
    mask  = cv2.inRange(roi, BLUE_LOWER, BLUE_UPPER)
    m     = cv2.moments(mask)
    if m["m00"] < 30:                     # too few blue pixels → not reliable
        return None
    cx       = int(m["m10"] / m["m00"])
    error_px = cx - w // 2               # positive = centroid right of center
    # Negative sign: centroid right → steer left (away from that tape edge)
    angle = 90 - int(error_px * 45.0 / (w // 2))
    return max(45, min(135, angle))


def make_points(frame, line):
    # Convert (slope, intercept) to pixel endpoints spanning bottom→mid of frame
    h, _, _ = frame.shape
    slope, intercept = line
    if slope == 0:
        slope = 0.1   # avoid divide-by-zero for near-horizontal lines
    y1 = h
    y2 = h // 2
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return [[x1, y1, x2, y2]]


def average_slope_intercept(frame, line_segments):
    # Group Hough segments into left/right lanes and average each group
    lane_lines = []
    if line_segments is None:
        return lane_lines

    h, w, _       = frame.shape
    left_fit      = []
    right_fit     = []
    left_boundary  = w * (2 / 3)   # x > this → right region
    right_boundary = w * (1 / 3)   # x < this → left region

    for seg in line_segments:
        for x1, y1, x2, y2 in seg:
            if x1 == x2:
                continue   # skip vertical lines (undefined slope)
            slope     = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1
            # Negative slope + left side of frame → left lane line
            if slope < 0 and x1 < left_boundary and x2 < left_boundary:
                left_fit.append((slope, intercept))
            # Positive slope + right side of frame → right lane line
            elif slope > 0 and x1 > right_boundary and x2 > right_boundary:
                right_fit.append((slope, intercept))

    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, np.average(left_fit,  axis=0)))
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, np.average(right_fit, axis=0)))

    return lane_lines


def display_lines(frame, lines, color=(0, 255, 0), thickness=6):
    # Overlay detected lane lines on a copy of the frame (green)
    overlay = np.zeros_like(frame)
    if lines:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(overlay, (x1, y1), (x2, y2), color, thickness)
    return cv2.addWeighted(frame, 0.8, overlay, 1, 1)


def get_steering_angle(frame, lane_lines):
    # Calculate desired steering angle in degrees from detected lane endpoints
    h, w, _ = frame.shape
    mid      = w // 2

    if len(lane_lines) == 2:
        # Both lanes visible: aim for midpoint between their upper endpoints
        _, _, lx2, _ = lane_lines[0][0]
        _, _, rx2, _ = lane_lines[1][0]
        x_offset = (lx2 + rx2) / 2 - mid
    elif len(lane_lines) == 1:
        # Only one lane: follow its direction vector
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        x_offset = 0   # no lines found – go straight

    y_offset       = h / 2
    angle_rad      = math.atan(x_offset / y_offset)
    steering_angle = int(angle_rad * 180.0 / math.pi) + 90  # 90° = straight
    return steering_angle


def display_heading_line(frame, steering_angle, color=(0, 0, 255), thickness=5):
    # Draw a red heading line showing the car's intended direction
    overlay   = np.zeros_like(frame)
    h, w, _   = frame.shape
    angle_rad  = steering_angle / 180.0 * math.pi

    x1 = w // 2
    y1 = h
    if abs(math.sin(angle_rad)) < 1e-6:
        angle_rad += 0.001   # prevent tan(0) degenerate case
    x2 = int(x1 - (h / 2) / math.tan(angle_rad))
    y2 = h // 2

    cv2.line(overlay, (x1, y1), (x2, y2), color, thickness)
    return cv2.addWeighted(frame, 0.8, overlay, 1, 1)


# ── Stop-Box Detection ───────────────────────────────────────────────────────

def detect_stop_box(frame):
    # Check the bottom half of the frame for a red region above the area threshold
    roi_start = frame.shape[0] // 2
    roi       = frame[roi_start:, :]

    hsv   = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
    mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    red   = cv2.bitwise_or(mask1, mask2)

    red_px   = cv2.countNonZero(red)
    total_px = roi.shape[0] * roi.shape[1]
    return (red_px / total_px) > STOP_AREA_THRESHOLD


# ── Encoder Speed ────────────────────────────────────────────────────────────

def read_encoder_speed():
    # Read wheel RPM from the kernel module sysfs parameter
    try:
        with open(ENCODER_SYSFS, "r") as f:
            return int(f.read().strip())
    except (FileNotFoundError, ValueError):
        return -1   # encoder driver not loaded; caller uses open-loop fallback


# ── MJPEG Stream Server ──────────────────────────────────────────────────────

_stream_frame = None
_stream_lock  = threading.Lock()

class _MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        while True:
            with _stream_lock:
                frame = _stream_frame
            if frame is None:
                time.sleep(0.05)
                continue
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ok:
                continue
            try:
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                                 + jpg.tobytes() + b"\r\n")
            except Exception:
                break

    def log_message(self, *args):
        pass   # suppress per-request logs


def start_stream(port=8080):
    server = HTTPServer(("0.0.0.0", port), _MJPEGHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Video stream: http://168.5.172.32:{port}")


# ── Steering Duty Cycle Conversion ──────────────────────────────────────────

def angle_to_duty(steering_angle):
    # Map steering angle (degrees) to duty cycle (%)
    # 90° = straight = 7.5%;  each degree offset = 0.033% (spans 6–9% over ±45°)
    offset = (steering_angle - 90) * (1.5 / 45.0)
    duty   = DUTY_NEUTRAL + offset
    return max(DUTY_LEFT, min(DUTY_RIGHT, duty))


# ── Main Entry Point ─────────────────────────────────────────────────────────

def main():
    start_stream(port=8080)
    h = setup_gpio()

    # Arm the ESC: hold neutral for 2 s (required before first use)
    print("Arming ESC – hold still for 2 seconds…")
    time.sleep(2)

    # Open webcam and set resolution
    video = cv2.VideoCapture(CAM_INDEX)
    video.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not video.isOpened():
        print(f"ERROR: Could not open camera {CAM_INDEX}")
        cleanup_gpio(h)
        sys.exit(1)

    # Warm up the camera – first several frames are black
    print("Warming up camera...")
    for _ in range(30):
        video.read()
    time.sleep(1)

    # Per-frame data log – written to CSV on exit for rubric plots
    log = []

    # PD controller state
    last_error = 0
    last_time  = time.time()

    # Tape-loss failsafe counter
    no_detect_frames = 0

    # Stop-box state machine
    stop_count     = 0      # stops completed so far
    stopped_frames = 0      # frames elapsed since current stop began
    is_stopped     = False  # whether the car is currently holding a stop

    print(f"Running lane keeping for up to {MAX_FRAMES} frames. Press ESC to quit.")

    try:
        for frame_num in range(MAX_FRAMES):

            ret, frame = video.read()
            if not ret:
                print("Camera read failed – exiting")
                break

            # ── Stop-Box Check every 3rd frame (reduces CPU load) ─────────────
            if frame_num % 3 == 0:
                red_seen = detect_stop_box(frame)

                if red_seen and not is_stopped:
                    stop_count    += 1
                    is_stopped     = True
                    stopped_frames = 0
                    print(f"Stop box #{stop_count} at frame {frame_num}")
                    stop_motors(h)

                    if stop_count >= STOP_COUNT_MAX:
                        print("Final stop box – parking permanently.")
                        break

                elif not red_seen and is_stopped:
                    stopped_frames += 1
                    if stopped_frames >= STOP_HOLD_FRAMES:
                        is_stopped = False   # resume driving

            if is_stopped:
                cv2.waitKey(1)
                continue

            # ── Lane Detection Pipeline ───────────────────────────────────────
            hsv = convert_to_HSV(frame)

            # Primary: centroid of blue pixels (robust when tape is at bottom of frame)
            steering_angle = get_steering_from_mask(hsv)
            tape_detected  = steering_angle is not None

            if not tape_detected:
                # Fallback: Hough line detection when centroid has too few pixels
                edges      = detect_edges(hsv)
                roi        = region_of_interest(edges)
                segments   = detect_line_segments(roi)
                lane_lines = average_slope_intercept(frame, segments)
                if lane_lines:
                    tape_detected = True
                steering_angle = get_steering_angle(frame, lane_lines)

            # Failsafe: if tape not found, count consecutive misses
            if tape_detected:
                no_detect_frames = 0
            else:
                no_detect_frames += 1
                if no_detect_frames >= 5:
                    print(f"[frame {frame_num}] FAILSAFE: tape lost for {no_detect_frames} frames – stopping throttle")
                    lgpio.tx_pwm(h, THROTTLE_PIN, PWM_FREQ, DUTY_NEUTRAL)
                    lgpio.tx_pwm(h, STEERING_PIN, PWM_FREQ, DUTY_NEUTRAL)
                    continue

            # Save raw frame + mask at frame 5 for HSV tuning inspection
            if frame_num == 5:
                cv2.imwrite("/tmp/frame_raw.jpg", frame)
                snap_roi = hsv[frame.shape[0] // 2:, :]
                cv2.imwrite("/tmp/frame_mask.jpg", cv2.inRange(snap_roi, BLUE_LOWER, BLUE_UPPER))

            # Print blue pixel count every 30 frames for live diagnostics
            if frame_num % 30 == 0:
                dbg_roi  = hsv[frame.shape[0] // 2:, :]
                dbg_mask = cv2.inRange(dbg_roi, BLUE_LOWER, BLUE_UPPER)
                print(f"[frame {frame_num}] blue_px={cv2.countNonZero(dbg_mask)}  steer={steering_angle}")

            # Push annotated frame to MJPEG stream
            disp = display_heading_line(display_lines(frame, []), steering_angle)
            with _stream_lock:
                global _stream_frame
                _stream_frame = disp

            # ── PD Controller – Steering ──────────────────────────────────────
            now   = time.time()
            dt    = max(now - last_time, 1e-6)
            error = steering_angle - 90   # 0 = straight; positive = veer right

            proportional = Kp_steer * error
            derivative   = Kd_steer * (error - last_error) / dt
            pd_output    = proportional + derivative

            # Convert PD output to duty cycle and command servo
            steer_duty = angle_to_duty(90 + pd_output)
            lgpio.tx_pwm(h, STEERING_PIN, PWM_FREQ, steer_duty)

            # ── Speed Controller (encoder feedback) ───────────────────────────
            current_rpm = read_encoder_speed()
            if current_rpm >= 0:
                speed_error  = TARGET_RPM - current_rpm
                throttle_duty = DUTY_FORWARD + Kp_speed * speed_error
            else:
                throttle_duty = DUTY_FORWARD   # open-loop fallback

            # Slow down slightly when turning hard, but keep minimum 50% throttle
            turn_factor   = max(0.5, 1.0 - abs(error) / 90.0)
            throttle_duty = DUTY_NEUTRAL + (throttle_duty - DUTY_NEUTRAL) * turn_factor
            throttle_duty = max(DUTY_NEUTRAL, min(DUTY_MAX, throttle_duty))
            lgpio.tx_pwm(h, THROTTLE_PIN, PWM_FREQ, throttle_duty)

            # ── Log data for rubric plots ─────────────────────────────────────
            log.append({
                "frame":        frame_num,
                "error":        error,
                "steering_pct": steer_duty,
                "throttle_pct": throttle_duty,
                "proportional": proportional,
                "derivative":   derivative,
            })

            last_error = error
            last_time  = now

            # cv2.imshow("Lane Keeping", heading_frame)
            if cv2.waitKey(1) == 27:   # ESC key quits
                break

    finally:
        # Always return motors to neutral regardless of how loop ended
        print("Shutting down – returning motors to neutral")
        stop_motors(h)
        video.release()
        cv2.destroyAllWindows()

        # Write CSV for plot_results.py
        if log:
            log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "run_log.csv")
            with open(log_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=log[0].keys())
                writer.writeheader()
                writer.writerows(log)
            print(f"Run data saved to {log_path}")

        cleanup_gpio(h)
        print("Done.")


if __name__ == "__main__":
    main()
