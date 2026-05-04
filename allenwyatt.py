# COMP 424 ELEC 424 - Final Project
# Inspired by: User raja_961, "Autonomous Lane-Keeping Car Using Raspberry Pi and OpenCV"
# Instructables. URL: https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/
# Built for Raspberry Pi 5 using gpiozero for hardware PWM.

import cv2
import numpy as np
import time
import os
import csv
import threading
import socketserver
import http.server
from gpiozero import Servo

ESC_PIN = 18
SERVO_PIN = 19

SHOW_DISPLAY = False
CAMERA_DEVICE = "/dev/video0"
ENCODER_PATH = "/sys/module/encoder_driver/parameters/speed_rpm"

print("Initializing ESC on GPIO18 and Servo on GPIO19...")
print("Video stream: http://168.5.172.32:8080")

esc = Servo(
	ESC_PIN,
	initial_value=0,
	min_pulse_width=0.001,
	max_pulse_width=0.002,
	frame_width=0.02
)

servo = Servo(
	SERVO_PIN,
	initial_value=0,
	min_pulse_width=0.001,
	max_pulse_width=0.002,
	frame_width=0.02
)

def set_esc(value):
	esc.value = max(-1.0, min(1.0, value))

def set_servo(value):
	servo.value = max(-1.0, min(1.0, value))

def neutral():
	set_esc(0)
	set_servo(0)

# PD controller for steering

Kp_steer = 0.025
Kd_steer = 0.005
prev_steer_error = 0

# Speed controller
Kp_speed = 0.001
target_speed = 300.0
current_esc_throttle = 0.20

# State machine
state = "DRIVING"
stop_count = 0
cooldown_end_time = 0

csv_file = open("run_data.csv", mode="w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
	"Frame",
	"Error",
	"P_Response",
	"D_Response",
	"Steer_Duty",
	"Speed_Duty",
	"Measured_Speed"
])

def read_encoder_speed():
	try:
		with open(ENCODER_PATH, "r") as f:
			val = float(f.read().strip())
			if val > 5000:
				return 0.0
			return val
	except Exception as e:
		return 0.0

cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

if not cap.isOpened():
	print("ERROR: Could not open", CAMERA_DEVICE)
	neutral()
	csv_file.close()
	exit(1)

print("Camera opened successfully:", CAMERA_DEVICE)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

latest_frame = None
frame_lock = threading.Lock()

class StreamHandler(http.server.BaseHTTPRequestHandler):
	def do_GET(self):
		self.send_response(200)
		self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
		self.end_headers()
		while True:
			with frame_lock:
				if latest_frame is None:
					continue
				_, jpg = cv2.imencode('.jpg', latest_frame)
			self.wfile.write(b'--frame\r\n')
			self.send_header('Content-type', 'image/jpeg')
			self.end_headers()
			self.wfile.write(jpg.tobytes())
			self.wfile.write(b'\r\n')
			time.sleep(0.05)

	def log_message(self, format, *args):
		pass

server = socketserver.TCPServer(('', 8080), StreamHandler)
threading.Thread(target=server.serve_forever, daemon=True).start()

stop_cd = 0
def process_frame(frame, frame_num):
	global prev_steer_error
	global state
	global stop_count
	global cooldown_end_time
	global current_esc_throttle
	global latest_frame
	global stop_cd

	current_time = time.time()

	# Red stop box detection every 3 frames
	if frame_num % 3 == 0:
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		lower_red1 = np.array([0, 100, 100])
		upper_red1 = np.array([10, 255, 255])
		lower_red2 = np.array([160, 100, 100])
		upper_red2 = np.array([180, 255, 255])

		red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | \
				   cv2.inRange(hsv, lower_red2, upper_red2)

		contours, _ = cv2.findContours(
			red_mask,
			cv2.RETR_TREE,
			cv2.CHAIN_APPROX_SIMPLE
		)

		red_detected = False

		for cnt in contours:
			if cv2.contourArea(cnt) > 1000:
				red_detected = True
				break

		if state == "DRIVING" and red_detected and current_time > cooldown_end_time:
			stop_count += 1
			stop_cd = 10
			state = "STOP_CD"
			print("Red Box", stop_count, "Detected!")
		if state == "STOP_CD":
			stop_cd -= 1
			if stop_cd <= 0:
				neutral()

				if stop_count == 1:
					state = "STOP1"
					cooldown_end_time = current_time + 3.0
				else:
					state = "STOP2"

	if state == "STOP1":
		neutral()
		if current_time > cooldown_end_time:
			print("Resuming driving...")
			state = "DRIVING"
			cooldown_end_time = current_time + 4.0
		with frame_lock:
			latest_frame = frame.copy()
		return frame

	if state == "STOP2":
		neutral()
		with frame_lock:
			latest_frame = frame.copy()
		return frame

	# Lane keeping
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	blur = cv2.GaussianBlur(gray, (5, 5), 0)
	edges = cv2.Canny(blur, 50, 150)

	h, w = edges.shape
	roi = np.zeros_like(edges)

	cv2.fillPoly(
		roi,
		[np.array([[(0, h), (w, h), (w, int(h * 0.7)), (0, int(h * 0.7))]])],
		255
	)

	cropped_edges = cv2.bitwise_and(edges, roi)

	lines = cv2.HoughLinesP(
		cropped_edges,
		1,
		np.pi / 180,
		20,
		minLineLength=15,
		maxLineGap=10
	)

	road_center = w // 2

	if lines is not None:
		left_xs = []
		right_xs = []

		for line in lines:
			x1, y1, x2, y2 = line[0]

			if x1 == x2:
				continue

			slope = (y2 - y1) / (x2 - x1)

			if slope < -0.3:
				left_xs.extend([x1, x2])
			elif slope > 0.3:
				right_xs.extend([x1, x2])

		lx = sum(left_xs) // len(left_xs) if left_xs else 0
		rx = sum(right_xs) // len(right_xs) if right_xs else w

		road_center = (lx + rx) // 2

	error = road_center - (w // 2)
	derivative = error - prev_steer_error

	p_response = Kp_steer * error
	d_response = Kd_steer * derivative

	steering_val = p_response + d_response
	prev_steer_error = error

	set_servo(steering_val)

	measured_speed = read_encoder_speed()
	speed_error = target_speed - measured_speed

	MAX_THROTTLE_CHANGE = 0.0005
	new_throttle = current_esc_throttle + Kp_speed * speed_error
	new_throttle = max(0.20, min(0.22, new_throttle))
	diff = new_throttle - current_esc_throttle
	diff = max(-MAX_THROTTLE_CHANGE, min(MAX_THROTTLE_CHANGE, diff))
	current_esc_throttle += diff

	set_esc(max(current_esc_throttle, 0.05))

	if frame_num % 2 == 0:
		print(
			"frame =", frame_num,
			"speed_error =", speed_error,
			"speed =", measured_speed,
			"throttle =", current_esc_throttle,
			"steering_error=", error,
			"steer_der=", derivative,
			"steer_val=", steering_val
		)

	csv_writer.writerow([
		frame_num,
		error,
		p_response,
		d_response,
		steering_val,
		current_esc_throttle,
		measured_speed
	])

	# Build debug frame
	debug = frame.copy()
	if lines is not None:
		for line in lines:
			x1, y1, x2, y2 = line[0]
			cv2.line(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
	cv2.line(debug, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
	cv2.line(debug, (road_center, 0), (road_center, h), (0, 0, 255), 1)

	with frame_lock:
		latest_frame = debug

	return frame

def main():
	print("\nArming ESC...")
	set_esc(0)
	time.sleep(3)
	print("ESC armed, starting navigation...")

	print("\n--- STARTING AUTONOMOUS NAVIGATION ---")

	try:
		MAX_FRAMES = 5000
		frame_count = 0

		while frame_count < MAX_FRAMES:
			ret, frame = cap.read()

			if not ret:
				print("ERROR: Camera read failed at frame", frame_count)
				break

			display_frame = process_frame(frame, frame_count)

			if SHOW_DISPLAY:
				cv2.imshow("Self-Driving View", display_frame)
				if cv2.waitKey(1) & 0xFF == ord("q"):
					break

			frame_count += 1

		print("\nMax frames reached or manually quit.")

	except KeyboardInterrupt:
		print("\nKeyboardInterrupt detected. Stopping vehicle.")

	finally:
		print("Shutting down... setting neutral.")
		neutral()
		esc.detach()
		servo.detach()
		cap.release()

		if SHOW_DISPLAY:
			cv2.destroyAllWindows()

		csv_file.close()
		print("Data saved to run_data.csv")

if __name__ == "__main__":
	main()
