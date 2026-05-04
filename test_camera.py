import cv2
import threading
import socketserver
import http.server
import time

CAMERA_DEVICE = "/dev/video0"

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
                    time.sleep(0.05)
                    continue
                _, jpg = cv2.imencode('.jpg', latest_frame)
            try:
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')
            except:
                break
            time.sleep(0.05)

    def log_message(self, *args):
        pass

socketserver.TCPServer.allow_reuse_address = True
server = socketserver.TCPServer(('', 8080), StreamHandler)
threading.Thread(target=server.serve_forever, daemon=True).start()
print("Stream: http://168.5.128.74:8080")

cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

print("Press Ctrl+C to stop")
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed")
            break
        with frame_lock:
            latest_frame = frame.copy()
finally:
    cap.release()
