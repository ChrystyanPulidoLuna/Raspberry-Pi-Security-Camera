import socket
import struct
import cv2
import numpy as np
import tkinter as tk
from tkinter import messagebox, ttk
from PIL import Image, ImageTk
import threading
import time
import os
from datetime import datetime
import sys

class CameraClient:
    def __init__(self, host, port=8000):
        self.host = host
        self.port = port
        self.is_connected = False
        self.is_recording = False
        self.is_motion_recording = False
        self.frame = None
        self.prev_frame = None
        self.motion_detected = False
        self.resolution = (640, 480)  # Default lower resolution
        self.fps = 15  # Default lower fps
        self.setup_gui()
        
    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("Pi Camera Remote Viewer")
        
        # Get the path to the icon file
        if getattr(sys, 'frozen', False):
            script_dir = sys._MEIPASS
        else:
            # Running as a normal Python script
            script_dir = os.path.dirname(os.path.abspath(__file__))

        icon_path = os.path.join(script_dir, "camera_icon.ico")

    # Set the window icon
        self.root.iconbitmap(icon_path)

        # Main frame
        main_frame = tk.Frame(self.root)
        main_frame.pack(padx=10, pady=10)
        
        # Connection frame
        conn_frame = tk.Frame(main_frame)
        conn_frame.pack(fill='x', pady=5)
        
        self.host_entry = tk.Entry(conn_frame, width=15)
        self.host_entry.insert(0, self.host)
        self.host_entry.pack(side='left', padx=5)
        
        self.connect_btn = tk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side='left', padx=5)
        
        # Settings frame
        settings_frame = tk.Frame(main_frame)
        settings_frame.pack(fill='x', pady=5)
        
        # Resolution dropdown
        tk.Label(settings_frame, text="Resolution:").pack(side='left', padx=5)
        self.resolution_var = tk.StringVar(value="640x480")
        resolutions = ["320x240", "640x480", "800x600", "1280x720"]
        self.resolution_combo = ttk.Combobox(settings_frame, textvariable=self.resolution_var, 
                                           values=resolutions, width=10, state='readonly')
        self.resolution_combo.pack(side='left', padx=5)
        
        # FPS dropdown
        tk.Label(settings_frame, text="FPS:").pack(side='left', padx=5)
        self.fps_var = tk.StringVar(value="15")
        fps_options = ["5", "10", "15", "20", "30"]
        self.fps_combo = ttk.Combobox(settings_frame, textvariable=self.fps_var,
                                     values=fps_options, width=5, state='readonly')
        self.fps_combo.pack(side='left', padx=5)
        
        # Quality slider
        tk.Label(settings_frame, text="Quality:").pack(side='left', padx=5)
        self.quality_var = tk.IntVar(value=80)
        self.quality_slider = ttk.Scale(settings_frame, from_=0, to=100, 
                                      variable=self.quality_var, orient='horizontal')
        self.quality_slider.pack(side='left', padx=5)
        
        # Status label
        self.status_label = tk.Label(main_frame, text="Disconnected", fg="red")
        self.status_label.pack(pady=5)
        
        # Video frame
        self.video_label = tk.Label(main_frame)
        self.video_label.pack(pady=10)
        
        # Recording controls
        ctrl_frame = tk.Frame(main_frame)
        ctrl_frame.pack(fill='x', pady=5)
        
        # Regular recording button
        self.record_btn = tk.Button(ctrl_frame, text="Start Recording", 
                                   command=self.toggle_recording, state='disabled')
        self.record_btn.pack(side='left', padx=5)
        
        # Motion detection recording button
        self.motion_record_btn = tk.Button(ctrl_frame, text="Start Motion Recording",
                                         command=self.toggle_motion_recording, state='disabled')
        self.motion_record_btn.pack(side='left', padx=5)
        
        # Storage info label
        self.storage_label = tk.Label(ctrl_frame, text="")
        self.storage_label.pack(side='left', padx=5)
        
        self.recording_label = tk.Label(ctrl_frame, text="")
        self.recording_label.pack(side='left', padx=5)
        
        # Recording timer
        self.timer_label = tk.Label(main_frame, text="00:00:00", font=("Arial", 16))
        self.timer_label.pack(pady=5)
        
        # Motion status
        self.motion_status = tk.Label(main_frame, text="No Motion", fg="gray")
        self.motion_status.pack(pady=5)
        
        # Bind resolution and FPS changes
        self.resolution_combo.bind('<<ComboboxSelected>>', self.update_settings)
        self.fps_combo.bind('<<ComboboxSelected>>', self.update_settings)
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def toggle_connection(self):
        if not self.is_connected:
            self.connect()
        else:
            self.disconnect()
    
    def connect(self):
        try:
            self.client_socket = socket.socket()
            self.client_socket.connect((self.host_entry.get(), self.port))
            self.is_connected = True
            
            self.status_label.config(text="Connected", fg="green")
            self.connect_btn.config(text="Disconnect")
            self.record_btn.config(state='normal')
            self.motion_record_btn.config(state='normal')
            
            # Start frame receiving thread
            self.receive_thread = threading.Thread(target=self.receive_frames)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
            # Start display update
            self.update_display()
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")
    
    def disconnect(self):
        self.is_connected = False
        if hasattr(self, 'client_socket'):
            self.client_socket.close()
        
        self.status_label.config(text="Disconnected", fg="red")
        self.connect_btn.config(text="Connect")
        self.record_btn.config(state='disabled')
        self.motion_record_btn.config(state='disabled')
        
        if self.is_recording:
            self.toggle_recording()
        if self.is_motion_recording:
            self.toggle_motion_recording()

    def update_settings(self, event=None):
        res_str = self.resolution_var.get()
        width, height = map(int, res_str.split('x'))
        self.resolution = (width, height)
        self.fps = int(self.fps_var.get())
        
        # Update video writer if recording
        if hasattr(self, 'video_writer') and (self.is_recording or self.is_motion_recording):
            self._restart_recording()
    
    def _restart_recording(self):
        # Store current state
        was_recording = self.is_recording
        was_motion_recording = self.is_motion_recording
        
        # Stop current recording
        if was_recording:
            self.toggle_recording()
        if was_motion_recording:
            self.toggle_motion_recording()
        
        # Restart recording with new settings
        if was_recording:
            self.toggle_recording()
        if was_motion_recording:
            self.toggle_motion_recording()
    
    def detect_motion(self, frame):
        if self.prev_frame is None:
            self.prev_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return False
        
        # Convert current frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Calculate absolute difference between frames
        frame_delta = cv2.absdiff(self.prev_frame, gray)
        
        # Apply threshold to highlight differences
        thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
        
        # Dilate thresholded image to fill in holes
        thresh = cv2.dilate(thresh, None, iterations=2)
        
        # Find contours in thresholded image
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Update previous frame
        self.prev_frame = gray
        
        # Check if any contour is large enough to be considered motion
        motion_threshold = (self.resolution[0] * self.resolution[1]) / 1000  
        for contour in contours:
            if cv2.contourArea(contour) > motion_threshold:
                return True
        
        return False
    
    def receive_frames(self):
        try:
            while self.is_connected:
                # Read frame size
                size_data = self.client_socket.recv(4)
                if not size_data:
                    break
                
                size = struct.unpack('>L', size_data)[0]
                
                # Read frame data
                data = b''
                while len(data) < size:
                    packet = self.client_socket.recv(size - len(data))
                    if not packet:
                        break
                    data += packet
                
                # Decode frame
                nparr = np.frombuffer(data, np.uint8)
                self.frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
        except:
            self.disconnect()
    
    def update_display(self):
        if self.is_connected and self.frame is not None:
            # Resize frame to current resolution setting
            frame_resized = cv2.resize(self.frame, self.resolution)
            
            # Convert frame to RGB
            frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
            
            # Check for motion if motion recording is active
            if self.is_motion_recording:
                self.motion_detected = self.detect_motion(frame_resized)
                self.motion_status.config(
                    text="Motion Detected!" if self.motion_detected else "No Motion",
                    fg="red" if self.motion_detected else "gray"
                )
            
            # Convert to PhotoImage
            image = Image.fromarray(frame_rgb)
            photo = ImageTk.PhotoImage(image=image)
            
            # Update label
            self.video_label.config(image=photo)
            self.video_label.image = photo
            
            # Write frame if recording (with compression)
            if self.is_recording or (self.is_motion_recording and self.motion_detected):
                # Apply JPEG compression before writing
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality_var.get()]
                _, compressed = cv2.imencode('.jpg', frame_resized, encode_param)
                frame_compressed = cv2.imdecode(compressed, cv2.IMREAD_COLOR)
                
                self.video_writer.write(frame_compressed)
                
                # Update recording duration
                duration = time.time() - self.recording_start_time
                hours = int(duration // 3600)
                minutes = int((duration % 3600) // 60)
                seconds = int(duration % 60)
                self.timer_label.config(text=f"{hours:02d}:{minutes:02d}:{seconds:02d}")
        
        # Schedule next update based on FPS
        self.root.after(int(1000/self.fps), self.update_display)
    
    def toggle_recording(self):
        if not self.is_recording:
            # Start recording
            self._start_recording("regular")
            self.record_btn.config(text="Stop Recording")
        else:
            # Stop recording
            self._stop_recording("regular")
            self.record_btn.config(text="Start Recording")
    
    def toggle_motion_recording(self):
        if not self.is_motion_recording:
            # Start motion recording
            self._start_recording("motion")
            self.motion_record_btn.config(text="Stop Motion Recording")
            self.motion_status.pack(pady=5)  # Show motion status
        else:
            # Stop motion recording
            self._stop_recording("motion")
            self.motion_record_btn.config(text="Start Motion Recording")
            self.motion_status.pack_forget()  # Hide motion status
    
    def _start_recording(self, mode):
        output_dir = os.path.expanduser("~/Videos")
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(output_dir, f"{mode}_recording_{timestamp}.mp4")
        
        # Use H.264 codec with MP4 container for better compression
        if os.name == 'nt':  # Windows
            fourcc = cv2.VideoWriter_fourcc(*'h264')
        else:  # Linux/Mac
            fourcc = cv2.VideoWriter_fourcc(*'avc1')
            
        self.video_writer = cv2.VideoWriter(
            filename, 
            fourcc, 
            self.fps,
            self.resolution,
            True
        )
        
        if mode == "regular":
            self.is_recording = True
        else:
            self.is_motion_recording = True
            
        self.recording_start_time = time.time()
        self.recording_label.config(text=f"Recording to: {filename}", fg="red")
        
        # Start storage monitoring
        self.update_storage_info()
    
    def _stop_recording(self, mode):
        if mode == "regular":
            self.is_recording = False
        else:
            self.is_motion_recording = False
            self.motion_detected = False
            self.prev_frame = None
        
        if hasattr(self, 'video_writer'):
            self.video_writer.release()
        
        self.recording_label.config(text="Recording saved", fg="green")
        self.timer_label.config(text="00:00:00")
        self.storage_label.config(text="")  # Clear storage info
    
    def update_storage_info(self):
        if hasattr(self, 'video_writer') and (self.is_recording or self.is_motion_recording):
            if os.path.exists(self.video_writer.getBackendName()):
                size_mb = os.path.getsize(self.video_writer.getBackendName()) / (1024 * 1024)
                self.storage_label.config(text=f"File size: {size_mb:.1f} MB")
                
            # Schedule next update
            self.root.after(1000, self.update_storage_info)
    
    def on_closing(self):
        if self.is_connected:
            self.disconnect()
        self.root.destroy()
    
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    # You can replace with your Raspberry Pi's IP address
    client = CameraClient(host="192.168.1.125")
    client.run()
    

