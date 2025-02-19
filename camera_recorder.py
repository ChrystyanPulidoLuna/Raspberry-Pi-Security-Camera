import socket
import struct
import time
from picamera2 import Picamera2
import cv2
import numpy as np

class CameraServer:
    def __init__(self, host='0.0.0.0', port=8000):
        # Initialize camera with correct color format
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"size": (1280, 720), "format": "RGB888"},  # Changed to BGR format
            encode="main"
        ))
        self.picam2.start()
        
        # Initialize network
        self.server_socket = socket.socket()
        self.server_socket.bind((host, port))
        self.server_socket.listen(0)
        
        print(f"Server listening on port {port}")
    
    def run(self):
        while True:
            # Accept client connection
            client_socket, addr = self.server_socket.accept()
            print(f"Connection from: {addr}")
            try:
                while True:
                    # Capture frame
                    frame = self.picam2.capture_array()
                    
                    # Convert to jpg for efficient transmission
                    _, img_encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    
                    # Send frame size followed by frame data
                    data = img_encoded.tobytes()
                    size = len(data)
                    client_socket.sendall(struct.pack('>L', size) + data)
                    
                    # Small delay to control frame rate
                    time.sleep(0.03)  # ~30 FPS
                    
            except (ConnectionResetError, BrokenPipeError):
                print("Client disconnected")
                client_socket.close()
            except Exception as e:
                print(f"Error: {e}")
                client_socket.close()

if __name__ == '__main__':
    server = CameraServer()
    server.run()
