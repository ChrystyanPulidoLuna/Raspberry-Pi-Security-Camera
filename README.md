# 📷 Security Camera System

A Python-based security camera system that allows a Raspberry Pi to stream video over a network while a client device records the feed.

## Features
- Live video streaming from a Raspberry Pi using `picamera2`.
- Client-side viewing & recording using OpenCV.
- Motion detection recording (optional).
- Configurable FPS & resolution.
- Cross-platform support (Windows, Mac, Linux).

---

## Installation
1. Install Dependencies
On both the Raspberry Pi (Server) and the Client (Viewer/Recorder), install the required Python packages:

```bash
pip install -r requirements.txt
