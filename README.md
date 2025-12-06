# 🚀 How to Set Up and Run the HRI Tangram Gaze‑LLM System
This guide explains how to install dependencies, configure Webots, start the gaze tracking + LLM servers, and launch the robot simulation.

## 📂 1. Create and Activate Virtual Environment
_.\.venv\Scripts\activate_ <br>
Activates the project's Python virtual environment so all dependencies install locally instead of globally.

## 📦 2. Install IKPy (robot inverse kinematics library)
_C:\Users\HM\AppData\Local\Programs\Python\Python311\python.exe -m pip install git+https://github.com/alters-mit/ikpy.git#egg=ikpy_ <br>
This installs IKPy directly from the MIT-alters GitHub repository. <br>
Webots requires IKPy for inverse kinematics when controlling the robot arm. <br>

## 📦 3. Install All Remaining Dependencies
Option A — install packages individually:
_C:\Users\HM\AppData\Local\Programs\Python\Python311\python.exe -m pip install fastapi uvicorn requests opencv-python mediapipe numpy ikpy_ <br>
Option B — install automatically from requirements.txt: <br>
_C:\Users\HM\AppData\Local\Programs\Python\Python311\python.exe -m pip install -r requirements.txt_ <br>

**This installs:**
FastAPI → for the gaze evaluation server <br>
Uvicorn → runs the FastAPI server <br>
Requests → used by Webots to call the servers <br>
OpenCV + Mediapipe → for webcam gaze tracking <br>
NumPy → mathematical operations <br>
IKPy → robot inverse kinematics <br>

## 🤖 4. Tell Webots to Use Your Virtual Environment Python
_$env:WEBOTS_PYTHON = (Resolve-Path .\.venv\Scripts\python.exe).Path_ <br>
This ensures Webots runs controllers using the exact same venv Python where your libraries are installed. <br>

## 🟩 5. Launch Webots
_& "C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe"_ <br>
This opens the Webots simulator where the Tangram robot environment will run. <br>

## 🎯 6. Calibrate Gaze Tracking
_python calibrategaze.py_ <br>
During calibration, follow the yellow dots for 2 seconds each. <br>
This step lets Mediapipe learn your eye geometry for accurate gaze detection. <br>

## 🧠 7. Start the Local LLM Agent
_python llm_agent.py_ <br>
This runs the LLM recommendation server at: <br> <br>

_http://127.0.0.1:8001/recommend_piece_ <br>
Webots calls this to get the next puzzle piece suggestion. <br>

## 🔍 8. Start the Gaze Evaluation Server (FastAPI)
Open a second terminal, activate the venv, then run: <br>
_cd WeBots/gaze_algo_ <br>
_.\.venv\Scripts\python.exe -m uvicorn gaze_server:app --host 127.0.0.1 --port 8000 --log-level info_ <br> <br>

This creates the gaze evaluation endpoint: <br>
_http://127.0.0.1:8000/evaluate_move_ <br> <br>

Webots continuously sends: <br>
- piece ID <br>
- gaze duration <br>
- board state <br>
The server returns Positive or Negative to guide robot behavior. <br>

## ▶️ 9. Start Webots Simulation
With: <br>
llm_agent.py running <br>
gaze_server running <br>
Webots open <br> <br>

Click Run inside Webots. <br> <br>

The system will now: <br>
- Track your gaze in real-time <br>
- Evaluate each robot move <br>
- Provide LLM-generated piece suggestions <br> <br>

Remove or place pieces according to your gaze feedback <br>
