# 🚀 How to Set Up and Run the HRI Tangram Gaze‑LLM System
This guide explains how to install dependencies, configure Webots, start the gaze tracking + LLM servers, and launch the robot simulation.

## 📂 1. Create and Activate Virtual Environment
_.\.venv\Scripts\activate_
Activates the project's Python virtual environment so all dependencies install locally instead of globally.

## 📦 2. Install IKPy (robot inverse kinematics library)
_C:\Users\HM\AppData\Local\Programs\Python\Python311\python.exe -m pip install git+https://github.com/alters-mit/ikpy.git#egg=ikpy_
This installs IKPy directly from the MIT-alters GitHub repository.
Webots requires IKPy for inverse kinematics when controlling the robot arm.

## 📦 3. Install All Remaining Dependencies
Option A — install packages individually:
_C:\Users\HM\AppData\Local\Programs\Python\Python311\python.exe -m pip install fastapi uvicorn requests opencv-python mediapipe numpy ikpy_
Option B — install automatically from requirements.txt:
_C:\Users\HM\AppData\Local\Programs\Python\Python311\python.exe -m pip install -r requirements.txt_

**This installs:**
FastAPI → for the gaze evaluation server
Uvicorn → runs the FastAPI server
Requests → used by Webots to call the servers
OpenCV + Mediapipe → for webcam gaze tracking
NumPy → mathematical operations
IKPy → robot inverse kinematics

## 🤖 4. Tell Webots to Use Your Virtual Environment Python
_$env:WEBOTS_PYTHON = (Resolve-Path .\.venv\Scripts\python.exe).Path_
This ensures Webots runs controllers using the exact same venv Python where your libraries are installed.

## 🟩 5. Launch Webots
_& "C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe"_
This opens the Webots simulator where the Tangram robot environment will run.

## 🎯 6. Calibrate Gaze Tracking
_python calibrategaze.py_
During calibration, follow the yellow dots for 2 seconds each.
This step lets Mediapipe learn your eye geometry for accurate gaze detection.

## 🧠 7. Start the Local LLM Agent
_python llm_agent.py_
This runs the LLM recommendation server at:

_http://127.0.0.1:8001/recommend_piece_
Webots calls this to get the next puzzle piece suggestion.

## 🔍 8. Start the Gaze Evaluation Server (FastAPI)
Open a second terminal, activate the venv, then run:

_cd WeBots/gaze_algo_
_.\.venv\Scripts\python.exe -m uvicorn gaze_server:app --host 127.0.0.1 --port 8000 --log-level info_
This creates the gaze evaluation endpoint:

_http://127.0.0.1:8000/evaluate_move_
Webots continuously sends:
- piece ID
- gaze duration
- board state
The server returns Positive or Negative to guide robot behavior.

## ▶️ 9. Start Webots Simulation
With:
llm_agent.py running
gaze_server running
Webots open

Click Run inside Webots.

The system will now:
- Track your gaze in real-time
- Evaluate each robot move
- Provide LLM-generated piece suggestions

Remove or place pieces according to your gaze feedback
