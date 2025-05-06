Goodluck -tucker

---

# ATAK Bridge Repository

This repository provides tools to facilitate communication between ATAK (Android Team Awareness Kit) and robots. It includes components for listening to ATAK messages, processing them, and plotting paths based on received data.

---

## Table of Contents
1. Dependencies
2. Installation
3. How to Run
   - Running `atak_listener`
   - Running `plot_path`
4. File Structure

---

## Dependencies

Before running the programs, ensure you have the following dependencies installed:

### System Requirements
- **Operating System**: Linux (tested on Ubuntu)
- **Python Version**: Python 3.x (tested on Python 3.8+)
- **ROS Version**: ROS Noetic (or compatible version)

### Python Packages

Install the following packages manually:
```bash
pip install pandas tkinter opencv-python simplekml
```


### Additional Dependencies
- **pytak**: Install the latest version of `pytak` for ATAK communication:
  ```bash
  pip install pytak
  ```
- **SSL Certificates**: Ensure you have valid SSL certificates (`user2.pem`, user2.key) in the repository.

---

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/westpoint-robotics/atak_bridge.git
   git checkout MineSweeper-Branch
   cd atak_bridge
   ```

2. Install ROS dependencies:
   ```bash
   sudo rosdep install --from-paths src --ignore-src
   ```

3. Ensure the SSL certificates (`user2.pem`, user2.key) are in the repository's root directory.

---

## How to Run

### Running `atak_listener`

The `atak_listener` listens for messages from ATAK and processes them into CSV files. It also provides a GUI for monitoring received data.

#### Steps to Run:
Make sure to source the workspace if you make any changes
```bash
cd 
source devel/setup.bash
 ```
1. Run the `atak_listener`:
   ```bash
   roslaunch atak_bridge atak_listener.launch
   ```

2. GUI Instructions:
   - Click the **Start** button to begin listening for messages.
   - The GUI will display counts and coordinates for Fly zones, NoFly zones, Start points, and End points.
   - Click the **End** button to stop the program and combine the CSV files into `User_input_total.csv`.

#### Output:
- The following CSV files will be generated:
  - `start_point.csv`: Contains the start point.
  - `fly_zones.csv`: Contains Fly zones.
  - `nofly_zones.csv`: Contains NoFly zones.
  - `end_point.csv`: Contains the end point.
  - `User_input_total.csv`: Combines all the above files with separation lines.

---

### Running `plot_path`

The `plot_path` script reads a CSV file containing latitude and longitude points and sends them as CoT (Cursor on Target) messages to the ATAK server.

#### Steps to Run:
1. Ensure the Output_Path.csv file exists in the takpak directory. This file should contain the path points in the format:
   ```csv
   Latitude, Longitude
   41.3906814186454,-73.9529146243604
   41.3906821237462,-73.9529153685257
   ...
   ```

2. Run the `plot_path` script:
   ```bash
   roslaunch atak_bridge plot_path.launch
   ```

3. The script will send the points in Output_Path.csv to the ATAK server as CoT messages.

---

## File Structure

```
atak_bridge/
├── atak_bridge_basestation.py
├── atak_bridge_node.py
├── LatLongUTMconversion.py
├── map_converter.py
├── map_server.py
├── takpak/
│   ├── atak_listener.py
│   ├── cmdvel_drive.py
│   ├── end_point.csv
│   ├── fly_zones.csv
│   ├── mkcot.py
│   ├── nofly_zones.csv
│   ├── open.py
│   ├── Output_Path.csv
│   ├── plot_path.py
│   ├── start_point.csv
│   ├── takcot.py
│   ├── User_input_total.csv
│   └── README.md
├── test_robot.py
├── user2.key
├── user2.pem
└── README.md
```
