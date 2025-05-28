# GuardBot
![Adobe Express - file](https://github.com/user-attachments/assets/72c98f8d-a6cb-4017-bd0a-56c2b236c2a1)

![Adobe Express - file](https://github.com/user-attachments/assets/4f444fa7-426d-44c7-bf14-89ad8aba4e7c)


# Guard-Bot-Scout

A differential-drive autonomous robot that fuses stereo-depth and 3D LiDAR data for real-time exploration and SLAM-based mapping. It overlays live person-detection results onto the generated map, enabling precise localization and tagging of individuals as the robot navigates its environment.

(IN PROGRESS ONCE I HAVE A MORE FINALIZED VERSION I WILL POST HOW TO RUN IT)
## Features

- **Differential-drive platform** with real-time SLAM-based mapping  
- **Stereo-depth + 3D LiDAR sensor fusion** pipeline for robust environment perception  
- **Live person-detection overlay** powered by YOLOv8 Nano (`yolov8n.pt`) 
- **Modular ROS 2 architecture** for isolated development and testing  
- **RViz2 visualization** support for map and detection markers  

## Repository Structure

```
├── LICENSE                     Apache 2.0 License 
├── README.md                   This file  
├── yolov8n.pt                  Pre-trained YOLOv8 Nano weights
└── src/                        Source code (ROS 2 packages)  
    ├── guard_bot/              Core sensor-fusion & detection nodes  
    ├── scout_slam/             SLAM mapping and exploration  
    └── scout_description/      URDF robot description & launch files  
```

## Technology Stack

- **Framework:** ROS 2 Humble Hawksbill  
- **Perception:** OpenCV, PyTorch (YOLOv8)  
- **Mapping:** OUSTER OSx (DTOF)  
- **Visualization:** RViz2  

## Prerequisites

- Ubuntu 22.04  
- ROS 2 Humble Hawksbill  
- `colcon` build tool  
- Python 3.8+ with `torch`, `opencv-python`  

For the person detection you will need to run the publisher and subscriber seperatly or make a launch file to run both. 
(In progress) Added YOLO to detect people but currently the publisher runs from the onboard jetson orin nano and we subscribe on a laptop. (Not required but this is how were are doing the set up currently)

Then open RViz2 with the provided config:

## License

This project is released under the Apache 2.0 License.
