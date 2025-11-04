# 3D Cone Detection in LiDAR Point Clouds (WIP)
## Overview

This repository contains code and resources for evaluating preprocessing pipelines and neural network architectures for 3D cone detection in LiDAR point clouds, with a focus on applications in **Formula Student Driverless competitions**.

In these competitions, autonomous race cars navigate tracks outlined by colored traffic cones. Reliable cone detection is essential for **localization, mapping, and path planning**, as well as ensuring the vehicle can safely navigate at high speeds. 

For more background information [jump to the Background section](#background)
## Project Goals

The main objectives of this project are:

1. **Preprocessing LiDAR data:** Implement a pipeline that removes ground points using RANSAC, clusters object proposals with DBSCAN, and reconstructs cones for neural network input.
2. **Fine-tuning PointNet:** Use a pre-trained PointNet backbone to classify clusters as cones (yellow, blue, orange) or non-cones, exploring efficiency and accuracy trade-offs.
3. **Lightweight custom network:** Design a smaller neural network to achieve similar classification performance with reduced computational cost.

## Current state
### Pre-processing pipeline
The pre-processing pipeline is implemented. It includes the following steps.
1. Filter the air points and the points that correspond to the car
2. Segment the pointcloud in a grid of 64x64
3. RANSAC ground removal
4. DBSCAN clustering
5. Reconstruct the cones

A visual of the steps and the succes. A picture of the initial point-cloud:  
<br> </br>
![Pointcloud](images/original_pc.png)
<br> </br>
A picture of the ground removal:
<br> </br>
![GroundRemoval](images/ground_removal.png)
<br> </br>
A picture of the proposal centroids
<br></br>
![proposals](images/proposal_centroids.png)
<br></br>
## Create rosbridge from WSL to Foxglove on Windows
Download the rosbridge using the following command
```bash
sudo apt install -y ros-humble-foxglove-bridge
```

Run the rosbridge
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Open the rosbridge under connection on foxglove.

## Background
The Formula Student Competitions task student teams to design, manufacture, and compete with single-seater, open-wheel race cars. Starting in 2018, the driverless category has been added. In this category autonomous race cars such as shown in the picture below, navigate tracks outlined by colored traffic cones. I have quite some experience with this competition as I spent a full-time year as Chief Embedded & Driverless Software at Formula Student Team Delft. The picture below is actually from the car competing in the biggest competition where we got third place overall. 
<br></br>
![car](images/TU_DELFT_car.png)
<br></br>
The competition consists of four dynamic events: Acceleration, a straight line sprint; Skidpad, a figure eight to test cornering ability; Autocross, a single lap on an unknown track; and Trackdrive, a ten-lap endurance run on an unknown track. A visualization of the events can be found in Figure 1 below. The cones (blue on the left and yellow on the right) mark the boundaries of the track, with small orange cones indicating braking zones and big orange cones indicating start/finish lines. The cones have a predefined size and can be viewed in Figure 2 below.
<p align="center">
  <img src="images/acceleration.png" width="30%" />
  <img src="images/skidpad.png" width="30%" />
  <img src="images/trackdrive.png" width=30%>
</p>
<p align="center"><em>Figure 1: Acceleration - skidpad - Autocross and Trackdrive event.</em></p>
<p align="center">
    <img src="images/cones.png">
</p>
<p align="center"><em>Figure 2: The cones marking the track</em></p>

Cones provide crucial reference points that enable the vehicle to estimate its position
and generate a map of the track when navigating an unknown track. For reliable Simultaneous Localization and Mapping (SLAM) or localization using a known track
map, the computer vision pipeline of a Formula Student Driverless vehicle must detect
the cones, estimate their relative position to the car, and in some approaches also deter-
mine their color. The requirement to determine the color of the cone depends on the
SLAM and path planning approach. A final consideration is the latency and detection
distance. We can estimate the minimum detection range by $ddetection = vmax ·tlookahead =
vmax · N · dt = 15 m
s · 40 · 50 ms = 30 m $. To ensure that the perception system enables
optimal performance of subsequent pipeline stages, its latency should remain below
50ms. This is primarily because SLAM and state estimation performance degrade as
latency increases. Summarized, the requirements for the system are as follows:
1. Detect the type of cone and their relative position to the car
2. The system should be able to handle car racing.
3. The system should be able to deal with different light conditions.
4. The runtime of the system should be in the order of 10’s of ms.
