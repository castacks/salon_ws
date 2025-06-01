# salon_ws
Open Source Implementation of [SALON](https://theairlab.org/SALON)



Documentation coming soon, but this repo is set up as a ROS1 workspace. The launch files in src/context_adaptation will run the code necessary to reproduce the examples on different robots, specifically:
- online_hdif.launch
- online_hdif_wheelchair.launch
- online_hdif_anymal.launch

---

## 🚀 Getting Started

The main requirements are ros 1 (or ros 2 but these examples are for ros 1), and pytorch. The rest of the packages should be easy to install.

### 1. Clone the workspace

```bash
git clone https://github.com/castacks/salon_ws.git
cd salon_ws
catkin build
source devel/setup.bash
```

### 2. Download Data

#### Off-road ATV
Use the [TartanDrive 2.0 GUI](https://github.com/castacks/tartan_drive_2.0)
to download a sample bag. A good higher-speed example is "2023-11-14-14-26-22_gupta". The corresponding launch file is in `src/context_adaptation/online_hdif.launch`

#### Wheelchair
Data will be available soon.
The corresponding launch file is in `src/context_adaptation/online_hdif_wheelchair.launch`

#### Wild Visual Navigation
Follow the instructions to set up [Wild Visual Navigation](https://github.com/leggedrobotics/wild_visual_navigation) separately, as we use their velocity-tracking cost function that the provide. As mentioned [here](https://github.com/leggedrobotics/wild_visual_navigation/issues/310) it needs to be tuned a bit to get something more expressive on their sample data. A good example to run on is their "MPI_Outdoor_Seq_1" example.

The corresponding launch file is in `src/context_adaptation/online_hdif_anymal.launch`

---

## 🧪 Running the System
Recommended setup is as follows

Terminal 1:

```
roscore
```

Terminal 2:
```
roslaunch context_adaptation online_hdif.launch
```
Replace `online_hdif.launch` with the appropriate launch file

Terminal 3:
```
rosbag play **bag_name**
```

Terminal 4:
Open up rviz and observe the "shortrange_costmap" and "shortrange_speedmap" rgb_viz layers.


If the node doesn't start producing maps soon after the bag starts, try using sim time (i.e. `rosparam set use_sim_time true` and use --clock in rosbag play)

---
If you use our work please consider citing:
```
     @misc{sivaprakasam2024salonselfsupervisedadaptivelearning,
        title={SALON: Self-supervised Adaptive Learning for Off-road Navigation}, 
        author={Matthew Sivaprakasam and Samuel Triest and Cherie Ho and Shubhra Aich and Jeric Lew and Isaiah Adu and Wenshan Wang and Sebastian Scherer},
        year={2024},
        eprint={2412.07826},
        archivePrefix={arXiv},
        primaryClass={cs.RO},
        url={https://arxiv.org/abs/2412.07826}, 
  }
```
