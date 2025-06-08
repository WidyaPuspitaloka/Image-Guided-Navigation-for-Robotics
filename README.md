# Image-Guided Navigation for Robotics

An integrated surgical planning and robotic simulation system that bridges medical imaging with robotic control for minimally invasive neurosurgery applications.

# 📹 Demo Video

A demonstration video showing the complete workflow from 3D Slicer path planning to robot execution is available:


https://github.com/user-attachments/assets/35570a16-6f63-4b27-b59a-7cbaf5bbffe8


*Complete walkthrough: Testing → Path planning → OpenIGTLink setup → Robot execution*

## 🎯 Project Overview

This project develops an end-to-end image-guided robotic navigation system that combines:
- **3D Slicer** for surgical path planning with anatomical constraints
- **OpenIGTLink** for real-time communication between imaging and robotic systems
- **ROS2** for robot simulation and control
- **MoveIt2** for motion planning and execution

The system demonstrates trajectory planning for deep brain procedures (e.g., hippocampal targeting) while avoiding critical structures like vessels, ventricles, and cortex.

### 🏆 **Key Achievements**
- **Safety Performance**: 6mm vessel clearance, 30mm ventricle clearance (exceeds clinical minimums)
- **Planning Efficiency**: 36-second complete trajectory optimization with 53.8% safety filtering
- **Clinical Validation**: Tested on whole-brain parcellation datasets with FULL SAFETY classification
- **Real-time Communication**: Zero data corruption during medical-to-robotic coordinate transmission

## ✨ Key Features

### 3D Slicer Path Planning Module
- **Hard Constraints**: Safety-critical requirements (collision avoidance, target validation, entry angle limits)
- **Soft Constraints**: Multi-criteria optimization (distance maximization, path length minimization)

### OpenIGTLink Communication Bridge
- Real-time TCP/IP communication between 3D Slicer and ROS2
- Coordinate system integrity preservation (RAS ↔ Robot workspace)
- Anatomical structure visualization (2,020+ brain structure points)
- Error handling and connection status monitoring

### ROS2 Robot Controller
- 6-DOF robotic arm simulation with URDF/SRDF configuration
- MoveIt2 integration for motion planning and execution
- Workspace boundary validation and collision detection
- Coordinate transformation pipeline (Medical RAS → Robot workspace)

## 📋 Before You Start

### Required Files
- Brain MRI volume (`.nii.gz`)
- Anatomical structures as separate label maps:
  - Hippocampus (target structure)
  - Blood vessels 
  - Ventricles
  - Cortex
- Entry points fiducial file (`.fcsv`)
- Target points fiducial file (`.fcsv`)

### File Format Requirements
- **MUST be** `vtkMRMLMarkupsFiducialNode` for fiducials
- **MUST be** `vtkMRMLLabelVolumeNode` for label maps
- Fiducial points **MUST** be named exactly "entry_point" and "target_point"

### Software Requirements
- **3D Slicer** (latest version) - Host machine
- **ROS2 Humble** (or compatible distribution) - VM
- **Ubuntu 22.04** or compatible Linux distribution - VM
- **Python 3.8+**

### Hardware Requirements
- Minimum 8GB RAM (16GB recommended)
- GPU support recommended for visualization
- Network capability for OpenIGTLink communication
- VMware Fusion or similar virtualization software

## 📦 Installation

### System Architecture Overview
This project requires setup on **TWO systems**:
- **Host Machine** (Mac/Windows): Runs 3D Slicer with path planning extension
- **Virtual Machine** (Ubuntu): Runs ROS2 robot simulation system

### 1. Host Machine Setup (3D Slicer)

#### Install 3D Slicer
```bash
# Download and install 3D Slicer from official website
# Install OpenIGTLinkIF extension through Extension Manager
```

#### Clone Repository for Slicer Extension
```bash
# On your host machine (Mac/Windows)
cd ~/Desktop  # or wherever you prefer
git clone https://github.com/WidyaPuspitaloka/Image-Guided-Navigation-for-Robotics.git
```

#### Install 3D Slicer Extension

**Method 1: Extension Wizard (Standard Approach)**
1. **Open 3D Slicer** on your host machine
2. **Go to:** Developer Tools → Extension Wizard  
3. **Click:** "Select Extension"
4. **Browse to and select:**
(e.g.) ~/Desktop/Image-Guided-Navigation-for-Robotics/path_planning_3dslicer/
5. **Wait for extension to load** (may take a few moments)
6. **Find PathPlanning** in modules dropdown (search for "PathPlanning" or check IGT category)
7. **Click "Reload and Test"** (wait 2-3 minutes for tests to complete)
8. **Look for "ALL TESTS COMPLETED"** confirmation

**⚠️ Method 2: Use this method if Extension Wizard causes conflicts or doesn't work**

1. **Open 3D Slicer** on your host machine
2. **Go to:** Edit → Application Settings → Modules
3. **Click:** "Add" button (next to "Additional module paths")
4. **Browse to and select:**
~/Desktop/Image-Guided-Navigation-for-Robotics/path_planning_3dslicer/PathPlanningMod/
5. **Click:** OK
6. **Restart 3D Slicer**
7. **Find PathPlanning** in modules dropdown (search for "PathPlanning" or check IGT category)
8. **Click "Reload and Test"** (wait 2-3 minutes for tests to complete)
9. **Look for "ALL TESTS COMPLETED"** confirmation

### 2. Virtual Machine Setup (ROS2)

#### Install ROS2 Environment
```bash
# In your VM terminal
sudo apt update
sudo apt install ros-humble-desktop

# Install dependencies
sudo apt install ros-humble-moveit
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-moveit-visual-tools
```

#### Create ROS2 Workspace and Clone Repository
```bash
# Create workspace (if you haven't already)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository for ROS2 components
git clone https://github.com/WidyaPuspitaloka/Image-Guided-Navigation-for-Robotics.git

# Install OpenIGTLink and ROS2-IGTL-Bridge (if you haven't already)
git clone https://github.com/openigtlink/ros2_igtl_bridge.git
```

#### Extract ROS2 Packages to Main src Directory
**⚠️ Critical Step: ROS2 packages must be at the top level of src/**

```bash
cd ~/ros2_ws/src

# Copy packages from nested location to main src directory
cp -r Image-Guided-Navigation-for-Robotics/robot_simulation_ros2_workspace/src/* .

# Verify packages are now in the correct location
ls ~/ros2_ws/src/
# You should see: my_robot_goal, six_dof_arm_description, six_dof_arm_moveit (at the top level)
```

#### Build OpenIGTLink from Source
```bash
cd ~
git clone https://github.com/openigtlink/OpenIGTLink.git
mkdir OpenIGTLink-build && cd OpenIGTLink-build
cmake ../OpenIGTLink
make
sudo make install
```

#### Build packages
```bash
# Build only your main packages (faster and safer)
cd ~/ros2_ws
colcon build --packages-select my_robot_goal six_dof_arm_description six_dof_arm_moveit

# Source the workspace (CRITICAL STEP!)
source install/setup.bash

# Add to bashrc to auto-source (recommended)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

#### Verify Installation
```bash
# Test if packages are built correctly
ros2 pkg list | grep -E "(my_robot_goal|six_dof_arm)"
# Should show all three packages

# Your final structure should look like:
# ~/ros2_ws/src/
# ├── my_robot_goal/                      # ← At the correct level
# ├── six_dof_arm_description/            # ← At the correct level
# ├── six_dof_arm_moveit/                 # ← At the correct level
# ├── ros2_igtl_bridge/
# ├── Image-Guided-Navigation-for-Robotics/  # Keep for reference
# └── OpenIGTLink/
```

## ✅ Quick Test (Recommended First Step)

### Test the System Before Using Your Data
1. **Open 3D Slicer** (Host Machine)
2. **Find PathPlanning** in modules dropdown (IGT category or search)
3. **Click "Reload and Test"** 
4. **Wait 2-3 minutes** for all tests to complete
5. **Look for "ALL TESTS COMPLETED"** confirmation

This ensures everything is working before you load your own data!

## 🚀 Quick Start Guide

### Step 1: Launch ROS2 System (VM First!)
```bash
# IMPORTANT: Always source your workspace first!
cd ~/ros2_ws
source install/setup.bash

# Launch complete robot system
ros2 launch my_robot_goal robot_plan.launch.py
```
**Wait for:** "Waiting for connection..." message before proceeding to Slicer

### Step 2: 3D Slicer Workflow (Host Machine)
1. **Load Your Data** (or use test data):
   - Brain MRI volume
   - Anatomical label maps (hippocampus, vessels, ventricles, cortex)
   - Entry and target fiducials

2. **Run Path Planning**:
   - Open **Needle Path Planning** module
   - Select appropriate columns/inputs
   - Click **"Plan Trajectory"** (wait 30-60 seconds)
   - Review results and safety metrics

3. **Setup Communication**:
   - Open **OpenIGTLinkIF** module
   - Create connector: **Server**, **Active**, Port **18944**
   - Click **"Send Complete Data"** when ROS2 shows "Connection established"
  
<img width="400" alt="slicer-openigtlink-extension" src="https://github.com/user-attachments/assets/d58d5f62-86c5-4377-982b-e6c7d67635ce" />  <img width="400" alt="slicer-send" src="https://github.com/user-attachments/assets/4dc6964b-39fe-488c-900c-35f2f20d7946" />



### Step 3: Robot Execution (VM)
1. **In RViz** (opens automatically):
   - Add **MarkerArray** display
   - Set topic to **`/brain_structures`**
   - Wait for brain structure points to appear

2. **Execute Movement**:
   - Use **RvizVisualTool** GUI panel
   - Click **"Next"** to progress through planning stages
   - Watch robot move to planned trajectory

## 🐛 Common Issues & Solutions

### Setup Issues

**"PathPlanning module not found in Slicer"**
```bash
# Use direct module loading instead of Extension Wizard:
# 1. Edit → Application Settings → Modules
# 2. Add path: ~/Desktop/Image-Guided-Navigation-for-Robotics/path_planning_3dslicer/PathPlanningMod/
# 3. Restart Slicer
# 4. Search for "PathPlanning" in modules dropdown
```

**"colcon build fails - package not found"**
```bash
# Make sure packages are at the top level of src/
cd ~/ros2_ws/src
ls  # Should see my_robot_goal, six_dof_arm_* at this level

# If packages are nested, copy them out:
cp -r Image-Guided-Navigation-for-Robotics/robot_simulation_ros2_workspace/src/* .
```

**"Duplicate package names error"**
```bash
# Remove old package versions if they exist
cd ~/ros2_ws/src
rm -rf old_package_name  # or move to backup folder
# Keep only one version of each package
```

**"Package not found when launching"**
```bash
# Make sure you've sourced the workspace
cd ~/ros2_ws
source install/setup.bash

# Verify package exists
ros2 pkg list | grep my_robot_goal

# Run from the main workspace directory (~/ros2_ws), not src/
```

### Communication Issues

**"No brain structures visible in RViz"**
```bash
# Check if MarkerArray is properly configured
# In RViz: Add → MarkerArray
# Topic: /brain_structures
# Wait 10-15 seconds for data transmission
```

**"Connection fails between Slicer and ROS2"**
```bash
# Correct startup sequence:
# 1. Launch ROS2 FIRST (wait for "Waiting for connection...")
# 2. THEN activate Slicer OpenIGTLink server
# 3. Look for "Connection established" message in VM terminal
```

## 🧪 Testing and Validation

### Built-in Module Testing

The 3D Slicer extension includes comprehensive built-in tests that run automatically when you click **"Reload and Test"** in the module interface.

#### **Test Coverage**

The built-in tests include:
- **5 Unit Tests**: Individual component functionality
- **2 Integration Tests**: Component interaction validation  
- **4 System Tests**: End-to-end workflow validation
- **5 Edge Case Tests**: Boundary conditions and error handling

**Expected Output**:
```
======= RUNNING UNIT TESTS =======
Unit test PASSED: Target identification
Unit test PASSED: Entry angle validation
Unit test PASSED: Collision detection
Unit test PASSED: Distance calculation
Unit test PASSED: Trajectory optimization

======= RUNNING INTEGRATION TESTS =======
Integration test PASSED: Validation pipeline
Integration test PASSED: Optimization pipeline

======= RUNNING SYSTEM TESTS =======
System test PASSED: Complete planning workflow
System test PASSED: Negative Cases
System test PASSED: Parameter Extremes
System test PASSED: Resource Limitations

======= FINAL TEST SUMMARY =======
Overall: 14/14 passed
```

## ⚙️ Customization and Personalization

### Robot Configuration

#### Using Your Own Robot
1. **Configure with MoveIt2 Setup Assistant**:
   ```bash
   ros2 launch moveit_setup_assistant setup_assistant.launch.py
   ```
   
2. **Update Parameters**:
   ```yaml
   # In user_parameters.yaml
   planning_group: "your_robot_arm"    # Match your MoveIt config
   base_link: "your_base_link"         # Your robot's base frame
   end_effector_link: "your_ee_link"   # Your end effector frame
   ```

3. **Launch Your Robot**:
   ```bash
   ros2 launch your_robot_moveit your_robot.launch.py
   ros2 launch needle_path_simulation needle_insertion.launch
   ```

### Parameter Tuning

#### Critical Parameters (user_parameters.yaml)
```yaml
# Path Planning
max_trajectory_length: 80.0  # mm - adjust for your application
max_entry_angle: 55.0        # degrees - steeper = more tissue damage
vessel_safety_margin: 2.0    # mm - minimum clearance from vessels
ventricle_safety_margin: 1.0 # mm - minimum clearance from ventricles

# Robot Control
velocity_scaling: 0.1        # 0.05-0.2 recommended for surgical precision
planning_timeout: 10.0       # seconds - increase for complex scenarios
planning_attempts: 30        # increase if planning often fails
jump_threshold: 0.0          # 0.0 for maximum flexibility

# Visualization
needle_orientation: [0, 0, 1]  # needle direction in end effector frame
```

#### Workspace Adjustment
```yaml
# Robot workspace bounds (adjust to match your robot)
workspace_bounds:
  x_min: 0.2    # meters
  x_max: 0.9
  y_min: -0.4
  y_max: 0.4
  z_min: 0.4
  z_max: 1.0
```

## 🎨 Visualization and User Experience

### RViz Display Configuration

For complete visualization, ensure these components are enabled in RViz:

#### Essential Displays
- **MotionPlanning**: Primary robot visualization and control interface
- **MarkerArray**: Topic `/brain_structures` - to show brain structure points
- **RvizVisualToolsGui**: Essential interaction panel (add via `Panels` → `RvizVisualToolsGui` if missing)

#### Interactive Elements
- **Next Button**: Progress through planning stages

<img width="400" alt="rviz-gui" src="https://github.com/user-attachments/assets/4bf47afb-4fdb-4da1-8588-ad5716ef8a58" />

## 📈 Future Improvements

### Short-term Enhancements
- [ ] Real-time planning (target <10 seconds)
- [ ] Patient-specific coordinate scaling
- [ ] Enhanced error handling and recovery
- [ ] Improved visualisation with mesh-based structures

### Long-term Development
- [ ] Integration with actual surgical robots
- [ ] Intraoperative replanning capabilities
- [ ] Multi-robot coordination
- [ ] Clinical validation studies


## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **7MRI0070 Course Team** at King's College London
- **3D Slicer Community** for imaging platform support
- **ROS/MoveIt2 Community** for robotics framework
- **OpenIGTLink Project** for communication protocol

## 📞 Contact

**Widya Ayu Puspitaloka**  
MSc Healthcare Technologies 
King's College London  
Email: widya.puspitaloka@kcl.ac.uk  
GitHub: [@WidyaPuspitaloka](https://github.com/WidyaPuspitaloka)

---

*This project was developed as part of the 7MRI0070: Image-guided Navigation for Robotics course at King's College London.*
