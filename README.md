# Image-Guided Navigation for Robotics

An integrated surgical planning and robotic simulation system that bridges medical imaging with robotic control for minimally invasive neurosurgery applications.

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

## 🔧 Prerequisites

### Software Requirements
- **3D Slicer** (latest version)
- **ROS2 Humble** (or compatible distribution)
- **Ubuntu 22.04** or compatible Linux distribution
- **Python 3.8+**

### Hardware Requirements
- Minimum 8GB RAM (16GB recommended)
- GPU support recommended for visualization
- Network capability for OpenIGTLink communication

## 📦 Installation

### 1. 3D Slicer Setup

```bash
# Install 3D Slicer from official website
# Install OpenIGTLinkIF extension through Extension Manager
```

### 2. ROS2 Environment Setup

```bash
# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Install dependencies
sudo apt install ros-humble-moveit
sudo apt install ros-humble-joint-state-publisher-gui
```

### 3. Clone and Build Project

```bash
# Clone repository
cd ~/ros2_ws/src
git clone https://github.com/WidyaPuspitaloka/Image-Guided-Navigation-for-Robotics.git

# Install OpenIGTLink and ROS2-IGTL-Bridge
git clone https://github.com/openigtlink/ros2_igtl_bridge.git

# Build OpenIGTLink from source
cd ~
git clone https://github.com/openigtlink/OpenIGTLink.git
mkdir OpenIGTLink-build && cd OpenIGTLink-build
cmake ../OpenIGTLink
make
sudo make install

# Build ROS2 packages
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 4. 3D Slicer Extension Installation

```bash
# Open 3D Slicer
# Go to: Developer Tools → Extension Wizard
# Select the downloaded 3d_slicer_path_planning folder
# Load the extension when prompted
```


## 🚀 Usage

### Step 1: Launch ROS2 System

```bash
# Single terminal launch - starts everything automatically
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_robot_goal robot_plan.launch.py
```

**What this launch file does:**
- ✅ Starts the robot simulation and MoveIt2 configuration
- ✅ Launches OpenIGTLink bridge (waits for 3D Slicer connection)
- ✅ Initializes robot controller
- ✅ Opens RViz with proper display configuration
- ✅ Shows "Waiting for connection..." message until 3D Slicer connects

### Step 2: Configure 3D Slicer

1. **Load Data**:
   - Load brain MRI volume
   - Load anatomical structure label maps (hippocampus, vessels, ventricles, cortex)
   - Load entry and target point fiducials

2. **Run Path Planning**:
   - Open the Needle Path Planning extension
   - Select input volumes and fiducial lists (**Must be `vtkMRMLMarkupsFiducialNode` and `vtkMRMLLabelVolumeNode`**)
   - Click "Plan Trajectory" and wait (8-20 seconds depending on dataset size)
   - Review safety metrics and selected path
   - Click "Prepare data to send to ROS"

3. **Setup OpenIGTLink**:
   - Open OpenIGTLinkIF module
   - In Connectors, click `+`
   - Select **Server** and **Active** in Properties section
   - Use default Hostname (`localhost`) and Port (`18944`)
   - Expand I/O Configuration and add:
     - The created `vtkMRMLMarkupsFiducialNode` (ROS_entry_and_target_points)
     - The created `vtkMRMLModelNode` (brain structures)
   - **CRITICAL**: Fiducial points must be named exactly "entry_point" and "target_point"
   - Activate connection (status should show "ON")

<img width="749" alt="slicer-openigtlink-extension" src="https://github.com/user-attachments/assets/40c4f61f-0ade-4c3c-95f1-f7bc2609580e" />


### Step 3: Execute Robot Movement

1. **Verify Connection**:
   - ROS2 terminal should show "Connection established. Start the IGTL loop.."
   - 3D Slicer OpenIGTLinkIF status changes from "WAIT" to "ON"

2. **Send Trajectory Data**:
   - In 3D Slicer, click "Send to Robot"
   - Verify data reception in ROS2 terminal output

3. **Robot Execution**:
   - RViz opens automatically with proper visualization
   - Use RViz interface to visualize planned motion
   - Execute trajectory through MoveIt2 interface
   - Monitor safety margins and collision detection

**Note**: The system waits for 3D Slicer connection before proceeding - you'll see "Waiting for connection..." until OpenIGTLinkIF is activated.

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

## 🧪 Testing and Validation

### Component Testing
```bash
# Run unit tests for path planning
cd ~/ros2_ws/src/needle_path_planning/tests
python test_path_planning.py

# Test OpenIGTLink communication
ros2 run ros2_igtl_bridge test_communication

# Validate robot controller
ros2 test robot_controller_tests
```

### Built-in Module Testing

The 3D Slicer extension includes comprehensive built-in tests that run automatically when you click **"Reload and Test"** in the module interface.

#### **Modifying Test Data Paths**

To use your own test data, modify the hardcoded paths in the test file:

1. **Open the module file**: `PathPlanningMod.py`
2. **Find the test setup section** (around line 1400):
   ```python
   # ====== HARDCODED TEST DATA PATHS - MODIFY THESE AS NEEDED ======
   # Change this path to your test data directory
   self.testDataDir = "/Users/widyapuspitaloka/Desktop/TestSet"
   
   # Individual file paths - modify these if your files have different names
   self.brain_file = "fakeBrainTest.nii.gz"
   self.hippo_file = "r_hippoTest.nii.gz" 
   self.ventricles_file = "ventriclesTest.nii.gz"
   self.vessels_file = "vesselsTestDilate1.nii.gz"
   self.cortex_file = "r_cortexTest.nii.gz"
   self.entries_file = "entriesSubsample.fcsv"
   self.targets_file = "targetsSubsample.fcsv"
   ```

3. **Update paths to match your data**:
   ```python
   # Example: Update to your local test data directory
   self.testDataDir = "/path/to/your/test/data"
   
   # Example: Update file names if different
   self.hippo_file = "my_hippocampus.nii.gz"
   self.vessels_file = "my_vessels.nii.gz"
   # ... etc
   ```

4. **Click "Reload and Test"** in the module interface to run tests with your data

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
Overall: 11/11 passed
```

### End-to-End Validation
1. **Load test dataset** (BrainPlanning dataset)
2. **Execute complete pipeline** from planning to robot movement
3. **Verify coordinate accuracy** between planned and executed positions
4. **Validate safety constraints** throughout execution

## 🐛 Troubleshooting

### Common Issues

#### Build/Compilation Issues

**catkin_make/colcon build fails**
```bash
# Verify all necessary packages are installed
ros2 pkg list | grep moveit
ros2 pkg list | grep igtl

# Install missing packages
sudo apt-get update
sudo apt-get install ros-humble-moveit
sudo apt-get install ros-humble-moveit-visual-tools

# Remember to source your workspace!
source ~/ros2_ws/install/setup.bash
```

#### Path Planning Issues

**Planning Fails with "No Safe Trajectories"**
```bash
# Check input data validity - ensure correct vtkMRMLMarkupsFiducialNode format
# Verify anatomical structure labels are properly loaded
# Adjust safety margins in configuration
# Test with simplified dataset first
```

**Planning Fails Consistently/Robot Self-Collision**
- **Rotate/Translate Input Data**: Use 3D Slicer's [Transforms module](https://slicer.readthedocs.io/en/latest/user_guide/modules/transforms.html) to adjust brain model and needle path positions
- **Modify Robot Origins**: Edit URDF file origins in `<workspace>/src/robot_description/urdf/robot.urdf`
- **Adjust Planning Parameters**:
  ```yaml
  # In user_parameters.yaml
  jump_threshold: 0.0  # Change from 5.0 to 0.0
  planning_time_limit: 30.0  # Increase from default
  planning_attempts: 50  # Increase attempts
  ```

#### Communication Issues

**OpenIGTLink Connection Failed**
```bash
# Verify port availability
netstat -an | grep 18944

# Check firewall settings
sudo ufw status

# Correct startup sequence (IMPORTANT):
# 1. Launch ROS2 system: ros2 launch my_robot_goal robot_plan.launch.py
# 2. Wait for "Waiting for connection..." message
# 3. THEN activate 3D Slicer OpenIGTLinkIF server
# 4. Look for "Connection established" confirmation
```

## 🎨 Visualization and User Experience

### RViz Display Configuration

For complete visualization, ensure these components are enabled in RViz:

#### Essential Displays
- **MotionPlanning**: Primary robot visualization and control interface
- **MarkerArray**: Topic `/rviz_visual_tools` - for interactive buttons and text
- **Marker**: Topic `/brain_mesh` - for anatomical structure visualization
- **RvizVisualToolsGui**: Essential interaction panel (add via `Panels` → `RvizVisualToolsGui` if missing)


#### Interactive Elements
- **Next Button**: Progress through planning stages

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

## 📹 Demo Video

A comprehensive demonstration video showing the complete workflow from 3D Slicer path planning to robot execution is available:



*Complete walkthrough: Data loading → Path planning → OpenIGTLink setup → Robot execution*


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
