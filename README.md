# ME459 Final Project: Weighted A* Pathfinding for UAVs  

## Description  
This repository contains the final project for **ME459**, which investigates the effect of weighted A* pathfinding algorithms on quadcopter navigation in constrained 3D spaces. The project includes Python scripts for implementing weighted A* algorithms, offboard MAVROS control, and pathfinding analysis. The repository also contains a presentation, report, and demonstration video showcasing the results.  

## Files Included  

### **Part 1: Weighted A* Algorithm for UAV Pathfinding**  
- **File:** ME 459 Final Project.py  
- **Topics Covered:**  
  - Weighted A* pathfinding  
  - 3D configuration space modeling  
  - Path cost evaluation and optimization  
  - Dynamic obstacle avoidance  

### **Part 2: MAVROS Offboard Control Scripts**  
- **File:** mavros_offboard_posctl_test_raw.py  
- **File:** mavros_offboard_posctl_weighted_astar.py  
- **File:** mavros_test_common.py  
- **Topics Covered:**  
  - MAVROS communication with PX4 flight stack  
  - Offboard position control for UAVs  
  - Real-time trajectory adjustments  
  - Autonomous flight in simulated environments  

### **Part 3: Project Documentation**  
- **File:** ME 459 Final Report.pdf  
- **File:** ME 459 Final Presentation.pptx  
- **Contents:**  
  - Overview of weighted A* and its implications  
  - Experimental setup and results analysis  
  - Path cost and computation time comparison  
  - Conclusion and future work directions  

### **Part 4: ROS Launch File for MAVROS**  
- **File:** px4.launch  
- **Topics Covered:**  
  - ROS launch configuration for PX4 and MAVROS  
  - UAV startup parameters and namespace configuration  

### **Part 5: Demonstration Video**  
- **File:** DroneWeightedAstar.mp4  
- **Contents:**  
  - Flight demonstration of weighted A* pathfinding  
  - Comparison of different weighting factors  
  - Visual representation of UAV trajectory in constrained spaces  

## Installation  
Ensure Python, ROS, and MAVROS are installed before running the scripts.

### Required Python Packages  
- numpy  
- matplotlib  
- rospy  
- pymavlink  

To install the necessary packages, run:  
```pip install numpy matplotlib rospy pymavlink```  

## Usage  

### **Running the Weighted A* Pathfinding Algorithm**  
1. Open a terminal or command prompt.  
2. Navigate to the directory containing `ME 459 Final Project.py`.  
3. Run the script using:  
   ```python ME 459 Final Project.py```  
4. View the plotted 3D path and computed path cost.  

### **Executing MAVROS Offboard Control**  
1. Ensure MAVROS and PX4 simulation are running.  
2. Start the MAVROS offboard position control script:  
   ```rosrun mavros_offboard_posctl_weighted_astar.py```  
3. Observe the UAV following the weighted A* computed path.  

### **Viewing the Demonstration Video**  
1. Open `DroneWeightedAstar.mp4` using any media player.  
2. Watch the effects of different weighting factors on UAV trajectory.  

## Example Output  

- **Weighted A* Performance Metrics**  
  - Weight = 1: Path cost = 96.8 m, Compute time = 10.5 s  
  - Weight = 5: Path cost = 105.7 m, Compute time = 9.9 s  
  - Weight = 1000: Path cost = 129.5 m, Compute time = 54.4 s  

- **UAV Path Execution**  
  - Successfully reached target at (90, 5, 2)  
  - Computation time savings observed with moderate weights  

## Contributions  
This repository is intended for academic research and educational use. Contributions and modifications are welcome.  

## License  
This project is open for research and educational purposes.  

---  
**Author:** Alexander Dowell  

