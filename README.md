<a name="readme-top"></a>


<div align="center">
<br />
  <h1 align="center">Two-layer adaptive trajectory tracking controller for quadruped robots on slippery terrains.</h1>

  
  <a href="https://github.com/despargy/maestro">
    <img src="go1_description/figures/go1.png" alt="Go1" width="358" height="235">
  </a>
  <h2 align="center">Maestro Project</h2> 

</div>



README.md file will be updated no later than Wednesday 8th March 2023.

## About the project

We propose an adaptive trajectory tracking controller
for quadruped robots, which involves two prioritized layers of
adaptation for avoiding possible slippage of one or multiple
legs.
### Description
This package is developed for the simulated and the real robot experiments of the submitted paper to IROS 2023 with title "Two-layer adaptive trajectory tracking controller for quadruped robots on slippery terrains". 

<!-- by Despina-Ekaterini Argiropoulos, Dimitrios Papageorgiou, Michael Maravgakis,
Drosakis Drosakis and Panos Trahanias.  -->
### Development and Implementation using <a href="https://www.unitree.com/en/go1">Unitree's Go1 EDU</a> legged robot.

<p align="right"><a href="#readme-top">Back to top</a></p>

## Getting Started

### Dependencies

#### Unitree's Go1 legged robot:

*  [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real)
* [unitree_ros](https://github.com/unitreerobotics/unitree_ros)
* [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk)

#### Probabilistic Contact Estimation
*  [ProbabilisticContactEstimation](https://github.com/MichaelMarav/ProbabilisticContactEstimation)

####  System 
* Ubuntu 20.04
* [ROS Noetic]
* [Gazebo](https://gazebosim.org/home)

### Installation
1. Get the above package dependencies: 
<br /> `unitree_ros_to_real`,  `unitree_ros`, `unitree_legged_sdk`, `ProbabilisticContactEstimation` .
1. Clone the repo
   ```sh
   git clone https://github.com/despargy/maestro
   ```
2. Catkin make
   ```sh
   cd ~/catkin_ws 
   catkin_make
   ```

### Run the package ( Simulation - without Adaptation ) (Default)
1. Default launch for simulation
   ```sh
   roslaunch maestro basic.launch
   ```
2. Run the controller / main_handler
    ```sh
   rosrun maestro main_handler
   ```
3. Terminate the controller / main_handler
    ```sh
   rostopic pub /maestro/ctrl std_msgs/Bool "data: false"
   ```
    
    
### Run the package (Simulation - Adaptation) 
1. Launch for simulation with adaptation parameters
   ```sh
   roslaunch maestro basic.launch slip_detection:=true adapt_b:=true
   ```
2. Launch ProbabilisticContactEstimation
   ```sh
   roslaunch contact_estimation contact.launch
   ```
3. Run the controller / main_handler
    ```sh
   rosrun maestro main_handler
   ```
4. Terminate the controller / main_handler
    ```sh
   rostopic pub /maestro/ctrl std_msgs/Bool "data: false"
   ```

### Run the package ( Real Robot* - without Adaptation ) 
1. Default launch for simulation
   ```sh
   roslaunch maestro basic.launch real_experiment:=true
   ```
2. Run the controller / main_handler
    ```sh
   rosrun maestro main_handler
   ```
3. Terminate the controller / main_handler
    ```sh
   rostopic pub /maestro/ctrl std_msgs/Bool "data: false"
   ```
    
### Run the package (Real Robot*  - Adaptation) 
1. Launch for simulation with adaptation parameters
   ```sh
   roslaunch maestro basic.launch real_experiment:=true slip_detection:=true adapt_b:=true
   ```
2. Launch ProbabilisticContactEstimation**
   ```sh
   roslaunch contact_estimation contact.launch
   ```
3. Run the controller / main_handler
    ```sh
   rosrun maestro main_handler
   ```
4. Terminate the controller / main_handler
    ```sh
   rostopic pub /maestro/ctrl std_msgs/Bool "data: false"
   ```

*Note: For Real Robot first follow the steps bellow to ping connection to 192.168.123.161 
(more details will be added - TODO)

**Note: ProbabilisticContactEstimation for real experiment needs an IMU sensor to publish at /imu topic (more details will be added - TODO)


## Parameters

## Experiment setup


