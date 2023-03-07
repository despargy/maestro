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
* [unitree\_ros](https://github.com/unitreerobotics/unitree_ros)
* [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk)

#### Probabilistic Contact Estimation
*  [ProbabilisticContactEstimation](https://github.com/MichaelMarav/ProbabilisticContactEstimation)

####  System 
*  Ubuntu 20.04
* ROS Noetic

### Installation
1. Get the above dependencies 
2. Clone the repo
   ```sh
   git clone https://github.com/despargy/maestro
   ```
2. Catkin make
   ```sh
   cd ~/catkin_ws 
   catkin_make
   ```

## Parameters

## Experiment setup


