# MuJoco_Simulation
MuJoCo is a high-performance physics engine for physical simulation.

You can try it out at the link below.

[MuJoCo — Advanced Physics Simulation](https://mujoco.org/)

## Simulation Result
![mujoco-2024-05-02_15 20 05-ezgif com-video-to-gif-converter](https://github.com/jebeom/Mujoco_Simulation/assets/107978090/f33187cd-5dea-4cc1-8585-4aae82974656)

>[!Note]
>This code uses a manual method, generating the necessary control commands in **controller.cpp** based on the simple trajectory calculations created in **trajectory.cpp**, rather than using high-level controllers like MPC or MPPI.
>* **controller.cpp** : Generate control commands for the robot arm. (Franka Research 3)
>* **main.cpp**       : Execute MuJoCo simulation and control robot arm. 
>* **quadraticprogram.cpp** : Provides functionality for defining and solving quadratic programming problems.
>* **robotmodel.cpp** : Load the robot model (URDF file) required for the simulation.
>* **simulate.cpp** : Configure the MuJoCo GUI window.
>* **trajectory.cpp**    : Using cubic spline interpolation for trajectory calculation, it computes the position and velocity at specific times to generate smooth trajectories.

<br>

## Quick Start
First, you need to delete the build files
```
cd /Mujoco_Simulation/
rm -r build
```
then
```
mkdir build
cd build
cmake ..
make  
./simulation
```
