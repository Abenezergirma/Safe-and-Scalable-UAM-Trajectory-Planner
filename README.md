# Safe and Scalable UAM Trajectory Planner  

Urban Air Mobility (UAM) is a novel concept in which partially or fully autonomous air vehicles transport passengers and cargo in dense urban environments. UAM operation is a multi-agent safety-critical application, where safety and scalability are the primary design considerations. Therefore, a UAM trajectory planning framework needs to generate trajectories efficiently while guaranteeing that the generated trajectories satisfy the system’s safety requirements. This project implements a multi-agent trajectory planner that enables unmanned aerial vehicles (UAVs) to navigate to the assigned destinations while avoiding collision with other agents in the environment.

## Usage
An example setup of running an entire simulation is given in [UAMTrajectoryPlannerExample.mlx](https://insidelabs-git.mathworks.com/marto/safe-and-scalable-uam-trajectory-planner/-/tree/master) file. The implemented trajectory planner is equipped with two main modules: Reachability Analysis and Markov Decision Process (MDP) based decision-making framework. The following diagram explanins how the framework operates.    

![](data/Reachability%20Analysis%20related%20experiments/Framework.png)

## Simulations
## 10 Agents - rendering single trajectory  
### Multi-view

![](data/Animations%20and%20Plots/10%20agents/Exp%202%20-3D%20-%20single%20render.gif)

### Top-view

![](data/Animations%20and%20Plots/10%20agents/Exp%202%20-%20single%20render%20-%20Top.gif)

### Side-view 

![](data/Animations%20and%20Plots/10%20agents/Exp%202%20-%20single%20render%20-%20Side.gif)

### State constraints

![](data/Animations%20and%20Plots/stateConstraint.gif)

## 10 Agents - rendering multiple trajectories 
### Multi-view

![](data/Animations%20and%20Plots/10%20agents/3D.gif)

### Top-view

![](data/Animations%20and%20Plots/10%20agents/Top.gif)

### Side-view 

![](data/Animations%20and%20Plots/10%20agents/animation.gif)

## Contact
If there are any issues with the library, please leave a comment on GitLab or feel free to send me an email at abengirma1@gmail.com 


