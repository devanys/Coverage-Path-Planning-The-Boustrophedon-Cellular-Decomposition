<img width="2256" height="1382" alt="Untitled design (16)" src="https://github.com/user-attachments/assets/d367614f-0772-4e68-bd88-ee0b812e099a" />


# Coverage Path Planning: The Boustrophedon Cellular Decomposition

Implements a Coverage Path Planning (CPP) simulation using the Boustrophedon Cellular Decomposition (BCD) algorithm. It allows a mobile robot to efficiently cover an entire workspace while avoiding obstacles.

## Coverage Path Planning

fundamental problem in mobile robotics that focuses on generating a path to ensure that a robot visits every point within a defined workspace while avoiding obstacles. Unlike traditional point-to-point navigation, the goal of CPP is to achieve complete area coverage, meaning that the robot’s sensor or tool footprint passes over all accessible regions of the environment at least once. This approach is widely applied in various domains such as cleaning robots, agricultural field coverage, inspection, and mapping. A well-designed CPP algorithm must balance completeness, efficiency, and redundancy minimization, ensuring that the robot covers all target areas with minimal overlap and travel distance.
<img width="440" height="259" alt="image" src="https://github.com/user-attachments/assets/be7c770b-9612-490c-9589-c3afa01cad72" />


## Boustrophedon Cellular Decomposition

A method used in Coverage Path Planning (CPP) for mobile robots to ensure complete and systematic coverage of a given workspace. The term boustrophedon derives from the Greek expression meaning “the way of the ox turns while plowing”, symbolizing a back-and-forth sweeping motion. In this approach, the environment is represented as an occupancy grid, and a sweep line is moved across the map to detect changes in free-space connectivity caused by obstacles. Each time a connectivity change is detected, a new cell is formed, resulting in a decomposition of the workspace into non-overlapping subregions that can be individually covered. 
<img width="850" height="609" alt="image" src="https://github.com/user-attachments/assets/1694a9c6-bf10-46c3-8efa-d44082b88138" />

## 🧩 Features

- 🖼️ Load **binary PNG maps** for coverage simulation.  
- 🧱 Automatic conversion to **occupancy grid** representation.  
- 🧭 **Boustrophedon Cellular Decomposition** to divide the workspace into simpler subregions.  
- 🧮 **Lawn-mower pattern generation** for intra-cell coverage.   
- ⚙️ Adjustable robot spacing (coverage line distance).  
- 🪟 Intuitive **Tkinter-based GUI** with real-time visualization.

## 🧩 Applications

- 🤖 **Autonomous cleaning robots** (vacuum, mopping)  
- 🌾 **Agricultural robots** for seeding or spraying  
- 🚨 **Search-and-rescue exploration** in complex environments  
- 🧱 **Surface inspection and mapping** for industrial or structural monitoring  


### References

- Howie Choset and Philippe Pignon, *"Coverage Path Planning: The Boustrophedon Cellular Decomposition,"* **Proceedings of the International Conference on Field and Service Robotics**, 1998.  
- Yongjin Kim and Howie Choset, *"Sensor-Based Exploration: Incremental Construction of the Hierarchical Generalized Voronoi Graph,"* **The International Journal of Robotics Research**, 2005. DOI: [10.1177/0278364905053863](https://doi.org/10.1177/0278364905053863)  
- Seoung-Ki Sul and Joonho Lee, *"Coverage Path Planning for Mobile Robots Using Decomposition Methods,"* **IEEE Transactions on Systems, Man, and Cybernetics**, 2011.  
- MathWorks YouTube Playlist — *"Autonomous Navigation"*, [https://youtube.com/playlist?list=PLn8PRpmsu08rLRGrnF-S6TyGrmcA2X7kg](https://youtube.com/playlist?list=PLn8PRpmsu08rLRGrnF-S6TyGrmcA2X7kg)  
- Richey Huang, *"Boustrophedon Cellular Decomposition Path Planning,"* GitHub Repository, [https://github.com/RicheyHuang/BoustrophedonCellularDecompositionPathPlanning](https://github.com/RicheyHuang/BoustrophedonCellularDecompositionPathPlanning)  
