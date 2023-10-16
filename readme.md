# Multi Agent Path Finder (MAPF)
![](/mapf_demo.mp4)

**Table of Contents**
- [Introduction to MAPF](#introduction-to-mapf)
- [Domain Knowledge](#domain-knowledge)
  - [CBS](#cbs)
  - [Improve CBS/CBS with Heuristics (CBSH)](#improve-cbscbs-with-heuristics-cbsh)
  - [MDD](#mdd)
  - [CG Heuristic (Cardinal Graph Heuristic)](#cg-heuristic-cardinal-graph-heuristic)
  - [DG Heuristic (Distance Graph Heuristic)](#dg-heuristic-distance-graph-heuristic)
  - [WDG Heuristic (Weighted Distance Graph Heuristic)](#wdg-heuristic-weighted-distance-graph-heuristic)
- [Installation and Setup Instructions](#installation-and-setup-instructions)
  - [Requirements](#requirements)
  - [First Time Installation](#first-time-installation)
  - [Run App Locally](#run-app-locally)
- [Research: Investigating the Impact of Increasing Agent Density on CBSH Performance](#research-investigating-the-impact-of-increasing-agent-density-on-cbsh-performance)

- [References](#references)
## Introduction to MAPF
Multi-Agent Path Finding (MAPF)  is an important topic in many fields, such as robotics, gaming, and AI. In this project we will implement and analyze different algorithms for the MAPF Problem. MAPF is defined as follows: given a set of agents and their respective start and goal locations, determine a set of minimum cost paths that lead the agents to their goals while avoiding collisions between agents and other obstacles.

## Domain Knowledge
### CBS
A commonly used algorithm for solving MAPF problems is the Conflict Based Search (CBS). CBS can be separated into two levels, the high level search and low level search. In the low level search, we will use A* search to find the shortest paths for each agent within a set of constraints. Note, that the low level search does not consider collisions between agents, this is the responsibility of the high level search. The high level search takes the paths returned from the low level search, checks for collisions between agents, and adds new constraints when pairs of paths results in a collision. The algorithm alternates between the low level search and high level search until we find a collision free solution.

### Improve CBS/CBS with Heuristics (CBSH)
To further improve CBS, we can add heuristics, an approximation of how close a given state is to a collision free solution, at the high level search. The heuristic helps us better determine what constraints to add that will likely lead to a collsion free solution sooner. In this project, we will take a look at three heuristics, namely the CG, DG, and WDG heuristics. The order of the heuristic from most informative to least informative is WDG, DG, CG. The main draw back of a more informative heuristic is that it takes more time and resource to calculate. Hence, we will also analyze and compare the performance of each heurisitc for different types of instances.

### MDD
Before delving into the explanation of the three heuristics, it's important to introduce the concept of Multi-Value Decision Diagrams (MDD). MDD serves as a graphical representation designed to encompass all valid paths that adhere to an agent's constraints. Notably, these paths are structured in such a way that ensures they remain free of cycles.

To efficiently identify conflicts or dependencies between pairs of agents, we leverage a comparative analysis of MDD graphs. These graphs are generated using a modified version of the A* algorithm. A* is a widely-used algorithm for finding the shortest path from an agent's starting location to their goal. In our modified A* version, we go beyond finding a single shortest path; we seek out all shortest paths and employ them to construct the MDD tree, which captures the various shortest paths for the agent.

### CG Heuristic (Cardinal Graph Heuristic):
The CG heuristic, or Cardinal Graph Heuristic, is a valuable approach in the realm of Multi-Agent Path Finding. It focuses on identifying and evaluating conflicts that are of cardinal importance. Cardinal conflicts are pivotal because resolving them is necessary to reach a collision-free solution with a cost similar to the current state. CG provides insights into the critical conflicts that, when resolved efficiently, pave the way for a cost-effective and safe multi-agent path. To find cardinal conflicts we can compare MDDs between agents. A cardinal conflict is represented as a single vertex where both agents are at the same location at the same time and there are no alternative paths for either agent to take, resulting in them being in different locations.

### DG Heuristic (Distance Graph Heuristic):
The DG heuristic, or Distance Graph Heuristic, is a heuristic approach that places a strong emphasis on understanding the spatial relationships between agents in Multi-Agent Path Finding scenarios. It aims to identify dependencies between agents, which signifies that the paths each agent chooses can potentially lead to collisions with one another. In other words, two agents are considered "dependent" if, in every pair of optimal paths they might take, there exists the potential for a collision between them.

In essence, the DG heuristic provides a valuable insight by ensuring that, as agents progress along their optimal paths, there will always be critical conflicts to resolve. This helps in guiding the planning process towards addressing these conflicts early, thereby contributing to a more efficient and safe multi-agent path planning strategy.

In practice, one effective way to identify these dependencies is to combine the MDDs of different agents. By examining the MDDs together, you can identify situations where the paths of two or more agents intersect or overlap, potentially leading to conflicts or dependencies.

### WDG Heuristic (Weighted Distance Graph Heuristic):
The WDG heuristic, or Weighted Distance Graph Heuristic, builds upon the DG heuristic by introducing weighted edges into the distance graph. These weights reflect the significance of specific agent-to-agent relationships in terms of collision avoidance. By assigning different weights to edges, the WDG heuristic acknowledges that not all agent interactions carry the same level of importance. This allows for a more nuanced evaluation of conflicts and the prioritization of certain paths or edges over others in the pursuit of efficient collision avoidance.

## Installation and Setup Instructions
### Requirements
- Python 3.x
- Jupyter Notebook
- Pandas, Numpy, and Matplotlib
- Visual Studio
### First Time Installation
Clone Repository

    git clone "https://github.com/Jtan99/Multi-Agent-Path-Finder"
### Run App locally
  1. Open folder in Visual Studio
  2. Update &lt;path to file> in launch.json
  3. In Visual Studio UI select run > run without debugging


## Research: Investigating the Impact of Increasing Agent Density on CBSH Performance
Our goal is to examine how the performance of the Conflict-Based Search with Heuristics (CBSH) algorithm degrades as the complexity of problem instances increases due to higher agent density. We aim to understand how the agent density, calculated as the ratio of the number of agents to the number of open locations in the instance, influences the efficiency and effectiveness of CBSH. By analyzing CBSH's performance across different levels of agent density, we seek to determine whether increasing density through the addition of more agents or obstacles affects the algorithm differently.

"For detailed information on our research plan, case study, and results, please refer to the  [MAPF_report.pdf](https://github.com/Jtan99/Multi-Agent-Path-Finder/blob/master/MAPF_report.pdf) starting from Section 3.

## References
- Li, J., Felner, A., Boyarski, E., Ma, H., & Koenig, S. (2019). Improved Heuristics for Multi-Agent Path Finding with Conflict-Based Search. Proceedings of the Twenty-Eighth International Joint Conference on Artificial Intelligence. doi:10.24963/ijcai.2019/63

- Vertex cover problem: Set 1 (introduction and approximate algorithm). GeeksforGeeks. (2021, August 18). Retrieved December 21, 2021, from https://www.geeksforgeeks.org/vertex-cover-problem-set-1-introduction-approximate-algorithm-2/ 

- Python tutorial. Python Tutorial: Graph Data Structure - 2021. (n.d.). Retrieved December 21, 2021, from https://www.bogotobogo.com/python/python_graph_data_structures.php 