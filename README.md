<h1 align="center">Robust Communication-Aware Jamming Detection and Avoidance for Distributed Multi-Agent Systems</h1>

<h6 align="center"><small>MATLAB implementations of Formation Control Algorithm.</small></h6>

<p align="center"><b>#multi-agent systems &emsp; #communication-aware  &emsp; #formation control <br/> #particle swarm optimization  &emsp; #path planning  &emsp; #jamming area</b></p>

Swarm robotics has emerged as an innovative field, enabling groups of agents to collectively tackle complex challenges. In this pursuit, Multi-Agent Systems (MAS) offer unparalleled potential for navigating through intricate environments. In this paper, an innovative control strategy is proposed to enable a swarm of agents to navigate through a complex environment, which includes regions affected by electromagnetic jamming. The swarm is unaware of the existence or location of these regions, known as jamming areas. The novel control strategy is proposed as a combination of a gradient and movement controller. The gradient controller achieves the desired swarm formation that maximizes the communication quality between agents. The movement controller uses Particle Swarm Optimization (PSO) and a path planning algorithm to move towards a destination while avoiding jamming areas. Various path planning algorithms, such as A*, Greedy Best First Search, Breadth First Search, Dijkstra, Jump Point Search, and Theta*, are compared in simulations. The results highlight the discovery of an optimal path planning algorithm that facilitates efficient and robust navigation for the MAS, while also upholding high communication quality.

<div align="center">

<table>
  <tr>
    <th>Paper</th>
    <th>Presentation</th>
  </tr>
  <tr>
    <td align="center">
          <a href="https://github.com/sjpeccoud/Swarm-Control/tree/main/lib/Preprint.pdf"><img src="/lib/README.assets/Preprint.png" /></a>
          <a href="https://github.com/sjpeccoud/Swarm-Control/tree/main/lib/Preprint.pdf"><img src="https://img.shields.io/badge/View%20More-282c34?style=for-the-badge&logoColor=white" width="100" /></a>
    </td>
    <td align="center">
          <a href="https://github.com/sjpeccoud/Swarm-Control/tree/main/lib/Presentation.pdf"><img src="/lib/README.assets/Presentation.png" /></a>
          <a href="https://github.com/sjpeccoud/Swarm-Control/tree/main/lib/lib/Presentation.pdf"><img src="https://img.shields.io/badge/View%20More-282c34?style=for-the-badge&logoColor=white" width="100" /></a>
    </td>
  </tr>
</table>


<h3>Deadlock Problem</h3>
<h6>Using Communication-aware Formation Controller + PSO in a complex jamming environment</h6>
<table>
  <tr align="center">
    <td><img src="/lib/README.assets/Simulation.fig/PSO_deadlock.png" width=500 /></td>
    <td><img src="/lib/README.assets/Simulation.fig/PSO_deadlock_trace.png" width=500 /></td>
  </tr>
  <tr align="center">
    <td>t=0s</td>
    <td>t=4s</td>
  </tr>
</table>

<h3>Deadlock Resolution</h3>
<h6>Use Communication-aware Formation Controller + PSO + Path Planning Algorithm</h6>
<img src="/lib/README.assets/Simulation.fig/Path_Plan.png" width=500 />

<h3>Resolution Workflow</h3>
<img src="/lib/README.assets/Simulation.fig/Control_Diagram.png" width=500 />

<h3>Final Simulation</h3>
<table>
  <tr align="center">
    <td><img src="/lib/README.assets/Simulation.fig/Sim1.png" width=500 /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Sim2.png" width=500 /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Sim3.png" width=500 /></td>
  </tr>
  <tr align="center">
    <td>No Jam Points</td>
    <td>1 Jam Point</td>
    <td>2 Jam Points</td>
  </tr>
  <tr align="center">
    <td><img src="/lib/README.assets/Simulation.fig/Sim4.png" width=500 /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Sim5.png" width=500 /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Sim6.png" width=500 /></td>
  </tr>
  <tr align="center">
    <td>After 2 Jam Points</td>
    <td>4 Jam Points</td>
    <td>8 Jam Points</td>
  </tr>
</table>

<table>
  <tr align="center">
    <td><img src="/lib/README.assets/Simulation.fig/Sim_Grid.png" width=500 /></td>
    <td><img src="/lib/README.assets/Simulation.fig/Sim_Trace.png" width=500 /></td>
  </tr>
  <tr align="center">
    <td>Grid Map</td>
    <td>Trajectory</td>
  </tr>
</table>

<h2>Simulation Evaluations</h2>
<table>
  <tr align="center">
    <td><img src="/lib/README.assets/Simulation.fig/algoEval_aStar.png" width=500 /></td>
    <td><img src="/lib/README.assets/Simulation.fig/algoEval_BFS.png" width=500 /></td>
    <td><img src="/lib/README.assets/Simulation.fig/algoEval_Dijkstra.png" width=500 /></td>
  </tr>
  <tr align="center">
    <td>A*</td>
    <td>Breadth First Search</td>
    <td>Dijkstra</td>
  </tr>
  <tr align="center">
    <td><img src="/lib/README.assets/Simulation.fig/algoEval_GBFS.png" width=500 /></td>
    <td><img src="/lib/README.assets/Simulation.fig/algoEval_JPS.png" width=500 /></td>
    <td><img src="/lib/README.assets/Simulation.fig/algoEval_thetaStar.png" width=500 /></td>
  </tr>
  <tr align="center">
    <td>Greedy Best First Search</td>
    <td>Jump Point Search</td>
    <td>Theta*</td>
  </tr>
</table>

<!-- <h2 align="center">BibTeX Citation</h2>

```
@John Doe{John Doe,
  author={John Doe},
  booktitle={John Doe}, 
  title={John Doe}, 
  year={2023},
  volume={},
  number={},
  pages={},
  doi={}}
``` -->
