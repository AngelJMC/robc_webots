# Heuristic optimization
This repository contains the implementation of two metaheuristic algorithms developed to optimize an autonomous driving algorithm with the aim of reducing the travel time of a competition circuit.

One of the heuristic algorithms is based on local search engines and taboo lists for optimization. The other heuristics implements a standard genetic algorithm.

Both algorithms are developed in **C** and run under the **Webots** simulation environment. Webots provides the simulation engine of the vehicle, the circuit and all the physics.

## Installation

 - [Webots](https://cyberbotics.com/) -   Open Source Robot Simulator 
 - [GSL](https://www.gnu.org/software/gsl/) - GNU Scientific Library
	```
	sudo apt-get install libgsl-dev 
	```

## Usage

Open and run an instance of the simulator, with the **search-based optimization** algorithm:
```
	webots --mode=pause robc_webots/worlds/circuit.wbt
```

Open and run an instance of the simulator, with the **genetic optimization** algorithm
```
	webots --mode=pause robc_webots/worlds/circuit_ga.wbt
```
