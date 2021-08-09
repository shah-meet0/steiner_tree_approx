# Approximation Algorithm for the Steiner Tree Problem.
## Information:
This repository contains java source code to approximately solve the [Steiner Tree problem](https://en.wikipedia.org/wiki/Steiner_tree_problem). 
This was a project for the course Allocations and Algorithms at Maastricht University.
Co-Author: Max van den Broek.
All instances were obtained from [Pace 2018](https://github.com/PACE-challenge/SteinerTree-PACE-2018-instances).
The primary algorithm used was introduced by [Kurt Mehlhorn](https://people.mpi-inf.mpg.de/~mehlhorn/ftp/SteinerTrees.pdf).

## Table of Contents:
1. [Instances](./Instances): Contains odd-numbered PACE 2018 Instances for the Steiner Tree Problem. (all files are .gr)
2. [solvedInstances](./solvedInstances): Contains appoximation solutions to the problems in Instances. (all files are .sol)
3. [approxSteinerTree](./approxSteinertree): Contains source files for various java classes.
    -Node.java: A class which represents a node on an undirected graph.
    -Edge.java: A class to denote the edge of a graph
    -Graph.java: A class to make undirected graph objects containing edges and nodes.
    -MehlhornAlgorithm.java: A class to perform the Mehlhorn algorithm on an arbitrary graph instance.
    -Main.java: Contains the execution code. Has code for both one particular instance and all instances.
4. [Report](./Report.pdf): Report which details implementation, approximation guarantee, runtime, etc. 
    
