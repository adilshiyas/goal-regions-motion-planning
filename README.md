# Motion Planning with Goal Regions

The following repo is a part of a larger project titled "Motion Planning in Semi-Static Environments with Goal Regions"  
This repo describes the structure of using goal regions for planning to objects with continuous pose variation.  

## Semi-Static Environments

A semi-static environment is described by the presence of static obstacles and semi-static obstacles where semi-static obstacles are those who's poses vary between queries but not during a given query.  

<!--
![A semi-static environment](images/semi_static1b.gif)
-->

<img src="images/semi_static1b.gif" alt="semi-static environment" style="width:70%; height:auto;" />

When object poses vary over a continuous distribution, a grasping query to one of these objects induces a motion planning problem. Thus, the object poses varying over a continuous distribution induce infinite motion planning problems.  
Our strategy to solving this problem is to develop an experience-based planner that precomputes and stores paths to completely cover all feasible motion planning problems which can later be queried within a fixed-time bound to solve a newly sampled problem.
