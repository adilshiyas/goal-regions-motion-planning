# Motion Planning with Goal Regions

This repository documents the system design and structure of a larger research project titled  
**“Motion Planning in Semi-Static Environments with Goal Regions.”**  
The focus here is on representing continuous workspace goals using goal regions and leveraging this representation for experience-based motion planning.

## Semi-Static Environments

A semi-static environment consists of static obstacles and *semi-static* objects, where semi-static objects may change pose between queries but remain fixed during the execution of any single query.

<!--
![A semi-static environment](images/semi_static1b.gif)


<img src="images/semi_static1b.gif" alt="semi-static environment" style="width:70%; height:auto;" />

<figure>
  <img src="images/semi_static1b.gif" alt="semi-static environment" style="width:70%">
  <figcaption><strong>Fig 1:</strong> A semi-static environment.  </figcaption>
</figure>

-->

<p align="center">
  <img src="images/semi_static1b.gif" alt="semi-static environment" style="width:70%; height:auto;">
  <br>
  <strong>Fig. 1:</strong> A semi-static environment
</p>
  
When object poses vary over a continuous distribution, each grasping query induces a distinct motion planning problem, resulting in an effectively unbounded family of planning instances.  
To address this, we adopt an experience-based planning strategy that precomputes and stores representative solution paths covering the feasible goal space. At query time, previously computed paths are retrieved and reused, enabling queries within a fixed-time bound.

## Goal Regions

Building on Task Space Regions (TSRs) from [Berenson 2011](https://www.ri.cmu.edu/publications/task-space-regions-a-framework-for-pose-constrained-manipulation-planning/), we represent continuous workspace goals with a set of TSR intersections.  
By representing subsets of the continuous object pose distribution using a finite collection of TSR intersections, we can sample representative workspace goals from each region and store corresponding solution paths to provide coverage over the object’s pose distribution.

## Queries

Using the above strategy, we create and load a data structure of precomputed solution paths. A query is defined as the retrieval of a solution path from this data structure. This design enables constant-time queries at the cost of a large memory footprint.

<p align="center">
  <img src="images/brute.gif" alt="Querying solutions from data structure" style="width:85%; height:auto;">
  <br>
  <strong>Fig. 2:</strong> Querying solutions from the data structure
</p>

This design reflects an explicit time–memory tradeoff: the system prioritizes predictable, constant-time queries by precomputing and storing a large number of solution paths. Ongoing work explores alternative representations that reduce memory usage while preserving bounded query-time performance.

## Algorithm

This section summarizes the current pipeline in algorithmic form. The system consists of an offline preprocessing stage that constructs the experience data structure, and an online query stage that retrieves a precomputed solution path. In the current implementation, a *query* is defined as constant-time retrieval of a precomputed solution path from the loaded data structure.

### Offline Precomputation (Experience Library Construction)

The current implementation assumes that the only semi-static object is the one being queried for a grasp. Further developments will include semi-static obstacles.

**Inputs:**  
- Static obstacle poses
- Pose distribution N for semi-static object
- Home pose H 
- TSR B for a grasp (constructed based on end-effector and object geometry)  

**Output:**  
- Experience data structure mapping bins of the object's possible poses to solution paths

```text
Algorithm 1: PREPROCESSING
1: Initialize empty data structure D
2: Place object at an extreme pose Pmin ∈ N giving rise to TSR at Pmin, B_min
3: Initialize covered object poses C = ∅ 
4: while C is not equal to N:
5:   Perturb object pose from Pmin to Pnew ∈ N to give TSR, B_new such that B_min ∩ B_new ≠ ∅   
6:   Sample workspace goal W from B_min ∩ B_new
7:   Find IK solution to W as Q
8:   Plan path p_i from H to Q, store p_i in D
9:   C = C ∪ convexhull(Pmin, Pnew)
10:  Pmin = Pnew
```



11: Serialize H to disk
12: return H
