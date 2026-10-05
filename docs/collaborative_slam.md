# Collaborative SLAM (ROMAN)

To demonstrate HERCULES as a benchmark sandbox, we provide a ready-to-run **collaborative SLAM** evaluation using [ROMAN](https://github.com/mit-acl/ROMAN) (MIT-ACL) on a *City Block* sequence with **two UAVs and two UGVs**.

Each robot runs **LIO-SAM** odometry with an open-set **ROMAN object map**; inter-robot loop closures are registered pairwise via [CLIPPER](https://github.com/mit-acl/clipper).

## Live mapping

Per-robot live trajectory, camera pose, and object map:

<div class="grid" markdown>
![UAV 1](media/V2.4.C-Drone1-LiveMapping.gif)
![UAV 2](media/V2.4.C-Drone2-LiveMapping.gif)
![UGV 1](media/V2.4.C-Husky1-LiveMapping.gif)
![UGV 2](media/V2.4.C-Husky2-LiveMapping.gif)
</div>

## Final maps

Per-robot LIO-SAM final map with the ROMAN object map:

<div class="grid" markdown>
![UAV 1 final map](media/V2.4.C-Drone1-FinalMapViz.gif)
![UGV 1 final map](media/V2.4.C-Husky1-FinalMapViz.gif)
</div>

## Loop-closure alignment

ROMAN loop closure aligning UGV 1 and UGV 2 submaps pairwise via CLIPPER:

![Loop-closure alignment](media/V2.4.C-Husky1Husky2-Alignment.gif)

## Running it

The evaluation runs on data exported by the HERCULES [dataset pipeline](dataset.md) and consumed through the optimized [ROS 2 wrapper](ros_cplusplus.md). See the [dataset](dataset.md) page for the sequence formats (ROS 2 bags / KITTI-style layouts). Instructions for replicating the paper's collaborative SLAM experiments are in [paper/CollaborativeSLAM.md](https://github.com/lunarlab-gatech/HERCULES/blob/main/paper/CollaborativeSLAM.md).
