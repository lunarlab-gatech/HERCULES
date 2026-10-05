# Multimodal Dataset

HERCULES ships a **dataset-collection pipeline** that exports **time-synchronized multimodal data** for heterogeneous robot teams. For each robot it captures **RGB**, **depth**, **semantic segmentation**, and **LiDAR** along a coverage or leader–follower trajectory, in standard formats: **KITTI-style layouts** and **ROS 2 bags**.

<div class="grid" markdown>
![City Block sequence](media/City_Block.gif)
![Forest sequence](media/Forest.gif)
![Australia — center coverage](media/Australia_Center.gif)
![Australia — perimeter coverage](media/Australia_Perimeter.gif)
</div>

## Download

The sequences across the desert, forest, and city worlds have their home on Hugging Face; uploading the full sequences is still on the project [roadmap](https://github.com/lunarlab-gatech/HERCULES#roadmap):

[:material-database: huggingface.co/datasets/GeorgiaTech/HERCULES](https://huggingface.co/datasets/GeorgiaTech/HERCULES){ .md-button .md-button--primary }

## Collecting your own

Trajectories are generated offline and executed with the unified [heterogeneous autonomy stack](heterogeneous_autonomy.md), with complementary **coverage** and **leader–follower** patterns. Sensor streams are configured through the [settings file](settings.md) and captured via the [Image APIs](image_apis.md), [LIDAR](lidar.md), and the optimized [ROS 2 wrapper](ros_cplusplus.md).

The same exported sequences drive the [collaborative SLAM](collaborative_slam.md) and [cooperative perception](cooperative_perception.md) benchmarks.
