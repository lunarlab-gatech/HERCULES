# HERCULES - Collaborative SLAM Experiments

This README contains instructions for replicating the Collaborative SLAM experiments found in the following paper:

> **Paper:** *HERCULES: An Open-Source Framework for Heterogeneous Multi-Robot SLAM, Collaborative Perception, and Exploration in Photorealistic Environments*
> Sandilya Sai Garimella, Daniel Chase Butterfield, Sean Wilson, and Lu Gan

## Dataset

Coming soon.

### Accessing the Data

### Data Conversion

Each baseline has different expected formats for the input data. For this paper, we utilize [robotdataprocess](https://github.com/lunarlab-gatech/robotdataprocess/tree/HERCULES) in order to convert the raw outputs from HERCULES into the desired formats. Follow the instructions in the "Installation" section to setup the repository. Then, depending on the baseline, run the following instructions below to pre-process any data necessary. You may need to edit variables in the file for your specific setup, including paths and the HERCULES dataset number:

- LIO-SAM
    - `python examples/Hercules/extract/extract_data_LIO-SAM.py` - 
    - `./examples/HERCULES/extract/ROS2_to_ROS1_LIO-SAM.sh`

## Baselines

This section contains information on code locations for running each of the baselines. Note that these repositories often recieve updates, so make sure that you use the links provided in this README.md, which provides a tagged commit to the specific version used in the HERCULES paper.

### ROMAN

Coming soon.

### OpenVINS

Coming soon.

### LIO-SAM

The code for running this experiment can be found [here](https://github.com/lunarlab-gatech/LIO-SAM/tree/HERCULES). Follow the instructions in the "Installation" section to setup the repository. Next, you'll need the HERCULES data in a ROS1 bag format; see [Data Conversion](#Data-Conversion). Finally, you can run the experiment by using the commands under "Run the package" in the LIO-SAM repository. You may need to update the commands for the output directory, the current robot, and the dataset number.

### ORB-SLAM3 

Coming soon.