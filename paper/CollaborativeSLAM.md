# HERCULES - Collaborative SLAM Experiments

This README contains instructions for replicating the Collaborative SLAM experiments found in the following paper:

> **Paper:** *HERCULES: An Open-Source Framework for Heterogeneous Multi-Robot SLAM, Collaborative Perception, and Exploration in Photorealistic Environments*
> Sandilya Sai Garimella, Daniel Chase Butterfield, Sean Wilson, and Lu Gan

---
## Dataset 


### Accessing the Data

Coming soon.

### Data Conversion

Each baseline has different expected formats for the input data. For this paper, we utilize [robotdataprocess](https://github.com/lunarlab-gatech/robotdataprocess/tree/HERCULES) in order to convert the raw outputs from HERCULES into the desired formats.

Follow the instructions in the **Installation** section to setup the repository. Then, depending on the baseline, run the following instructions below to pre-process any data necessary. You may need to edit variables in the file for your specific setup, including paths and the HERCULES dataset number:

- **OpenVINS**: No pre-processing needed.
- **LIO-SAM**: Creation of ROS1 bags.
    - `python examples/Hercules/extract/extract_data_LIO-SAM.py`
    - `./examples/HERCULES/extract/ROS2_to_ROS1_LIO-SAM.sh`

---
## Baselines

This section contains information on code locations for running each of the baselines. Note that these repositories often recieve updates, so make sure that you use the links provided in this README.md, which provides a tagged commit to the specific version used in the HERCULES paper.

For each baseline:

1. Follow the instructions in the **Installation** section of their README.
2. See [Data Conversion](#Data-Conversion) for any data pre-processing command you may need to run.
3. Run the experiment! The necessary commands are found in the baseline README under headers like **Run the package**  or **Experiments**. You may need to update some command line arguments or code parameters.


### ROMAN

Coming soon.

### OpenVINS

The code for running this experiment can be found here: [OpenVINS - Tag "HERCULES"](https://github.com/lunarlab-gatech/open_vins/tree/HERCULES).

### ORB-SLAM3 

Coming soon.

### LIO-SAM

The code for running this experiment can be found here: [LIO-SAM - Tag "HERCULES"](https://github.com/lunarlab-gatech/LIO-SAM/tree/HERCULES).