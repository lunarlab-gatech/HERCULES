# HERCULES - Collaborative SLAM Experiments

This README contains instructions for replicating the Collaborative SLAM experiments found in the following paper:

> **Paper:** *HERCULES: An Open-Source Framework for Heterogeneous Multi-Robot SLAM, Collaborative Perception, and Exploration in Photorealistic Environments*
> Sandilya Sai Garimella, Daniel Chase Butterfield, Sean Wilson, and Lu Gan

---
## Dataset 


### Accessing the Data

*Coming soon.*

### Data Conversion

Each baseline has different expected formats for the input data. For this paper, we utilize [robotdataprocess](https://github.com/lunarlab-gatech/robotdataprocess/tree/HERCULES) in order to convert the raw outputs from HERCULES into the desired formats:

```
git clone https://github.com/lunarlab-gatech/robotdataprocess.git
cd robotdataprocess
git checkout HERCULES
```

Follow the instructions in the **Installation** section to setup the repository. Then, depending on the baseline, run the following instructions below to pre-process any data necessary. You may need to edit variables in the file for your specific setup, including paths and the HERCULES dataset number:

- **ROMAN**: Creation of .npy files and use of LIO-SAM odometry.
    - `python examples/Hercules/extract/extract_data_ROMAN.py`
    - `python examples/Hercules/reformat/reformat_LIO-SAM.py`
- **OpenVINS**: No pre-processing needed.
- **LIO-SAM**: Creation of ROS1 bags.
    - `python examples/Hercules/extract/extract_data_LIO-SAM.py`
    - `./examples/HERCULES/extract/ROS2_to_ROS1_LIO-SAM.sh`

### Result Evaluation

The [robotdataprocess](https://github.com/lunarlab-gatech/robotdataprocess/tree/HERCULES) repository is also used for evaluation, see [Data Conversion](#Data-Conversion) for instructions on installing it. After running the baseline below, use the corresponding command to get RMS ATE results. You will likely need to alter some paths and code parameters:

- **ROMAN**: `python examples/Hercules/results/results_ROMAN.py`
- **OpenVINS**: `python examples/Hercules/results/results_OpenVINS.py`
- **LIO-SAM**: `python examples/Hercules/results/results_LIO-SAM.py`


---
## Baselines

This section contains information on code locations for running each of the baselines. Note that these repositories often recieve updates, so make sure that you use the links provided in this README.md, which provides a tagged commit to the specific version used in the HERCULES paper.

For each baseline:

1. Follow the instructions in the **Installation** section of their README.
2. See [Data Conversion](#Data-Conversion) for any data pre-processing command you may need to run.
3. Run the experiment! The necessary commands are found in the baseline README under headers like **Run the package**  or **Experiments**. You may need to update some command line arguments or code parameters.
4. See [Result Evaluation](#result-evaluation) for generating the RMS ATE metrics.


### ROMAN

The code for running this experiment can be found here: TODO

```

```

### OpenVINS

The code for running this experiment can be found here: [OpenVINS - Tag "HERCULES"](https://github.com/lunarlab-gatech/open_vins/tree/HERCULES).

```
git clone git@github.com:lunarlab-gatech/open_vins.git
cd open_vins
git checkout HERCULES
```

### ORB-SLAM3 

*Coming soon.*

### LIO-SAM

The code for running this experiment can be found here: [LIO-SAM - Tag "HERCULES"](https://github.com/lunarlab-gatech/LIO-SAM/tree/HERCULES).

```
git clone git@github.com:lunarlab-gatech/LIO-SAM.git
cd LIO-SAM
git checkout HERCULES
```

---

## Contact

If you encounter issues building or running any of these experiments, feel free to open an GitHub Issue on this repository or on the corresponding baseline repository. You can also contact the authors.