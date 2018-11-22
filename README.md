# The DriveU Traffic Light Dataset (DTLD): Introduction and Comparison with Existing Datasets
This repository provides code for parsing the DriveU Traffic Light Dataset (DTLD), which is published in the course of our 2018 ICRA publication "The DriveU Traffic Light Dataset: Introduction and Comparison with Existing Datasets". 

## Paper
Paper see https://ieeexplore.ieee.org/document/8460737.
## Download the dataset
The data itself can be downloaded from http://www.traffic-light-data.com/. 

## Before starting: 
### Check our documentation
Documentation is stored at /dtld_parsing/doc/. We give insights into the data and explain how to interpret it.
### Change absolute paths
Do not forget to change the absolute paths of the images in all label files (.yml)

## Using the dataset 
### C++

1. Clone the dtld_parsing respository
```Shell
git clone https://github.com/julimueller/dtld_parsing
```
2. Build everything
```Shell
1. cd dtld_parsing/C++/driveu_dataset/
2. mkdir build && cd build
3. cmake .. -DCMAKE_INSTALL_PREFIX="YOUR_PATH" && make -j12 install
4. driveu_test -label_file <label_file_path.yml> -calib_path <path_to_calib>

```
Note: "YOUR_PATH" has to be in LD_LIBRARY_PATH.
The visualization should look like this

![alt text](https://github.com/julimueller/dtld_parsing/blob/master/images/c%2B%2B_demo.png)
### Python
```Shell
git clone https://github.com/julimueller/dtld_parsing
cd dtld_parsing/Python
python loadDataset.py
```
Result should look like above
### MATLAB
Run main.m

Results should look like this
![alt text](https://github.com/julimueller/dtld_parsing/blob/master/images/matlab_demo.png)

## Citation
Do not forget to cite our work for the case you used DTLD
### Citation:
```
@INPROCEEDINGS{8460737,
author={A. Fregin and J. Müller and U. Kreβel and K. Dietmayer},
booktitle={2018 IEEE International Conference on Robotics and Automation (ICRA)},
title={The DriveU Traffic Light Dataset: Introduction and Comparison with Existing Datasets},
year={2018},
volume={},
number={},
pages={3376-3383},
keywords={computer vision;image recognition;traffic engineering computing;DriveU traffic light dataset;traffic light recognition;autonomous driving;computer vision;University of Ulm Traffic Light Dataset;Daimler AG;Cameras;Urban areas;Benchmark testing;Lenses;Training;Visualization;Detectors},
doi={10.1109/ICRA.2018.8460737},
ISSN={2577-087X},
month={May},}

```
