# The DriveU Traffic Light Dataset (DTLD): Introduction and Comparison with Existing Datasets
This repository provides code for parsing the DriveU Traffic Light Dataset (DTLD), which is published in the course of our 2018 ICRA publication "The DriveU Traffic Light Dataset: Introduction and Comparison with Existing Datasets". 

## Paper
Paper see https://ieeexplore.ieee.org/document/8460737.
## Download the dataset
The data itself can be downloaded from http://www.traffic-light-data.com/. 

    .
    ├── DTLD                 # DTLD
        ├── Berlin           # Contains all Routes of Berlin
        ├── Bochum           # Contains all routes of Bochum
        ├── Bremen           # Contains all routes of Bremen
        ├── Dortmund         # Contains all routes of Dortmund
        ├── Duesseldorf      # Contains all routes of Duesseldorf
        ├── Essen            # Contains all routes of Essen
        ├── Frankfurt        # Contains all routes of Frankfurt
        ├── Fulda            # Contains all routes of Fulda
        ├── Hannover         # Contains all routes of Hannover
        ├── Kassel           # Contains all routes of Kassel
        ├── Koeln            # Contains all routes of Cologne
        ├── Berlin_all.yml   # Label file for Berlin
        ├── ...
        ├── Koeln_all.yml    # Label file for Cologne
        ├── DTLD_all.yml     # Complete label file
        ├── DTLD_train.yml   # Training file
        ├── DTLD_test.yml    # Testing file
        ├── LICENSE          # License
        └── README.md        # Readme

### Route structure
We separated each drive in one city into different routes

    .
    ├── Berlin                # Berlin
        ├── Berlin1           # First route
        ├── Berlin2           # Second route
        ├── Berlin3           # Third route
        ├── ...
### Sequence structure
We separated each route into several sequences. One sequence describes one unique intersection up to passing it. The foldername indicates date and time.

    .
    ├── Berlin 1                    # Route Berlin1
        ├── 2015-04-17_10-50-05     # First intersection
        ├── 2015-04-17_10-50-41     # Second intersection
        ├── ...
        
### Image structure
For each sequences, images and disparity images are available. Filename indicates time and date

    .
    ├── 2015-04-17_10-50-05                                      # Route Berlin1
        ├── DE_BBBR667_2015-04-17_10-50-13-633939_k0.tiff        # First left camera image
        ├── DE_BBBR667_2015-04-17_10-50-13-633939_nativeV2.tiff  # First disparity image
        ├── DE_BBBR667_2015-04-17_10-50-14-299876_k0.tiff        # Second left camera image
        ├── DE_BBBR667_2015-04-17_10-50-14-299876_nativeV2       # Second disparity image
        ├── ...
## Before starting
#### 1. Check our documentation
Documentation is stored at /dtld_parsing/doc/. We give insights into the data and explain how to interpret it.
#### 2. Change absolute paths
Do not forget to change the absolute paths of the images in all label files (.yml). 

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
4. driveu_test -label_file <label_file_path.yml> -calib_path <path_to_calib> -data_base_dir <dtld_dir> 

```
Note: "YOUR_PATH" has to be in LD_LIBRARY_PATH. DTLD_DIR is the directory where all .zips should be unpacked.
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
