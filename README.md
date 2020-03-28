# point_cloud_registration
Point cloud registration using PCL and ceres libraries.

## Dependencies
- PCL 1.9
- Ceres
- Eigen3
- gflags
- OpenCV

## Build and run
### Build
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

### Run
*Using TUM dataset*

Need dataset directory path that includes `depth` folder and `groundtruth.txt`.

```
$ ./load_from_tum --path-to-dataset /example/rgbd_dataset_freiburg2_pioneer_slam2
```


*Using ETH dataset*

Need `Hokuyo_**.csv` as point cloud data in base coordinate and `pose_scanner_leica.csv` as pose ground truth.

```
$ ./load_from_ethdataset --path_to_ground_truth_csv /path/to/pose_scanner_leica.csv --base_path_to_csvs /path/to/csvs_dir
```


## Supported datasets
### TUM dataset
https://vision.in.tum.de/data/datasets/rgbd-dataset/download

### ETH dataset
https://projects.asl.ethz.ch/datasets/

## References
Mainly refer mv-lm-icp, and reimplement the logic using PCL library (the original doesn't use PCL):
https://github.com/adrelino/mv-lm-icp
