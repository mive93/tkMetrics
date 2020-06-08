# Metrics

tkMetrics is a library to compute the most common metrics for vision tasks (e.g. mAP, MOTA,...).

## Dependencies
  * OpenCV 3.4
  * yaml-cpp 0.5.2 (sudo apt install libyaml-cpp-dev)

## How to build this repo

```
git clone https://git.hipert.unimore.it/tk/core/tkMetrics.git
cd tkMetrics
mkdir build
cd build
cmake ..
make
```

---


## Tracking demo

This demo will evaluate the performance of your tracking method in terms of accuracy. 
It computes: IDF1, IDP, IDR, Rcll, Prcn, FAR, GT, MT, PT, ML, FP, FN, IDs, FM, MOTA, MOTP, MOTAL.

```
 ./tkMetrics_tracking <gt-folder> <det-folder> <IoU-threshold> <det-delimiter>
``` 
where: 
  * ```<gt-folder>``` is a folder contaning a subfolder for each video sequence, named as the video, containing a subfolder gt contaning a gt.txt
    For example, with only one sequence seq1, it would be: 
    ```
    gt_folder
     |
     ---seq1
         |
         ---gt
            | 
            --gt.txt
    ```
  * ```<det-folder>``` is a folder containing a txt file for each video sequence, named as the video.
  * ```<IoU-threshold> ``` is a value between 0 and 1.
  * ```<det-delimiter>``` is a characted, a delimiter used to separate values in detection files.

For example:
```
 ./tkMetrics_tracking ../data/gt/ ../data/dets/ 0.5
``` 

### Tracking format
A txt file is required, where, in each line there is a bounding box with the following format: 
```
<frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <class>, <visibility>

```
```
<frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>,

```
Both the format are currently accepted because x,y,z and class and visibilty are currently not used. 

---

## mAP demo

This demo will evaluate the performance of your object detection method in terms of accuracy. 
It computes mAP, avg precision, avg recall and f1-score.

A validation set is needed. 
To download COCO_val2017 (80 classes) run (form the root folder): 
```
bash scripts/download_validation.sh COCO
```
To download Berkeley_val (10 classes) run (form the root folder): 
```
bash scripts/download_validation.sh BDD
```

Then to compute the metrics run:
```
./tkMetrics_mAP <images-path-file> <det-folder> <config-file> <show-flag>
```
where:
* ```<images-path-file>```: path to a text file containing all the paths of the ground-truth images. It is important that all the images of the ground-truth are in a folder called 'images'. In the folder containing the folder 'images' there should be also a folder 'labels', containing all the ground-truth labels having the same name as the images. To better understand, if there is an image path/to/images/000001.jpg there should be a corresponding label path/to/labels/000001.txt. 
* ```<det-folder>```: path to a folder containing all the detection produced by your object detector. There should be a txt file for each image, with the same image name. The format needed is the yolo format.
* ```<config-file>```: path to a yaml file with the parameters needed for the mAP computation, similar to data/config.yaml
* ```<show-flag>```: flag that shows the detection is set to 1, nothing is shown otherwise.

For example
```
./tkMetrics_mAP ../data/BDD100K_val/all_images.txt ../data/det/ ../data/config.yaml 1
```



### Detection format (YOLO)
A txt file is required, where, in each line there is a bounding box with the following format for the groundtruth: 
```
<class-id> <x_center/image_width> <y_center/image_height> <bb_width/img_width> <bb_height/img_height>
```
for the detections
```
<class-id> <confidence> <x_center/image_width> <y_center/image_height> <bb_width/img_width> <bb_height/img_height>
```

## Credits

For the tracking metrics the code from MOT Challenge dev-kit has been adapted (https://motchallenge.net/devkit/). The credits goes to Anton Milan (antmila@amazon.com) and Ergys Ristani (ristani@cs.duke.edu).