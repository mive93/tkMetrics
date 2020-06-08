
#include <iostream>
#include <signal.h>
#include <stdlib.h>     /* srand, rand */
#include <unistd.h>
#include <mutex>
#include <map>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "meanAveragePrecision.h"
#include "tkMetricsUtils.h"
#include "readData.h"

void drawBoxes(const std::vector<tk::metrics::BoundingBox>& boxes, cv::Mat& frame, const int width, const int height, const bool groundtruth=false){
    cv::Scalar color;
    if(groundtruth) color = cv::Scalar(0, 255, 0);
    else            color = cv::Scalar(0, 0, 255);
    for(const auto& b:boxes)
        cv::rectangle(frame, cv::Point((b.x-b.w/2)*width, (b.y-b.h/2)*height), cv::Point((b.x+b.w/2)*width,(b.y+b.h/2)*height), color, 2);             
}

int main(int argc, char *argv[]) 
{
    std::string labels_path = "../data/BDD100K_val/all_images.txt";
    std::string det_folder = "../data/det/";
    std::string config_filename = "../data/config.yaml";
    bool show = false;

    if(argc > 1)
        labels_path = argv[1]; 
    if(argc > 2)
        det_folder = argv[2]; 
    if(argc > 3)
        config_filename = argv[3];
    if(argc > 4)
        show = atoi(argv[4]);

    bool write_res_on_file = true;
    int n_images = 5000;

    bool verbose;
    int classes, map_points, map_levels;
    float map_step, IoU_thresh, conf_thresh;

    //check if files needed exist
    if(!fileExist(config_filename.c_str()))
        FatalError("Wrong config file path.");
    if(!fileExist(labels_path.c_str()))
        FatalError("Wrong labels file path.");

    //read mAP parameters
    readmAPParams( config_filename, classes,  map_points, map_levels, map_step,IoU_thresh, conf_thresh, verbose);

    //read images 
    std::ifstream all_labels(labels_path);
    std::string i_filename;
    std::vector<tk::metrics::Frame> images;
    std::vector<tk::metrics::BoundingBox> det;

    std::cout<<"Reading groundtruth and detections"<<std::endl;

    if(show)
        cv::namedWindow("detection", cv::WINDOW_NORMAL);

    for (int images_done=0 ; std::getline(all_labels, i_filename) && images_done < n_images ; ++images_done) {
        std::cout << "Images done:\t" << images_done<< "\n";

        tk::metrics::Frame f;
        f.lFilename = i_filename;
        f.iFilename = i_filename;
        convertFilename(f.lFilename, "images", "labels", ".jpg", ".txt");

        // read frame
        if(!fileExist(f.iFilename.c_str()))
            FatalError("Wrong image file path.");
        cv::Mat frame = cv::imread(f.iFilename.c_str(), cv::IMREAD_COLOR);
        int width = frame.cols;
        int height = frame.rows;

        if(!frame.data) 
            break;
        
        // read and save detection labels
        std::string det_name = det_folder + f.lFilename.substr(f.lFilename.find("labels/") + 7);

        if(fileExist((det_name).c_str())){
            readYoloFormat(det_name, f.det, false);
            drawBoxes(f.det, frame, width, height, false);
        }

        // read and save groundtruth labels
        if(fileExist(f.lFilename.c_str())){
            readYoloFormat(f.lFilename, f.gt, true);
            drawBoxes(f.gt, frame, width, height, true);
        }    
      
        images.push_back(f);
        
        if(show){
            cv::imshow("detection",frame);
            cv::waitKey(0);
        }
    }

    //compute mAP
    double AP = computeMapNIoULevels(images,classes,IoU_thresh,conf_thresh, map_points, map_step, map_levels, verbose, write_res_on_file);
    std::cout<<"\nmAP "<<IoU_thresh<<":"<<IoU_thresh+map_step*(map_levels-1)<<" = "<<AP<<std::endl;

    //compute average precision, recall and f1score
    computeTPFPFN(images,classes,IoU_thresh,conf_thresh, verbose, write_res_on_file);
    return 0;
}

