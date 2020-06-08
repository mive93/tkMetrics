#ifndef READDATA_H
#define READDATA_H

#include "BoundingBox.h"
#include <vector>
#include <fstream>
#include <string>
#include <sstream>

#include <yaml-cpp/yaml.h>

std::vector<tk::metrics::BoundingBox> readMOTFormat(const std::string& gt_file, const char del=',', const bool groundtruth=false){

    std::string line;
    std::ifstream gt(gt_file);

    std::vector<tk::metrics::BoundingBox> data;
    while(getline(gt,line)){
        std::stringstream linestream(line);
        std::string value;
        std::vector<float> values;

        while(getline(linestream,value, del)){        
            values.push_back(std::stof(value));
        }

        tk::metrics::BoundingBox b;
        b.frameId   = values[0];
        b.trackId   = values[1];
        b.x         = values[2];
        b.y         = values[3];
        b.w         = values[4];
        b.h         = values[5];
        b.prob      = values[6];
        b.cl        = values[7];

        if((groundtruth && b.prob > 0) || !groundtruth)
            data.push_back(b);
    } 

    // std::cout<<data.size()<<" data read"<<std::endl;
    // for(int i=0; i<20; ++i)
    //     std::cout<<data[i]<<std::endl;
    return data;
}

void readYoloFormat(const std::string& filename, std::vector<tk::metrics::BoundingBox> & boxes, const bool groundtruth=false){
    std::ifstream labels(filename);
    for(std::string line; std::getline(labels, line); ){
        std::istringstream in(line); 
        tk::metrics::BoundingBox b;
        in >> b.cl; 
        if(groundtruth){
            b.prob = 1;
            b.truthFlag = 1;
        }
        else
            in >> b.prob;
        in >> b.x >> b.y >> b.w >> b.h;  
        boxes.push_back(b);
    }
}

void readmAPParams( std::string config_filename, int& classes, int& map_points, 
                    int& map_levels, float& map_step, float& IoU_thresh, 
                    float& conf_thresh, bool& verbose){
    YAML::Node config   = YAML::LoadFile(config_filename);
    classes     = config["classes"].as<int>();
    map_points  = config["map_points"].as<int>();
    map_levels  = config["map_levels"].as<int>();
    map_step    = config["map_step"].as<float>();
    IoU_thresh  = config["IoU_thresh"].as<float>();
    conf_thresh = config["conf_thresh"].as<float>();
    verbose     = config["verbose"].as<bool>();
}

#endif /* READDATA_H */