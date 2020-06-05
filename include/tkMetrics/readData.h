#ifndef READDATA_H
#define READDATA_H

#include "BoundingBox.h"
#include <vector>
#include <fstream>
#include <string>
#include <sstream>

std::vector<metrics::BoundingBox> readMOTFormat(const std::string& gt_file, const char del=',', const bool groundtruth=false){

    std::string line;
    std::ifstream gt(gt_file);

    std::vector<metrics::BoundingBox> data;
    while(getline(gt,line)){
        std::stringstream linestream(line);
        std::string value;
        std::vector<float> values;

        while(getline(linestream,value, del)){        
            values.push_back(std::stof(value));
        }

        metrics::BoundingBox b;
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

#endif /* READDATA_H */