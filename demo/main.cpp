#include <iostream>
#include <set>
#include "readData.h"
#include "trackingMetrics.h"

#include <sys/types.h>
#include <dirent.h>


void readDirectory(const std::string& name, std::vector<std::string>& v)
{
    DIR* dirp = opendir(name.c_str());
    struct dirent * dp;
    while ((dp = readdir(dirp)) != NULL) {
        std::string filename = dp->d_name;
        if(filename != "." && filename != "..")
            v.push_back(filename);
    }
    closedir(dirp);
}

int main(int argc, char *argv[]) {
    bool world = false;
    float threshold = 0.5;

    std::string det_folder = "../data/dets/";
    std::string gt_folder = "../data/gt/";

    std::vector<std::string> files;
    readDirectory(det_folder, files);

    for(const auto& f:files){
        std::string gt_test_folder = f.substr(0, f.find(".txt"));
        std::cout<<gt_test_folder<<std::endl;
        auto gt = readMOTFormat(gt_folder + gt_test_folder + "/gt/gt.txt",',', true);
        auto det = readMOTFormat(det_folder + f, ' ');
        computeTrackingMetrics(gt, det, threshold, false);
    }

    return EXIT_SUCCESS;

}