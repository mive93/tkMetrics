#include "readData.h"
#include "trackingMetrics.h"
#include "tkMetricsUtils.h"

int main(int argc, char *argv[]) {
    bool world = false;
    float IoU_threshold = 0.5;
    std::string gt_folder = "../data/gt/";
    std::string det_folder = "../data/dets/";
    char delimiter = ' ';

    if(argc > 1)
        gt_folder = argv[1]; 
    if(argc > 2)
        det_folder = argv[2]; 
    if(argc > 3)
        IoU_threshold = atof(argv[3]);
    if(argc > 4)
        delimiter = argv[4][0];

    std::vector<std::string> files= getDirectoryFiles(det_folder);

    std::vector<tk::metrics::trackingMetrics_t> results;
    for(const auto& f:files){
        std::string gt_test_folder = f.substr(0, f.find(".txt"));
        std::cout<<gt_test_folder<<std::endl;
        auto gt = readMOTFormat(gt_folder + gt_test_folder + "/gt/gt.txt",',', true);
        auto det = readMOTFormat(det_folder + f, delimiter);
        results.push_back(computeTrackingMetrics(gt, det, IoU_threshold, false));
    }

    std::cout<<"Total evaluation: "<<std::endl;
    evaluateTrackingBenchmark(results);

    return EXIT_SUCCESS;

}