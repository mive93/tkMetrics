#include <iostream>
#include <set>
#include "readData.h"
#include "trackingMetrics.h"

int main(int argc, char *argv[]) {
    bool world = false;
    float threshold = 0.5;
    auto gt = readMOTFormat("../data/gt.txt",',', true);
    auto det = readMOTFormat("../data/MOT17-11-FRCNN.txt", ' ');

    computeTrackingMetrics(gt, det, threshold, false);

}