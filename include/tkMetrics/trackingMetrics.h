#ifndef TRACKING_METRICS_H
#define TRACKING_METRICS_H

#include <iostream>
#include <iomanip>
#include <set>
#include <cassert>
#include "clearMOTMex.h"
#include "costBlockMex.h"
#include "minCostMatching.h"

namespace tk{ namespace metrics {
struct trackingMetrics_t{
    float prcn = 0;
    float rcll = 0;
    float FAR = 0;
    float IDF1 = 0;
    float IDprcn = 0;
    float IDrcll = 0;
    int GT = 0;
    int MT = 0;
    int PT = 0;
    int ML = 0;
    int FN = 0;
    int FP = 0;
    int TP = 0;
    int gt_count = 0;
    int IDs = 0;
    float MOTA = 0;
    float MOTP = 0;
    float MOTAL = 0;
    int IDFP = 0;
    int IDTP = 0;
    int IDFN = 0;
    int FRA = 0;
    int numGt = 0;
    int numDet = 0;
    int Fgt = 0;
    int Ngt = 0;
    float tot_dist = 0;
    float threshold = 0;
    bool world = false;

    void printMetrics(){
        std::cout<< "IDF1\tIDP\tIDR|\tRcll\tPrcn\tFAR|\tGT\tMT\tPT\tML|\tFP\tFN\tIDs\tFM|\tMOTA\tMOTP\tMOTAL"<<std::endl;

        std::cout<< std::fixed  << std::setprecision(2)
                    <<IDF1<<"\t"<<IDprcn<<"\t"<<IDrcll<<"\t"
                    <<rcll<<"\t"<<prcn<<"\t"<<FAR<<"\t"
                    <<GT<<"\t"<<MT<<"\t"<<PT<<"\t"<<ML<<"\t"
                    <<FP<<"\t"<<FN<<"\t"<<IDs<<"\t"<<FRA<<"\t"
                    <<MOTA<<"\t"<<MOTP<<"\t"<<MOTAL<<std::endl;
    }
};

}}

float   computeMOTP(const int matches, const double tot_dist, const float threshold, const bool world=false);
float   computeMOTA(const int missed, const int false_positives, const int id_switches, const int gt_count);
float   computeMOTAL(const int missed, const int false_positives, const int id_switches, const int gt_count);
float   computePrecision(const int true_positive, const int false_positive);
float   computeRecall(const int true_positive, const int gt_count);
float   computeFAR(const int false_positives, const int max_frame_gt);
void    computeMLPTMT(  const clearMotMexRes_t& res, std::vector<tk::metrics::BoundingBox> gt, int max_frame_det,
                        int& MT, int& PT, int& ML);
int     computeFRA(const clearMotMexRes_t& res);
void    computeintIDTPFPFN( const std::vector<tk::metrics::BoundingBox>& gt, 
                            const std::vector<tk::metrics::BoundingBox>& det,  
                            const float threshold, const bool world, 
                            int& IDTP,int& IDFP,int& IDFN, int& GT);
float   computeIDF1(const int true_positive, const int gt_size, const int dt_size);

void    normalizeIds(std::vector<tk::metrics::BoundingBox>& boxes);
tk::metrics::trackingMetrics_t computeTrackingMetrics(  std::vector<tk::metrics::BoundingBox> gt, 
                                                        std::vector<tk::metrics::BoundingBox> det, 
                                                        const float threshold, const bool world, 
                                                        bool verbose=false);
void    evaluateBenchmark(std::vector<tk::metrics::trackingMetrics_t> results);

#endif /*TRACKING_METRICS_H*/
