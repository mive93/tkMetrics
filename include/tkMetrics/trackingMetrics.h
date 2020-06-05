#ifndef TRACKING_METRICS_H
#define TRACKING_METRICS_H

#include <iostream>
#include <iomanip>
#include <set>
#include <cassert>
#include "clearMOTMex.h"
#include "costBlockMex.h"
#include "MinCostMatching.h"



float computeMOTP(const int matches, const double tot_dist, const float threshold, const bool world=false){
    //Multi-object tracking precision in [0,100]
    float MOTP = 0;
    if(matches != 0) MOTP = (1-tot_dist/matches) * 100; 
    if(world) MOTP = MOTP / threshold; 
    return MOTP;
}

float computeMOTA(const int missed, const int false_positives, const int id_switches, const int gt_count){
    //Multi-object tracking accuracy in [0,100]
    return (1-((missed+false_positives+(id_switches))/float(gt_count)))*100;
}

float computeMOTAL(const int missed, const int false_positives, const int id_switches, const int gt_count){
    //Multi-object tracking accuracy in [0,100] with log10(idswitches)
    return (1-((missed+false_positives+log10(id_switches+1))/gt_count))*100;
}

float computePrecision(const int true_positive, const int false_positive){
    float precision = 0;
    if(true_positive + false_positive > 0) precision = float(true_positive)/(true_positive + false_positive)*100;
    return precision;
}

float computeRecall(const int true_positive, const int gt_count){
    //measue of completeness
    return float(true_positive)/gt_count*100;
}

float computeFAR(const int false_positives, const int max_frame_gt){
    //number of false alarms per frame
    return float(false_positives)/max_frame_gt;
}

void computeMLPTMT(const clearMotMexRes_t& res, std::vector<metrics::BoundingBox> gt, int max_frame_det, int& MT, int& PT, int& ML){
    MT = 0; //mostly tracked
    PT = 0; //partially tracked
    ML = 0; //mostly lost trajectories
    
    for(int i=1; i<= res.Ngt; ++i){
        std::vector<int> gt_frames;
        for(const auto &g:gt)
            if(g.trackId == i) 
                gt_frames.push_back(g.frameId);

        if(gt_frames.size()){

            float non_negative = 0;
            for(const auto& gf: gt_frames)
                if(res.allTracked[gf-1][i-1] > 0) non_negative++;

            if (non_negative/float(gt_frames.size()) < 0.2)
                ML++;
            else if (max_frame_det>= gt_frames[gt_frames.size()-1] && non_negative/float(gt_frames.size()) <= 0.8)
                PT++;
            else if (non_negative/float(gt_frames.size()) >= 0.8)
                MT++;

        }
    }
}

int computeFRA(const clearMotMexRes_t& res){
    //number of fragmentations in trajectories
    int FRA = 0;
    for(int i=0; i< res.Ngt; ++i){
        
        int first_non_null = 0;
        while(first_non_null < res.allTracked.size() && !res.allTracked[first_non_null][i])    
            first_non_null++;
        if(first_non_null < res.allTracked.size()){
            int prev = res.allTracked[first_non_null][i];
            for(int j=first_non_null; j< res.allTracked.size();++j){
                if(res.allTracked[j][i] != 0 && res.allTracked[j][i] != prev && prev == 0)
                    FRA++;
                prev = res.allTracked[j][i];
            }
        }
    }
    return FRA;
}

void computeintIDTPFPFN(const std::vector<metrics::BoundingBox>& gt, const std::vector<metrics::BoundingBox>& det,  const float threshold, const bool world, int& IDTP,int& IDFP,int& IDFN, int& GT){
    std::set<int> gt_unique_gt_track_ID;
    for(const auto& g:gt)
        gt_unique_gt_track_ID.insert(g.trackId);
    GT = gt_unique_gt_track_ID.size();
    
    // for(auto u:gt_unique_gt_track_ID)
    //     std::cout<<u<<" ";
    // std::cout<<std::endl;
    // std::cout<<gt_unique_gt_track_ID.size()<<std::endl;

    std::set<int> det_unique_gt_track_ID;
    for(const auto& d:det)
        det_unique_gt_track_ID.insert(d.trackId);

    std::vector<std::vector<metrics::BoundingBox>> gt_per_trackID (gt_unique_gt_track_ID.size());
    int i=0;
    for(const auto& u: gt_unique_gt_track_ID){
        for(const auto&g:gt)
            if(g.trackId == u)
                gt_per_trackID[i].push_back(g);
        i++;
    }
    
    // int j= 2;
    // for(auto& g:gt_per_trackID[j])
    //     std::cout<<g<<std::endl;
    // std::cout<<gt_per_trackID[j].size()<<std::endl;


    std::vector<std::vector<metrics::BoundingBox>> det_per_trackID (det_unique_gt_track_ID.size());
    i=0;
    for(const auto& u: det_unique_gt_track_ID){
        
        for(const auto&d:det)
            if(d.trackId == u)
                det_per_trackID[i].push_back(d);
        i++;
    }

    //initialize cost
    double INF = 1e9;
    int cost_size = det_unique_gt_track_ID.size() + gt_unique_gt_track_ID.size();
    // std::cout<<cost_size<<std::endl;
    std::vector<std::vector<double>> cost (cost_size, std::vector<double>(cost_size, 0));
    for(int i=0; i< cost_size; ++i){
        if(i < gt_unique_gt_track_ID.size())
            for(int j=det_unique_gt_track_ID.size(); j< cost_size; ++j)
                cost[i][j] = INF;
        else
            for(int j=0; j< det_unique_gt_track_ID.size(); ++j)
                cost[i][j] = INF;
    }

    std::vector<std::vector<double>> fp (cost_size, std::vector<double>(cost_size, 0));
    std::vector<std::vector<double>> fn (cost_size, std::vector<double>(cost_size, 0));


    // int j= 85;
    // for(auto& g:det_per_trackID[j])
    //     std::cout<<g<<std::endl;
    // std::cout<<det_per_trackID[j].size()<<std::endl;



    // for(auto u:det_unique_gt_track_ID)
    //     std::cout<<u<<" ";
    // std::cout<<std::endl;
    // std::cout<<det_unique_gt_track_ID.size()<<std::endl;





    costBlockMex(gt_per_trackID, det_per_trackID, threshold, fn, fp, cost, world);

    for(int i=0, j; i< det_per_trackID.size();++i){
        j = i + gt_per_trackID.size();
        cost[j][i] = det_per_trackID[i].size();
        fp[j][i] = det_per_trackID[i].size();
    }

    for(int i=0, j; i< gt_per_trackID.size();++i){
        j = i + det_per_trackID.size();
        cost[i][j] = gt_per_trackID[i].size();
        fn[i][j] = gt_per_trackID[i].size();
    }



    double total_cost = 0;
    std::vector<std::vector<double>> optimal_match;
    applyMinCostMatching(cost, optimal_match, total_cost );

    // std::cout<<total_cost<<std::endl;
    
    // for(auto c_r:optimal_match){
    //     for(auto c_v: c_r)
    //         std::cout<<c_v<<" ";
    //     std::cout<<std::endl;
    // }
    
    std::vector<int> assignment(cost_size,0);
    for(int i=0; i<optimal_match.size();++i){
        for(int j=0; j<optimal_match[i].size();++j)
            if(optimal_match[i][j]){
                assignment[i] = j;
                break;
            }
    }

    // for(auto a:assignment)
    //     std::cout<<a<<" ";
    // std::cout<<std::endl;


    IDFP = 0;
    IDFN = 0;

    for(int i=0; i<assignment.size(); ++i){
        IDFP = IDFP + fp[i][assignment[i]];
        IDFN = IDFN + fn[i][assignment[i]];
    }

    IDTP = gt.size() - IDFN;
    assert(IDTP == det.size() - IDFP);

}

float computeIDF1(const int true_positive, const int gt_size, const int dt_size){
   return (2.*float(true_positive)/(gt_size + dt_size)) * 100;
}
 

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


trackingMetrics_t computeTrackingMetrics(std::vector<metrics::BoundingBox> gt, std::vector<metrics::BoundingBox> det, const float threshold, const bool world, bool verbose=false){

    trackingMetrics_t metrics;
    metrics.numGt       = gt.size();
    metrics.numDet      = det.size();
    metrics.threshold   = threshold;
    metrics.world       = world;

    //compute clear mot metrics
    auto res    = clearMotMex(gt, det, threshold, world);
    metrics.Ngt = res.Ngt;
    metrics.Fgt = res.Fgt;

    int max_frame_gt = 0;
    for(const auto&g: gt)
        if(g.frameId > max_frame_gt) max_frame_gt = g.frameId;

    int max_frame_det = 0;
    for(const auto&d: det)
        if(d.frameId > max_frame_det) max_frame_det = d.frameId;

	metrics.FN          = std::accumulate(res.m.begin(), res.m.end(), 0);
    metrics.FP          = std::accumulate(res.fp.begin(), res.fp.end(), 0);
    metrics.IDs         = std::accumulate(res.mme.begin(), res.mme.end(), 0);
    metrics.TP          = std::accumulate(res.c.begin(), res.c.end(), 0);
    metrics.gt_count    = std::accumulate(res.g.begin(), res.g.end(), 0);

    for(const auto& d:res.d)
        metrics.tot_dist+= std::accumulate(d.begin(), d.end(), 0.);
    
    metrics.MOTP  = computeMOTP(metrics.TP, metrics.tot_dist, threshold, world);
    metrics.MOTAL = computeMOTAL(metrics.FN, metrics.FP, metrics.IDs, metrics.gt_count);
    metrics.MOTA  = computeMOTA(metrics.FN, metrics.FP, metrics.IDs, metrics.gt_count);
    metrics.prcn  = computePrecision(metrics.TP, metrics.FP);
    metrics.rcll  = computeRecall(metrics.TP, metrics.gt_count);
    metrics.FAR   = computeFAR(metrics.FP, metrics.Fgt);
    metrics.FRA   = computeFRA(res);

    computeMLPTMT(res, gt, max_frame_det, metrics.MT, metrics.PT, metrics.ML);

    computeintIDTPFPFN(gt, det, threshold, world, metrics.IDTP, metrics.IDFP, metrics.IDFN, metrics.GT);

    metrics.IDprcn  = computePrecision(metrics.IDTP, metrics.IDFP);
    metrics.IDrcll  = computeRecall(metrics.IDTP, metrics.IDTP+metrics.IDFN);
    metrics.IDF1    = computeIDF1(metrics.IDTP,metrics.numGt, metrics.numDet);

    metrics.printMetrics();
    return metrics;
}



void evaluateBenchmark(std::vector<trackingMetrics_t> results){

    trackingMetrics_t final_metrics;

    for(int i=0; i< results.size(); ++i){
        final_metrics.numGt     += results[i].numGt;
        final_metrics.numDet    += results[i].numDet;
        final_metrics.IDTP      += results[i].IDTP;
        final_metrics.IDFP      += results[i].IDFP;
        final_metrics.IDFN      += results[i].IDFN;
        final_metrics.MT        += results[i].MT;
        final_metrics.ML        += results[i].ML;
        final_metrics.PT        += results[i].PT;
        final_metrics.FRA       += results[i].FRA;
        final_metrics.FP        += results[i].FP;
        final_metrics.FN        += results[i].FN;
        final_metrics.IDs       += results[i].IDs;
        final_metrics.Fgt       += results[i].Fgt;
        final_metrics.Ngt       += results[i].Ngt;
        final_metrics.TP        += results[i].TP;
        final_metrics.GT        += results[i].GT;
        final_metrics.tot_dist  += results[i].tot_dist;
        final_metrics.gt_count  += results[i].gt_count;
    }

    final_metrics.IDprcn    = computePrecision(final_metrics.IDTP, final_metrics.IDFP);
    final_metrics.IDrcll    = computeRecall(final_metrics.IDTP, final_metrics.IDTP+final_metrics.IDFN);
    final_metrics.IDF1      = computeIDF1(final_metrics.IDTP, final_metrics.numGt, final_metrics.numDet);
    final_metrics.FAR       = computeFAR(final_metrics.FP, final_metrics.Fgt);
    final_metrics.MOTP      = computeMOTP(final_metrics.TP,final_metrics.tot_dist, results[0].threshold, results[0].world);
    final_metrics.MOTAL     = computeMOTAL(final_metrics.FN, final_metrics.FP, final_metrics.IDs, final_metrics.gt_count);
    final_metrics.MOTA      = computeMOTA(final_metrics.FN, final_metrics.FP, final_metrics.IDs, final_metrics.gt_count);
    final_metrics.rcll      = computeRecall(final_metrics.TP, final_metrics.gt_count);
    final_metrics.prcn      = computePrecision(final_metrics.TP,final_metrics.FP);

    final_metrics.printMetrics();
    
}

#endif /*TRACKING_METRICS_H*/
