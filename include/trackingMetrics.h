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

float computeFRA(const clearMotMexRes_t& res){
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
   return 2.*float(true_positive)/(gt_size + dt_size);
}
 



void computeTrackingMetrics(std::vector<metrics::BoundingBox> gt, std::vector<metrics::BoundingBox> det, const float threshold, const bool world, bool verbose=false){

    auto res = clearMotMex(gt, det, threshold, world);

    int max_frame_gt = 0;
    for(const auto&g: gt)
        if(g.frameId > max_frame_gt) max_frame_gt = g.frameId;

    int max_frame_det = 0;
    for(const auto&d: det)
        if(d.frameId > max_frame_det) max_frame_det = d.frameId;

	int missed          = std::accumulate(res.m.begin(), res.m.end(), 0);
    int false_positives = std::accumulate(res.fp.begin(), res.fp.end(), 0);
    int id_switches     = std::accumulate(res.mme.begin(), res.mme.end(), 0);
    int matches         = std::accumulate(res.c.begin(), res.c.end(), 0);
    int gt_count        = std::accumulate(res.g.begin(), res.g.end(), 0);

    // std::cout<<"sum(m) "<<missed<<std::endl;
    // std::cout<<"sum(fp) "<<false_positives<<std::endl;
    // std::cout<<"sum(mme) "<<id_switches<<std::endl;
    // std::cout<<"sum(c) "<<matches<<std::endl;
    // std::cout<<"sum(g) "<<gt_count<<std::endl;

    double tot_dist = 0;
    for(const auto& d:res.d)
        tot_dist+= std::accumulate(d.begin(), d.end(), 0.);
    

    // std::cout<<tot_dist<<std::endl;

    int ML, PT, MT;
    float MOTP  = computeMOTP(matches, tot_dist, threshold, world);
    float MOTAL = computeMOTAL(missed, false_positives, id_switches, gt_count);
    float MOTA  = computeMOTA(missed, false_positives, id_switches, gt_count);
    float prcn  = computePrecision(matches, false_positives);
    float rcll  = computeRecall(matches, gt_count);
    float FAR   = computeFAR(false_positives, max_frame_gt);
    float FRA   = computeFRA(res);
    computeMLPTMT(res, gt, max_frame_det, MT, PT, ML);

    int IDFP, IDFN, IDTP, GT;
    computeintIDTPFPFN(gt, det, threshold, world, IDTP, IDFP, IDFN, GT);

    float IDP   = computePrecision(IDTP, IDFP);
    float IDR   = computeRecall(IDTP, IDTP+IDFN);
    float IDF1  = computeIDF1(IDTP,gt.size(), det.size()) * 100;


    if(true){

        std::cout<< "IDF1\tIDP\tIDR|\tRcll\tPrcn\tFAR|\tGT\tMT\tPT\tML|\tFP\tFN\tIDs\tFM|\tMOTA\tMOTP\tMOTAL"<<std::endl;

        std::cout<< std::fixed  << std::setprecision(2)
                    <<IDF1<<"\t"<<IDP<<"\t"<<IDR<<"\t"
                    <<rcll<<"\t"<<prcn<<"\t"<<FAR<<"\t"
                    <<GT<<"\t"<<MT<<"\t"<<PT<<"\t"<<ML<<"\t"
                    <<false_positives<<"\t"<<missed<<"\t"<<id_switches<<"\t"<<FRA<<"\t"
                    <<MOTA<<"\t"<<MOTP<<"\t"<<MOTAL<<std::endl;

    }
    
}

#endif /*TRACKING_METRICS_H*/
