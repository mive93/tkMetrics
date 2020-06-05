#include <iostream>
#include <set>
#include "readData.h"
#include "clearMOTMex.h"

int main(int argc, char *argv[]) {
    std::cout<<"hello"<<std::endl;


    bool world = false;
    float threshold = 0.5;
    auto gt = readMOTFormat("../data/gt.txt",',', true);
    auto det = readMOTFormat("../data/MOT17-11-FRCNN.txt", ' ');

    auto res = clearMotMex(gt, det, threshold, world);



	// for(auto miss:m)
	int missed = std::accumulate(res.m.begin(), res.m.end(), 0);
    int false_positives = std::accumulate(res.fp.begin(), res.fp.end(), 0);
    int id_switches = std::accumulate(res.mme.begin(), res.mme.end(), 0);
    int matches = std::accumulate(res.c.begin(), res.c.end(), 0);
    int gt_count = std::accumulate(res.g.begin(), res.g.end(), 0);


    std::cout<<"sum(m) "<<missed<<std::endl;
    std::cout<<"sum(fp) "<<false_positives<<std::endl;
    std::cout<<"sum(mme) "<<id_switches<<std::endl;
    std::cout<<"sum(c) "<<matches<<std::endl;
    std::cout<<"sum(g) "<<gt_count<<std::endl;

    double tot_dist = 0;
    for(const auto& d:res.d)
        tot_dist+= std::accumulate(d.begin(), d.end(), 0.);
    

    std::cout<<tot_dist<<std::endl;

    float MOTP = 0;
    if(matches != 0) MOTP = (1-tot_dist/matches) * 100; 
    if(world) MOTP = MOTP / threshold; 


    float MOTAL     = (1-((missed+false_positives+log10(id_switches+1))/gt_count))*100;
    float MOTA      = (1-((missed+false_positives+(id_switches))/float(gt_count)))*100;
    float recall    = float(matches)/gt_count*100;
    float precision = 0;
    if(false_positives + matches > 0) precision = float(matches)/(matches + false_positives)*100;

    int max_frame_gt = 0;
    for(const auto&g: gt)
        if(g.frameId > max_frame_gt) max_frame_gt = g.frameId;

    int max_frame_det = 0;
    for(const auto&d: det)
        if(d.frameId > max_frame_det) max_frame_det = d.frameId;

    float FAR= float(false_positives)/max_frame_gt;

    std::cout<<" MOTP: "<< MOTP<<std::endl;
    std::cout<<" MOTA: "<< MOTA<<std::endl;
    std::cout<<" MOTAL: "<< MOTAL<<std::endl;
    std::cout<<" precision: "<< precision<<std::endl;
    std::cout<<" recall: "<< recall<<std::endl;
    std::cout<<" FAR: "<< FAR<<std::endl;

    std::set<int> unique_gt_track_ID;
    int max_track_id = 0;
    for(const auto& g:gt){
        unique_gt_track_ID.insert(g.trackId);
        if(g.trackId > max_track_id) 
            max_track_id = g.trackId;
    }

    int Ngt = unique_gt_track_ID.size();

    int MT = 0;
    int PT = 0;
    int ML = 0;
    
    for(int i=1; i<= max_track_id; ++i){
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

    std::cout<<" ML: "<< ML<<std::endl;
    std::cout<<" PT: "<< PT<<std::endl;
    std::cout<<" MT: "<< MT<<std::endl;



    //find fragmented tracks 
    int FRA = 0;
    for(int i=0; i< max_track_id; ++i){
        
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

    std::cout<<" FRA: "<< FRA<<std::endl;

}