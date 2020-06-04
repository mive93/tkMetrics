#include <iostream>
#include "readData.h"
#include "clearMOTMex.h"

int main(int argc, char *argv[]) {
    std::cout<<"hello"<<std::endl;


    bool world = false;
    float threshold = 0.5;
    auto gt = readMOTFormat("../data/gt.txt",',',true);
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

    int max_frame = 0;
    for(const auto&g: gt)
        if(g.frameId > max_frame) max_frame = g.frameId;

    float FAR= float(false_positives)/max_frame;

    std::cout<<" MOTP: "<< MOTP<<std::endl;
    std::cout<<" MOTA: "<< MOTA<<std::endl;
    std::cout<<" MOTAL: "<< MOTAL<<std::endl;
    std::cout<<" precision: "<< precision<<std::endl;
    std::cout<<" recall: "<< recall<<std::endl;
    std::cout<<" FAR: "<< FAR<<std::endl;



	
}