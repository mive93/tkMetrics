#include "costBlockMex.h"

void correspondingFrames(const std::vector<tk::metrics::BoundingBox>& tr1, const std::vector<tk::metrics::BoundingBox>& tr2, int* loc)
{
	int pos = 0;
	int i = 0;
	while (i < tr1.size() && pos < tr2.size()) {
		while (pos < tr2.size()) {
			if (tr1[i].frameId == tr2[pos].frameId) {
				loc[i] = pos; pos++; i++;
				break;
			}
			else if (tr1[i].frameId < tr2[pos].frameId) {
				loc[i] = -1; i++;
				if (i == tr1.size()) break;
			}
			else pos++;
		}
	}
}

void computeDistances(const std::vector<tk::metrics::BoundingBox>& tr1, const std::vector<tk::metrics::BoundingBox>& tr2, int* position, bool world, double* distance)
{
	if (world)	{
		for (int i = 0; i < tr1.size(); i++){
			if (position[i] == -1)
				distance[i] = 1e9;
			else
				distance[i] =  tr1[i].euclidean(tr2[position[i]]);
		}
	}
	else{
		for (int i = 0; i < tr1.size(); i++){
			if (position[i] == -1)
				distance[i] = 0;
			else
				distance[i] =  tr1[i].IoUtracker(tr2[position[i]]);
		}
	}
}

void compute(const std::vector<tk::metrics::BoundingBox>& tr1, const std::vector<tk::metrics::BoundingBox>& tr2, double threshold, bool world, double& cost, double& fp, double& fn)
{
	int numPoints1 = tr1.size();
	int numPoints2 = tr2.size();
	int tr1start, tr1end, tr2start, tr2end;
	tr1start = tr1[0].frameId;
	tr1end = tr1[numPoints1-1].frameId;
	tr2start = tr2[0].frameId;
	tr2end = tr2[numPoints2-1].frameId;


	bool overlapTest = ((tr1start >= tr2start && tr1start <= tr2end) ||
		(tr1end >= tr2start && tr1end <= tr2end) ||
		(tr2start >= tr1start && tr2start <= tr1end) ||
		(tr2end >= tr1start && tr2end <= tr1end));
		
	if (!overlapTest){
		fp = numPoints2;
		fn = numPoints1;
		cost = numPoints1 + numPoints2;
		return;
	}

	int* positionGT = new int[numPoints1];
	int* positionPred = new int[numPoints2];
	for (int i = 0; i < numPoints1; i++) positionGT[i] = -1;
	for (int i = 0; i < numPoints2; i++) positionPred[i] = -1;
	double* distanceGT = new double[numPoints1];
	double* distancePred = new double[numPoints2];
	
	correspondingFrames(tr1,tr2, positionGT);
	correspondingFrames(tr2, tr1, positionPred);
	computeDistances(tr1, tr2, positionGT, world, distanceGT);
	computeDistances(tr2, tr1,positionPred, world, distancePred);

	fp = 0; fn = 0;
	if (world) {
		for (int i = 0; i < numPoints1; i++) if (distanceGT[i] > threshold) fn++;
		for (int i = 0; i < numPoints2; i++) if (distancePred[i] > threshold) fp++;
	}
	else {
		for (int i = 0; i < numPoints1; i++) if (distanceGT[i] < threshold) fn++;
		for (int i = 0; i < numPoints2; i++) if (distancePred[i] < threshold) fp++;
	}
	cost = fp + fn;

	delete[] positionGT;
	delete[] positionPred;
	delete[] distanceGT;
	delete[] distancePred;

}

void costBlockMex(const std::vector<std::vector<tk::metrics::BoundingBox>> &gt, const std::vector<std::vector<tk::metrics::BoundingBox>>&det, const float threshold, std::vector<std::vector<double>> &fn, std::vector<std::vector<double>>& fp, std::vector<std::vector<double>>& cost, const bool world)
{
#pragma omp parallel for 
	for (int i = 0; i < gt.size(); i++) {
#pragma omp parallel for 
		for (int j = 0; j < det.size(); j++) {
			int ind = j*gt.size()+ i;
			compute(gt[i], det[j], threshold, world, cost[i][j], fp[i][j], fn[i][j]);
		}
	}	
}