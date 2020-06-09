#include "clearMOTMex.h"
// Min cost bipartite matching std::vector<int>a shortest augmenting paths
//
// Code from https://github.com/jaehyunp/
//
// This is an O(n^3) implementation of a shortest augmenting path
// algorithm for finding min cost perfect matchings in dense
// graphs.  In practice, it solves 1000x1000 problems in around 1
// second.
//
//   cost[i][j] = cost for pairing left node i with right node j
//   Lmate[i] = index of right node that left node i pairs with
//   Rmate[j] = index of left node that right node j pairs with
//
// The values in cost[i][j] may be positive or negative.  To perform
// maximization, simply negate the cost[][] matrix.

double MinCostMatchingClear(const std::vector<std::vector<double>> &cost, std::vector<int> &Lmate, std::vector<int> &Rmate) {
	int n = int(cost.size());

	// construct dual feasible solution
	std::vector<double> u(n);
	std::vector<double> v(n);
	for (int i = 0; i < n; i++) {
		u[i] = cost[i][0];
		for (int j = 1; j < n; j++) u[i] = std::min(u[i], cost[i][j]);
	}
	for (int j = 0; j < n; j++) {
		v[j] = cost[0][j] - u[0];
		for (int i = 1; i < n; i++) v[j] = std::min(v[j], cost[i][j] - u[i]);
	}

	// construct primal solution satisfying complementary slackness
	Lmate = std::vector<int>(n, -1);
	Rmate = std::vector<int>(n, -1);
	int mated = 0;
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			if (Rmate[j] != -1) continue;
			if (fabs(cost[i][j] - u[i] - v[j]) < 1e-10) {
				Lmate[i] = j;
				Rmate[j] = i;
				mated++;
				break;
			}
		}
	}

	std::vector<double> dist(n);
	std::vector<int> dad(n);
	std::vector<int> seen(n);

	// repeat until primal solution is feasible
	while (mated < n) {

		// find an unmatched left node
		int s = 0;
		while (Lmate[s] != -1) s++;

		// initialize Dijkstra
		fill(dad.begin(), dad.end(), -1);
		fill(seen.begin(), seen.end(), 0);
		for (int k = 0; k < n; k++)
			dist[k] = cost[s][k] - u[s] - v[k];

		int j = 0;
		while (true) {

			// find closest
			j = -1;
			for (int k = 0; k < n; k++) {
				if (seen[k]) continue;
				if (j == -1 || dist[k] < dist[j]) j = k;
			}
			seen[j] = 1;

			// termination condition
			if (Rmate[j] == -1) break;

			// relax neighbors
			const int i = Rmate[j];
			for (int k = 0; k < n; k++) {
				if (seen[k]) continue;
				const double new_dist = dist[j] + cost[i][k] - u[i] - v[k];
				if (dist[k] > new_dist) {
					dist[k] = new_dist;
					dad[k] = j;
				}
			}
		}

		// update dual variables
		for (int k = 0; k < n; k++) {
			if (k == j || !seen[k]) continue;
			const int i = Rmate[k];
			v[k] += dist[k] - dist[j];
			u[i] -= dist[k] - dist[j];
		}
		u[s] += dist[j];

		// augment along path
		while (dad[j] >= 0) {
			const int d = dad[j];
			Rmate[j] = Rmate[d];
			Lmate[Rmate[j]] = j;
			j = d;
		}
		Rmate[j] = s;
		Lmate[s] = j;

		mated++;
	}

	double value = 0;
	for (int i = 0; i < n; i++)
		value += cost[i][Lmate[i]];

	return value;
}

clearMotMexRes_t clearMotMex(std::vector<tk::metrics::BoundingBox> gt, std::vector<tk::metrics::BoundingBox> det, const float threshold, const bool world, bool verbose)
{
    
	// data format: frame, ID, left, top, width, height, worldX, worldY
    int Fgt=0, Ngt = 0, Nst = 0;
    
    for (int i = 0; i < gt.size(); ++i) {
        int frame = gt[i].frameId;
        int ID = gt[i].trackId;
        if (frame > Fgt) Fgt = frame;
        if (ID > Ngt) Ngt = ID;
    }
    for (int i = 0; i < det.size(); i++) {
		int frame = det[i].frameId;
		if (frame > Fgt) Fgt = frame;
		int ID = det[i].trackId;
        if (ID > Nst) Nst = ID;
    } 

	double INF = 1e9;

	std::vector<std::unordered_map<int, int>> gtInd(Fgt);
	std::vector<std::unordered_map<int, int>> stInd(Fgt);
	std::vector<std::unordered_map<int,int>> M(Fgt);
	std::vector<int> mme(Fgt, 0); // ID Switchtes(mismatches)
	std::vector<int> c(Fgt, 0); // matches found
	std::vector<int> fp(Fgt, 0); // false positives
	std::vector<int> m(Fgt, 0); // misses = false negatives
	std::vector<int> g(Fgt, 0); // gt count for each frame
	std::vector<std::vector<double>> d(Fgt, std::vector<double>(Ngt, 0)); // all distances mapped to [0..1]
	std::vector<std::vector<int>> allfalsepos(Fgt, std::vector<int>(Nst, 0));
	std::vector<std::vector<int>> alltracked(Fgt, std::vector<int>(Ngt, 0));

	for (int i = 0; i < gt.size(); i++) {
		int frame = gt[i].frameId-1;
		int ID = gt[i].trackId-1;
		gtInd[frame][ID] = i;
	}
	for (int i = 0; i < det.size(); i++) {
		int frame = det[i].frameId-1;
		int ID = det[i].trackId-1;
		stInd[frame][ID] = i;
	}

	for (int i = 0; i < Fgt; i++) {
		for (std::unordered_map<int, int>::iterator it = gtInd[i].begin(); it != gtInd[i].end(); it++) g[i]++;
	}

	
	for (int t = 0; t < Fgt; t++){
		if (t > 0){
			std::vector<int> mappings;
			for (std::unordered_map<int,int>::iterator it = M[t - 1].begin(); it != M[t - 1].end(); it++) mappings.push_back(it->first);
			sort(mappings.begin(), mappings.end());
			for (int k = 0; k < mappings.size(); k++){
				std::unordered_map<int, int>::const_iterator foundGtind = gtInd[t].find(mappings[k]);
				std::unordered_map<int, int>::const_iterator foundStind = stInd[t].find(M[t - 1][mappings[k]]);

				if (foundGtind != gtInd[t].end() && foundStind != stInd[t].end()){
					bool matched = false;
					if (world){
						int rowgt = gtInd[t][mappings[k]];
						int rowres = stInd[t][M[t - 1][mappings[k]]];
						double dist = det[rowres].euclidean(gt[rowgt]);
						matched = (dist <= threshold);
					}
					else{
						int rowgt = gtInd[t][mappings[k]];
						int rowres = stInd[t][M[t - 1][mappings[k]]];
						double dist = 1 - det[rowres].IoUtracker(gt[rowgt]);
						matched = (dist <= threshold);
					}

					if (matched) {
						M[t][mappings[k]] = M[t - 1][mappings[k]];
						if (verbose) printf("%d: preserve %d\n", t+1, mappings[k]+1);
					}
				}
			}
		}

		std::vector<int> unmappedGt, unmappedEs, stindt, findm;
		for (std::unordered_map<int,int>::iterator it = gtInd[t].begin(); it != gtInd[t].end(); it++) {
			std::unordered_map<int, int>::const_iterator found = M[t].find(it->first);
			if (found==M[t].end()) unmappedGt.push_back(it->first);
		}
		for (std::unordered_map<int,int>::iterator it = M[t].begin(); it != M[t].end(); it++) findm.push_back(it->second);
		for (std::unordered_map<int,int>::iterator it = stInd[t].begin(); it != stInd[t].end(); it++) stindt.push_back(it->first);

		sort(stindt.begin(), stindt.end());
		sort(findm.begin(), findm.end());
		set_difference(stindt.begin(), stindt.end(), findm.begin(), findm.end(), inserter(unmappedEs, unmappedEs.end()));

        sort(unmappedGt.begin(), unmappedGt.end());
        
		int squareSize = std::max(unmappedGt.size(), unmappedEs.size());
		std::vector<std::vector<double>> alldist(squareSize, std::vector<double>(squareSize, INF));

		if (verbose){
			printf("%d: UnmappedGTs: ", t+1);
			for (int i = 0; i < unmappedGt.size(); i++) printf("%d, ", unmappedGt[i]+1);
			printf("\n%d: UnmappedEs: ", t+1);
			for (int i = 0; i < unmappedEs.size(); i++) printf("%d, ", unmappedEs[i]+1);
			printf("\n");
		}

        int uid = 0; // Unique identifier
		for (int i = 0; i < unmappedGt.size(); i++){
			for (int j = 0; j < unmappedEs.size(); j++){
				int o = unmappedGt[i];
				int e = unmappedEs[j];
				if (world){
					int rowgt = gtInd[t][o];
					int rowres = stInd[t][e];
					double dist = det[rowres].euclidean(gt[rowgt]);
					if (dist <= threshold) {
                        alldist[i][j] = dist;
                        // Add unique identifier to break ties
                        alldist[i][j] += 1e-9 * uid;
                        uid++;
                    }
				}
				else{
					int rowgt = gtInd[t][o];
					int rowres = stInd[t][e];
					double dist = 1 - det[rowres].IoUtracker(gt[rowgt]);
                    if (dist <= threshold) {
                        alldist[i][j] = dist;
                        // Add unique identifier to break ties
                        alldist[i][j] += 1e-9 * uid;
                        uid++;
                    }
				}
			}
		}

		std::vector<int> Lmate, Rmate;
		MinCostMatchingClear(alldist, Lmate, Rmate);

		for (int k = 0; k < Lmate.size(); k++) {
			if (alldist[k][Lmate[k]] == INF) continue;
			M[t][unmappedGt[k]] = unmappedEs[Lmate[k]];
			if (verbose) printf("%d: map %d with %d\n", t+1, unmappedGt[k]+1, unmappedEs[Lmate[k]]+1);
		}

		std::vector<int> curtracked, alltrackers, mappedtrackers, falsepositives, set1;

		for (std::unordered_map<int,int>::iterator it = M[t].begin(); it != M[t].end(); it++) {
			curtracked.push_back(it->first);
			set1.push_back(it->second);
		}
		for (std::unordered_map<int,int>::iterator it = stInd[t].begin(); it != stInd[t].end(); it++) alltrackers.push_back(it->first);

		sort(set1.begin(), set1.end());
		sort(alltrackers.begin(), alltrackers.end());
		set_intersection(set1.begin(), set1.end(), alltrackers.begin(), alltrackers.end(), inserter(mappedtrackers, mappedtrackers.begin()));
		set_difference(alltrackers.begin(), alltrackers.end(), mappedtrackers.begin(), mappedtrackers.end(), inserter(falsepositives, falsepositives.end()));

		for (int k = 0; k < falsepositives.size(); k++) allfalsepos[t][falsepositives[k]] = falsepositives[k];

		//  mismatch errors
		if (t > 0){
			for (int k = 0; k < curtracked.size(); k++){
				int ct = curtracked[k];
				int lastnonempty = -1;
				for (int j = t - 1; j >= 0; j--) {
					if (M[j].find(ct) != M[j].end()) {
						lastnonempty = j; break;
					}
				}
				if (gtInd[t-1].find(ct)!=gtInd[t-1].end() && lastnonempty != -1){
					int mtct = -1, mlastnonemptyct = -1;
					if (M[t].find(ct) != M[t].end()) mtct = M[t][ct];
					if (M[lastnonempty].find(ct) != M[lastnonempty].end()) mlastnonemptyct = M[lastnonempty][ct];

					if (mtct != mlastnonemptyct)
						mme[t]++;
				}
			}
		}

		c[t] = curtracked.size();
		for (int k = 0; k < curtracked.size(); k++){
			int ct = curtracked[k];
			int eid = M[t][ct];
			if (world){
				int rowgt = gtInd[t][ct];
				int rowres = stInd[t][eid];
				d[t][ct] = det[rowres].euclidean(gt[rowgt]);
			}
			else{
				int rowgt = gtInd[t][ct];
				int rowres = stInd[t][eid];
				d[t][ct] = 1 - det[rowres].IoUtracker(gt[rowgt]);
			}
		}

		for (std::unordered_map<int,int>::iterator it = stInd[t].begin(); it != stInd[t].end(); it++) fp[t]++;
		fp[t] -= c[t];
		m[t] = g[t] - c[t];
	}

	for (int i = 0; i < Fgt; i++) {
		for (std::unordered_map<int, int>::iterator it = M[i].begin(); it != M[i].end(); it++) {
			int j = it->first;
			alltracked[i][j] = M[i][j];
		}
	}

	clearMotMexRes_t res;
	res.allFalsePos = allfalsepos;
	res.c = c;
	res.d = d;
	res.fp = fp;
	res.g = g;
	res.m = m;
	res.mme = mme;
	res.allTracked = alltracked;
	res.Fgt = Fgt;
	res.Ngt = Ngt;

	return res;
}