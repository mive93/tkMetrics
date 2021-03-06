
#ifndef CLEARMOTMEX_H
#define CLEARMOTMEX_H

#include "BoundingBox.h"
#include <cmath>
#include <omp.h>
#include <set>
#include <map>
#include <vector>
#include <algorithm>
#include <numeric>
#include <iterator>
#include <unordered_map>

struct clearMotMexRes_t{
	std::vector<int> mme; // ID Switchtes(mismatches)
	std::vector<int> c; // matches found
	std::vector<int> fp; // false positives
	std::vector<int> m; // misses = false negatives
	std::vector<int> g; // gt count for each frame
	std::vector<std::vector<double>> d; // all distances mapped to [0..1]
	std::vector<std::vector<int>> allFalsePos;
    std::vector<std::vector<int>> allTracked;

	int MT = 0;
    int PT = 0;
    int ML = 0;
	int FRA = 0;
	float threshold = 0;
	int Fgt = 0;
	int Ngt = 0;
};

clearMotMexRes_t clearMotMex(std::vector<tk::metrics::BoundingBox> gt, std::vector<tk::metrics::BoundingBox> det, const float threshold, const bool world, bool verbose=false);

#endif /* CLEARMOTMEX_H */