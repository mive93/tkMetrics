#ifndef MINCOSTMATCHING_H
#define MINCOSTMATCHING_H

#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdlib>

void applyMinCostMatching( const std::vector<std::vector<double>>& cost, std::vector<std::vector<double>>& optimal_match, double& total_cost );

#endif /*MINCOSTMATCHING_H*/