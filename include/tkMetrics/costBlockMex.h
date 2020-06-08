
#ifndef COSTBLOCKMEX_H
#define COSTBLOCKMEX_H

#include <cmath>
#include <omp.h>
#include <algorithm>
#include <vector>
#include "BoundingBox.h"

void costBlockMex(const std::vector<std::vector<tk::metrics::BoundingBox>> &gt, const std::vector<std::vector<tk::metrics::BoundingBox>>&det, const float threshold, std::vector<std::vector<double>> &fn, std::vector<std::vector<double>>& fp, std::vector<std::vector<double>>& cost, const bool world=false);

#endif /*COSTBLOCKMEX_H*/