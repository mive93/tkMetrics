#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <iostream>
#include <cmath>

namespace tk {namespace metrics {
class BoundingBox
{
    float overlap(const float p1, const float l1, const float p2, const float l22) const;
    float boxesIntersection(const BoundingBox &b) const;
    float boxesUnion(const BoundingBox &b) const;

    public:
    
    float xRealWorld = 0;
    float yRealWorld = 0;

    float x = 0;
    float y = 0;
    float w = 0;
    float h = 0;
    float prob = 0;
    int cl = 0;

    int frameId = 0;
    int trackId = 0;
    
    float maxIoU = 0;
    int uniqueTruthIndex = -1;
    int truthFlag = 0;
    

    float IoU(const BoundingBox &b) const;
    double IoUtracker(const BoundingBox &b) const;
    float euclidean(const BoundingBox &b) const;
    void clear();

    friend std::ostream& operator<<(std::ostream& os, const BoundingBox& bb);
};

std::ostream& operator<<(std::ostream& os, const BoundingBox& bb);
bool boxComparison (const BoundingBox& a,const BoundingBox& b) ;

}}
#endif /*BOUNDINGBOX_H*/

