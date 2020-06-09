#include "BoundingBox.h"

namespace tk { namespace metrics{

float BoundingBox::overlap(const float p1, const float d1, const float p2, const float d2) const{
    float l1 = p1 - d1/2.;
    float l2 = p2 - d2/2.;
    float left = l1 > l2 ? l1 : l2;
    float r1 = p1 + d1/2.;
    float r2 = p2 + d2/2.;
    float right = r1 < r2 ? r1 : r2;
    return right - left;
}

float BoundingBox::boxesIntersection(const BoundingBox &b) const{
    float width = this->overlap(x, w, b.x, b.w);
    float height = this->overlap(y, h, b.y, b.h);
    if(width < 0 || height < 0) 
        return 0;
    float area = width*height;
    return area;
}

float BoundingBox::boxesUnion(const BoundingBox &b) const{
    float i = this->boxesIntersection(b);
    float u = w*h + b.w*b.h - i;
    return u;
}

float BoundingBox::IoU(const BoundingBox &b) const{
    float I = this->boxesIntersection(b);
    float U = this->boxesUnion(b);
    if (I == 0 || U == 0) 
        return 0;
    return I / U;
}

double BoundingBox::IoUtracker(const BoundingBox &b) const {
    double area1 = w * h;
	double area2 = b.w * b.h;

	double x_overlap = std::max(float(0.0), std::min(x + w, b.x + b.w) - std::max(x, b.x));
	double y_overlap = std::max(float(0.0), std::min(y + h, b.y + b.h) - std::max(y, b.y));
	double intersectionArea = x_overlap*y_overlap;
	double unionArea = area1 + area2 - intersectionArea;
	double iou = intersectionArea / unionArea;
	return iou;
}

float BoundingBox::euclidean(const BoundingBox &b) const{
	return std::sqrt((xRealWorld - b.xRealWorld)*(xRealWorld - b.xRealWorld) + (yRealWorld - b.yRealWorld)*(yRealWorld - b.yRealWorld));
}

void BoundingBox::clear(){
    uniqueTruthIndex = -1;
    truthFlag = 0;
    maxIoU = 0;
}

std::ostream& operator<<(std::ostream& os, const BoundingBox& bb){
    os  << "\t"
        << "frameID: "<< bb.frameId
        << ", trackID: "<< bb.trackId
        << ", x: "<< bb.x 
        << ", y: "<< bb.y 
        << ", w: "<< bb.w 
        << ", h: "<< bb.h 
        << ", cat: "<< bb.cl 
        << ", conf: "<< bb.prob
        << ", xReal: "<< bb.xRealWorld
        << ", yReal: "<< bb.yRealWorld
        << ", truth: "<< bb.truthFlag
        << ", assignedGT: "<< bb.uniqueTruthIndex
        << ", maxIoU: "<< bb.maxIoU;
    return os;
}

bool boxComparison (const BoundingBox& a,const BoundingBox& b) { 
    return (a.prob>b.prob); 
}

}}