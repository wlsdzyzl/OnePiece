#ifndef ARRANGEMENT_OF_LINES_H
#define ARRANGEMENT_OF_LINES_H

#include "DCEL.h"
#include <memory>
namespace one_piece
{
namespace algorithm
{
//A demo for arrangments of lines: https://linearrangement.stackblitz.io/
    void ComputeIntersect(const std::vector<geometry::Line> &lines, 
        geometry::Point2List &intersect_points);
    std::shared_ptr<DCEL> CreateBoundingBoxDcel(const geometry::Point2List &points);

}
}
#endif