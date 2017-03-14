#ifndef MAP_POINT_H
#define MAP_POINT_H

#include "Utils.h"

namespace plane_slam2
{

class MapPoint
{
public:
    MapPoint();

    int id_;
    cv::Mat descriptor_;
    pcl::PointXYZ position_;
    //
    RGBValue color_;
    int render_point_size_;

    void render();

};

}

#endif
