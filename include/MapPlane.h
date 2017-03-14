#ifndef MAP_PLANE_H
#define MAP_PLANE_H

#include "Utils.h"

using namespace std;

namespace plane_slam2
{

class MapPlane
{
public:
    MapPlane();

    int id_;
    PlaneCoefficients coefficients_;
    PointType centroid_;
    PointCloudTypePtr cloud_;
    PointCloudTypePtr boundary_;
    PointCloudTypePtr hull_;
    //
    RGBValue color_;
    float render_point_size_;

    void render();
};

}

#endif
