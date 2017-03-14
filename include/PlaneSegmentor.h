#ifndef PLANE_SEGMENTOR_H
#define PLANE_SEGMENTOR_H

#include <line_based_plane_segmentation.h>
#include <pcl/io/io.h>
#include "Utils.h"
#include "MapPlane.h"


using namespace std;

namespace plane_slam2
{

typedef line_based_plane_segment::CAMERA_INFO CLOUD_CAMERA_INFO;
typedef line_based_plane_segment::PlaneType PLANE_TYPE;

class PlaneSegmentor
{
public:
    PlaneSegmentor(CLOUD_CAMERA_INFO *camera_cloud);
    PlaneSegmentor(const std::string &file_setting, CLOUD_CAMERA_INFO *camera_cloud);
    void operator()(PointCloudTypePtr &input, cv::Mat &indices_mask, std::vector<MapPlane> &planes);

private:
    line_based_plane_segment::LineBasedPlaneSegmentation * segmentor_;
    cv::RNG rng_;
};

}

#endif
