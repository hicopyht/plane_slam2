#include "PlaneSegmentor.h"

namespace plane_slam2
{

PlaneSegmentor::PlaneSegmentor(CLOUD_CAMERA_INFO *camera_cloud)
    : rng_(12345)
    , segmentor_(new line_based_plane_segment::LineBasedPlaneSegmentation(camera_cloud))
{

}

PlaneSegmentor::PlaneSegmentor(const std::string &file_setting, CLOUD_CAMERA_INFO *camera_cloud)
    : rng_(12345)
    , segmentor_(new line_based_plane_segment::LineBasedPlaneSegmentation(file_setting))
{
    if(segmentor_->getCameraParameters().width != camera_cloud->width
            || segmentor_->getCameraParameters().height != camera_cloud->height
            || segmentor_->getCameraParameters().cx != camera_cloud->cx
            || segmentor_->getCameraParameters().cy != camera_cloud->cy
            || segmentor_->getCameraParameters().fx != camera_cloud->fx
            || segmentor_->getCameraParameters().fy != camera_cloud->fy
            || segmentor_->getCameraParameters().scale != camera_cloud->scale)
    {
        cout << RED << "[Error]: Camera parameters in 'LineBasedPlaneSegmentation' doesn't match that of SLAM system. Exit." << RESET << endl;
        exit(-1);
    }
}

void PlaneSegmentor::operator()(PointCloudTypePtr &input, cv::Mat &indices_mask, std::vector<MapPlane> &planes)
{
    segmentor_->setInputCloud(input);
    segmentor_->setMask(indices_mask);
    //
    std::vector<PLANE_TYPE> segment_planes;
    segmentor_->segment(segment_planes);

    for(size_t i = 0; i < segment_planes.size(); i++)
    {
        PLANE_TYPE &pn = segment_planes[i];
        MapPlane plane;
        plane.id_ = i;
        // Random color
        randomRGBColor(rng_, plane.color_);
        // Coefficients
        plane.coefficients_ = pn.coefficients;
        // Centroid
        plane.centroid_ = pn.centroid;
        plane.centroid_.rgb = plane.color_.float_value;
        // Cloud
        getPointCloudFromIndices(input, pn.indices, plane.cloud_);
        // Boundary
        getPointCloudFromIndices(input, pn.boundary_indices, plane.boundary_);
        // Hull
        getPointCloudFromIndices(input, pn.hull_indices, plane.hull_);
        //
        planes.push_back(plane);
    }

//    cout << WHITE << " - Plane segmenting, planes = " << BLUE << segment_planes.size() << RESET << endl;
}

}
