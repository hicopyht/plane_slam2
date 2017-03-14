#ifndef FRAME_H
#define FRAME_H

#include <thread>
#include "MapPoint.h"
#include "MapPlane.h"
#include "OrbExtractor.h"
#include "PlaneSegmentor.h"


using namespace std;
using namespace cv;

namespace plane_slam2
{

class Viewer;

class CAMERA_PARAMETERS
{
public:
    enum {
        VGA = 8,
        QVGA = 4,
        QQVGA = 2,
        QQQVGA = 1,
        NONE = 0
    };

    CAMERA_PARAMETERS() : resl(VGA), width(640), height(480), cx(319.5), cy(239.5),
        fx(525.0), fy(525.0), scale(1.0) {}
    CAMERA_PARAMETERS( int _resl, int _width, int _height, float _cx, float _cy, float _fx, float _fy, float _scale)
        : resl(_resl), width(_width), height(_height), cx(_cx), cy(_cy), fx(_fx), fy(_fy), scale(_scale) {}
    CAMERA_PARAMETERS( CAMERA_PARAMETERS &other, int skip = 1) : resl(other.resl/skip), width(other.width/skip),
        height(other.height/skip), cx(other.cx/skip), cy(other.cy/skip), fx(other.fx/skip), fy(other.fy/skip),
        scale(other.scale){}

    friend std::ostream& operator<< (std::ostream& os, const CAMERA_PARAMETERS& cam){
        os << cam.resl << " " << cam.width << " " << cam.height << " " << cam.cx << " " << cam.cy
           << " " << cam.fx << " " << cam.fy << " " << cam.scale;
        return os;
    }
    friend std::istream& operator>> (std::istream& is, CAMERA_PARAMETERS& cam){
        is >> cam.resl >> cam.width >> cam.height >> cam.cx >> cam.cy >> cam.fx >> cam.fy >> cam.scale;
        return is;
    }

    //
    int resl;
    int width, height;
    float cx, cy, fx, fy, scale;
};


class Frame
{
public:
    Frame( const Mat &image_rgb, const Mat &image_depth,
           const CAMERA_PARAMETERS &camera_image, const CAMERA_PARAMETERS &camera_cloud,
           OrbExtractor *orb_extractor, PlaneSegmentor *plane_segmentor);

    void projectKeypointTo3D( const Mat &image_depth,
                              const CAMERA_PARAMETERS &camera,
                              cv::Mat &mdescriptors,
                              vector<cv::KeyPoint> &locations_2d,
                              vector<cv::Mat> &descriptors,
                              vector<pcl::PointXYZ> &locations_3d);

    void getOrganizedCloud( const Mat &m_depth,
                            const CAMERA_PARAMETERS &camera_image,
                            const CAMERA_PARAMETERS &camera_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                            cv::Mat &indices_mask,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_std_cov_);

    void getOrganizedCloud( const Mat &m_depth,
                            const CAMERA_PARAMETERS &camera_image,
                            const CAMERA_PARAMETERS &camera_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                            cv::Mat &indices_mask);


    void getOrganizedCloud( const Mat &m_rgb, const Mat &m_depth,
                            const CAMERA_PARAMETERS &camera_image,
                            const CAMERA_PARAMETERS &camera_cloud,
                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                            cv::Mat &indices_mask);

    void getOrganizedCloud( const Mat &m_rgb, const Mat &m_depth,
                            const CAMERA_PARAMETERS &camera_image,
                            const CAMERA_PARAMETERS &camera_cloud,
                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
private:
    void extractKeypoint(OrbExtractor *orb_extractor,
                         cv::Mat image_rgb, cv::Mat image_depth, const CAMERA_PARAMETERS *camera);
    void segmentPlane(PlaneSegmentor *plane_segmentor,
                      cv::Mat indices_mask);

public:
    //
    cv::Mat image_rgb_;

    // rgb image, depth image and downsample point cloud
    PointCloudTypePtr cloud_;
    // Plane
    vector<MapPlane> planes_;
    // Keypoint
    vector<cv::KeyPoint> feature_locations_2d_;
    vector<cv::Mat> feature_descriptors_;
    vector<pcl::PointXYZ> feature_locations_3d_;
};

}

#endif
