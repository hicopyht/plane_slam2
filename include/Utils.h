#ifndef UTILS_H
#define UTILS_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/concatenate.h>

using namespace std;

namespace plane_slam2
{

// RGB Value
typedef union
{
    struct /*anonymous*/
    {
        unsigned char Blue;
        unsigned char Green;
        unsigned char Red;
        unsigned char Alpha;
    };
    float float_value;
    long long_value;
} RGBValue;

const double DEG_TO_RAD = ( M_PI / 180.0 );
const double RAD_TO_DEG = ( 180.0 / M_PI );

//typedef PlaneFromLineSegment::CAMERA_PARAMETERS  CameraParameters;
//
typedef pcl::PointCloud< pcl::PointXYZ > PointCloudXYZ;
typedef PointCloudXYZ::Ptr PointCloudXYZPtr;
//
typedef pcl::PointCloud< pcl::PointXYZRGBA > PointCloudXYZRGBA;
typedef PointCloudXYZRGBA::Ptr PointCloudXYZRGBAPtr;
//
typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud< PointType > PointCloudType;
typedef PointCloudType::Ptr PointCloudTypePtr;
typedef PointCloudType::ConstPtr PointCloudTypeConstPtr;
//
typedef boost::shared_ptr<const pcl::PointRepresentation< PointType > > PointRepresentationConstPtr;
typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > std_vector_of_eigen_vector4f;
typedef Eigen::Vector4f PlaneCoefficients;


void loadParam(cv::FileStorage &fs, const std::string &name, bool &var, bool default_value);
void loadParam(cv::FileStorage &fs, const std::string &name, int &var, int default_value);
void loadParam(cv::FileStorage &fs, const std::string &name, float &var, float default_value);
void loadParam(cv::FileStorage &fs, const std::string &name, double &var, double default_value);
void loadParam(cv::FileStorage &fs, const std::string &name, std::string &var, std::string default_value);
//
void randomRGBColor(cv::RNG &rng, RGBValue &color);
void getPointCloudFromIndices(const PointCloudTypePtr &input,
                              const std::vector<int> &indices,
                              PointCloudTypePtr &output);
void getPointCloudFromIndices(const PointCloudTypePtr &input,
                              const std::vector<int> &indices,
                              const Eigen::Vector4f &coefficients,
                              PointCloudTypePtr &output);

}

//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */


#endif // UTILS_H
