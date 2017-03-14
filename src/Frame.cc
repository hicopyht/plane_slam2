#include "Frame.h"
#include <opencv2/imgproc/imgproc.hpp>


namespace plane_slam2
{

Frame::Frame( const Mat &image_rgb, const Mat &image_depth,
              const CAMERA_PARAMETERS &camera_image, const CAMERA_PARAMETERS &camera_cloud,
              OrbExtractor *orb_extractor, PlaneSegmentor *plane_segmentor)
{
    // Get point cloud
    cv::Mat indices_mask;
    getOrganizedCloud( image_rgb, image_depth, camera_image, camera_cloud, cloud_, indices_mask);

    thread threadExtract( &Frame::extractKeypoint, this, orb_extractor, image_rgb, image_depth, &camera_image);
    thread threadSegment( &Frame::segmentPlane, this, plane_segmentor, indices_mask);
    threadExtract.join();
    threadSegment.join();
}

void Frame::extractKeypoint(OrbExtractor *orb_extractor, cv::Mat image_rgb, cv::Mat image_depth, const CAMERA_PARAMETERS *camera)
{
    // Get gray image
    cv::Mat image_gray;
    if(image_rgb.type() == CV_8UC3)
        cv::cvtColor( image_rgb, image_gray, CV_BGR2GRAY );
    else
        image_gray = image_rgb;

    cv::Mat mat_descriptors;
    // Extract features
    (*orb_extractor)( image_gray, cv::Mat(), feature_locations_2d_, mat_descriptors);

    // Project Keypoint to 3D
    projectKeypointTo3D( image_depth, *camera, mat_descriptors, feature_locations_2d_,
                         feature_descriptors_, feature_locations_3d_);

//    cv::drawKeypoints(image_rgb, feature_locations_2d_, image_rgb_);
    image_rgb_ = image_rgb;
}


void Frame::segmentPlane(PlaneSegmentor *plane_segmentor, cv::Mat indices_mask)
{
    (*plane_segmentor)(cloud_, indices_mask, planes_);
}

void Frame::projectKeypointTo3D( const Mat &image_depth,
                                 const CAMERA_PARAMETERS &camera,
                                 cv::Mat &mdescriptors,
                                 vector<cv::KeyPoint> &locations_2d,
                                 vector<cv::Mat> &descriptors,
                                 vector<pcl::PointXYZ> &locations_3d)
{
    // Clear
    descriptors.clear();
    locations_3d.clear();
    std::vector<cv::KeyPoint> new_locations_2d;

    const float min_depth = 0.1;
    const float invfx = 1.0 / camera.fx;
    const float invfy = 1.0 / camera.fy;
    const float scale = camera.scale;
    for(size_t i = 0; i < locations_2d.size(); i++)
    {
        cv::Point2f p2d = locations_2d[i].pt;
        pcl::PointXYZ p3d;
        float Z = image_depth.at<float>(p2d.x + p2d.y*camera.width) * scale;
        // Check for invalid measurements
        if (Z <= min_depth) //Should also be trigger on NaN//std::isnan (Z))
        {
            p3d.x = (p2d.x - camera.cx) * invfx; //FIXME: better solution as to act as at 1meter?
            p3d.y = (p2d.y - camera.cy) * invfy;
            p3d.z = std::numeric_limits<float>::quiet_NaN();
        }
        else // Fill in XYZ
        {
            p3d.x = (p2d.x - camera.cx) * Z * invfx;
            p3d.y = (p2d.y - camera.cy) * Z * invfy;
            p3d.z = Z;
        }

        // Check for invalid measurements
        if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z) )
        {
            continue;
        }

        //
        new_locations_2d.push_back(locations_2d[i]);
        descriptors.push_back(mdescriptors.row(i));
        locations_3d.push_back(p3d);
    }

    locations_2d = new_locations_2d;
}

void Frame::getOrganizedCloud( const Mat &m_depth,
                               const CAMERA_PARAMETERS &camera_image,
                               const CAMERA_PARAMETERS &camera_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                               cv::Mat &indices_mask,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_std_cov_)
{
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>);
    cloud->is_dense = false;
    cloud->width = camera_cloud.width;
    cloud->height = camera_cloud.height;
    cloud->points.resize(cloud->width * cloud->height);
    //
    indices_mask = cv::Mat::ones(camera_cloud.height, camera_cloud.width, CV_8UC1);
    //
    cloud_std_cov_ = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>);
    cloud_std_cov_->is_dense = false;
    cloud_std_cov_->width = camera_cloud.width;
    cloud_std_cov_->height = camera_cloud.height;
    cloud_std_cov_->points.resize(cloud_std_cov_->width * cloud_std_cov_->height);

    const double invfx = 1.0 / camera_image.fx;
    const double invfy = 1.0 / camera_image.fy;
//    const double min_depth = range_min_depth_;
    const double min_depth = 0.1;
    pcl::PointCloud<pcl::PointXYZ>::iterator cloud_end = cloud->end();
    pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud->begin();
    pcl::PointCloud<pcl::PointXYZ>::iterator std_cov_end = cloud_std_cov_->end();
    pcl::PointCloud<pcl::PointXYZ>::iterator std_cov_iter = cloud_std_cov_->begin();
    int skip = camera_image.width / camera_cloud.width;
    int depth_idx = 0;
    //
    MatIterator_<uchar> mat_iter = indices_mask.begin<uchar>(), mat_end = indices_mask.end<uchar>();
    for (int v = 0; v < m_depth.rows; v+=skip)
    {
        depth_idx = v *m_depth.cols;
        for (int u = 0; u < m_depth.cols; u+=skip)
        {
            if(pt_iter == cloud_end || std_cov_iter == std_cov_end || mat_iter == mat_end)
                break;

            pcl::PointXYZ &pt = *pt_iter;
            pcl::PointXYZ &std_cov = *std_cov_iter;
            //
            float Z = m_depth.at<float>(depth_idx);
            // Check for invalid measurements
            if (Z <= min_depth || isnan(Z)) //Should also be trigger on NaN//std::isnan (Z))
            {
                pt.x = (u - camera_image.cx) * 1.0 * invfx; //FIXME: better solution as to act as at 1meter?
                pt.y = (v - camera_image.cy) * 1.0 * invfy;
                pt.z = std::numeric_limits<float>::quiet_NaN();
                //
                std_cov.x = -1.0;
                std_cov.y = -1.0;
                std_cov.z = -1.0;
            }
            else // Fill in XYZ
            {
                float Zifx = Z*invfx;
                float Zify = Z*invfy;
                float Zd = Z-0.4;
                pt.x = (u - camera_image.cx) * Zifx;
                pt.y = (v - camera_image.cy) * Zify;
                pt.z = Z;
                //
                std_cov.x = 0.8 * Zifx;
                std_cov.y = 0.8 * Zify;
                std_cov.z = 0.0012+0.009*Zd*Zd;
            }

            //
            pt_iter ++;
            std_cov_iter ++;
            depth_idx += skip;
        }
    }
}

void Frame::getOrganizedCloud( const Mat &m_depth,
                               const CAMERA_PARAMETERS &camera_image,
                               const CAMERA_PARAMETERS &camera_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                               cv::Mat &indices_mask)
{
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>);
    cloud->is_dense = false;
    cloud->width = camera_cloud.width;
    cloud->height = camera_cloud.height;
    cloud->points.resize(cloud->width * cloud->height);
    //
    indices_mask = cv::Mat::ones(camera_cloud.height, camera_cloud.width, CV_8UC1);

    const double invfx = 1.0 / camera_image.fx;
    const double invfy = 1.0 / camera_image.fy;
//    const double min_depth = range_min_depth_;
    const double min_depth = 0.1;
    pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud->begin();
    pcl::PointCloud<pcl::PointXYZ>::iterator cloud_end = cloud->end();
    int skip = camera_image.width / camera_cloud.width;
    int depth_idx = 0;
    //
    MatIterator_<uchar> mat_iter = indices_mask.begin<uchar>(), mat_end = indices_mask.end<uchar>();
    for (int v = 0; v < m_depth.rows; v+=skip)
    {
        depth_idx = v *m_depth.cols;
        for (int u = 0; u < m_depth.cols; u+=skip)
        {
            if(pt_iter == cloud_end || mat_iter == mat_end)
            {
                break;
            }
            pcl::PointXYZ &pt = *pt_iter;
            float Z = m_depth.at<float>(depth_idx);
            // Check for invalid measurements
            if (Z <= min_depth || isnan(Z)) //Should also be trigger on NaN//std::isnan (Z))
            {
                pt.x = (u - camera_image.cx) * 1.0 * invfx; //FIXME: better solution as to act as at 1meter?
                pt.y = (v - camera_image.cy) * 1.0 * invfy;
                pt.z = std::numeric_limits<float>::quiet_NaN();
                //
                *mat_iter = 0;
            }
            else // Fill in XYZ
            {
                pt.x = (u - camera_image.cx) * Z * invfx;
                pt.y = (v - camera_image.cy) * Z * invfy;
                pt.z = Z;
            }

            //
            pt_iter ++;
            mat_iter ++;
            depth_idx += skip;
        }
    }
}

void Frame::getOrganizedCloud( const Mat &m_rgb, const Mat &m_depth,
                               const CAMERA_PARAMETERS &camera_image,
                               const CAMERA_PARAMETERS &camera_cloud,
                               pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                               cv::Mat &indices_mask)
{
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr( new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloud->is_dense = false;
    cloud->width = camera_cloud.width;
    cloud->height = camera_cloud.height;
    cloud->points.resize(cloud->width * cloud->height);
    //
    indices_mask = cv::Mat::ones(camera_cloud.height, camera_cloud.width, CV_8UC1);

    const double invfx = 1.0 / camera_image.fx;
    const double invfy = 1.0 / camera_image.fy;
//    const double min_depth = range_min_depth_;
    const double min_depth = 0.1;
    pcl::PointCloud<pcl::PointXYZRGBA>::iterator pt_iter = cloud->begin();
    pcl::PointCloud<pcl::PointXYZRGBA>::iterator cloud_end = cloud->end();
    int skip = camera_image.width / camera_cloud.width;
    int depth_idx = 0;
    int color_idx = 0;
    int color_skip_idx = 3 * skip;
    //
    MatIterator_<uchar> mat_iter = indices_mask.begin<uchar>(), mat_end = indices_mask.end<uchar>();
    for (int v = 0; v < m_depth.rows; v+=skip)
    {
        depth_idx = v *m_depth.cols;
        color_idx = depth_idx * 3;
        for (int u = 0; u < m_depth.cols; u+=skip)
        {
            if(pt_iter == cloud_end || mat_iter == mat_end)
            {
                break;
            }
            pcl::PointXYZRGBA &pt = *pt_iter;
            float Z = m_depth.at<float>(depth_idx);
            // Check for invalid measurements
            if (Z <= min_depth || isnan(Z)) //Should also be trigger on NaN//std::isnan (Z))
            {
                pt.x = (u - camera_image.cx) * 1.0 * invfx; //FIXME: better solution as to act as at 1meter?
                pt.y = (v - camera_image.cy) * 1.0 * invfy;
                pt.z = std::numeric_limits<float>::quiet_NaN();
                //
                *mat_iter = 0;
            }
            else // Fill in XYZ
            {
                pt.x = (u - camera_image.cx) * Z * invfx;
                pt.y = (v - camera_image.cy) * Z * invfy;
                pt.z = Z;
            }

            RGBValue color;

            color.Blue = m_rgb.at<uint8_t>(color_idx);
            color.Green = m_rgb.at<uint8_t>(color_idx+1);
            color.Red = m_rgb.at<uint8_t>(color_idx+2);
            color.Alpha = 255.0;
            pt.rgb = color.float_value;

            //
            pt_iter ++;
            mat_iter ++;
            depth_idx += skip;
            color_idx += color_skip_idx;
        }
    }
}

void Frame::getOrganizedCloud( const Mat &m_rgb, const Mat &m_depth,
                               const CAMERA_PARAMETERS &camera_image,
                               const CAMERA_PARAMETERS &camera_cloud,
                               pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud)
{
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr( new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloud->is_dense = false;
    cloud->width = camera_cloud.width;
    cloud->height = camera_cloud.height;
    cloud->points.resize(cloud->width * cloud->height);

    const double invfx = 1.0 / camera_image.fx;
    const double invfy = 1.0 / camera_image.fy;
//    const double min_depth = range_min_depth_;
    const double min_depth = 0.1;
    pcl::PointCloud<pcl::PointXYZRGBA>::iterator pt_iter = cloud->begin();
    int skip = camera_image.width / camera_cloud.width;
    int depth_idx = 0;
    int color_idx = 0;
    int color_skip_idx = 3 * skip;
    for (int v = 0; v < m_depth.rows; v+=skip)
    {
        depth_idx = v *m_depth.cols;
        color_idx = depth_idx * 3;
        for (int u = 0; u < m_depth.cols; u+=skip)
        {
            if(pt_iter == cloud->end())
            {
                break;
            }
            pcl::PointXYZRGBA &pt = *pt_iter;
            float Z = m_depth.at<float>(depth_idx);
            // Check for invalid measurements
            if (Z <= min_depth || isnan(Z)) //Should also be trigger on NaN//std::isnan (Z))
            {
                pt.x = (u - camera_image.cx) * 1.0 * invfx; //FIXME: better solution as to act as at 1meter?
                pt.y = (v - camera_image.cy) * 1.0 * invfy;
                pt.z = std::numeric_limits<float>::quiet_NaN();
            }
            else // Fill in XYZ
            {
                pt.x = (u - camera_image.cx) * Z * invfx;
                pt.y = (v - camera_image.cy) * Z * invfy;
                pt.z = Z;
            }

            RGBValue color;

            color.Blue = m_rgb.at<uint8_t>(color_idx);
            color.Green = m_rgb.at<uint8_t>(color_idx+1);
            color.Red = m_rgb.at<uint8_t>(color_idx+2);
            color.Alpha = 255.0;
            pt.rgb = color.float_value;

            //
            pt_iter ++;
            depth_idx += skip;
            color_idx += color_skip_idx;
        }
    }
}


}
