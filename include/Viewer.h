#ifndef VIEWER_H
#define VIEWER_H

#include "Utils.h"
#include "Frame.h"
#include "Tracking.h"
#include "Mapping.h"
#include <mutex>
#include <pangolin/pangolin.h>

namespace plane_slam2
{

class Viewer
{
public:
    Viewer( const std::string &setting_file);

    void run();

    void requestFinish();

    void requestStop();

    bool isFinished();

    bool isStopped();

    void release();

    //
    void setTracker(Tracking *tracker) {tracker_ = tracker;}
    void setMapper(Mapping *mapper) {mapper_ = mapper;}

    //
    void setImageRGB(cv::Mat &rgb);
    cv::Mat getImageRGB();
    void setPlanes(std::vector<MapPlane> &planes);
    std::vector<MapPlane> getPlanes();
    void setFrame(Frame *frame);
    Frame* getFrame();

private:
    void showImageRGB(pangolin::View &d_cam_img, pangolin::OpenGlRenderState &s_cam_img,
                      cv::Mat &image_rgb, pangolin::GlTexture &image_texture, bool display_image = true);
    void showPlanes(pangolin::View &d_cam_plane, pangolin::OpenGlRenderState &s_cam_plane,
                    std::vector<MapPlane> &planes, bool display_plane = true);
    void showMap(pangolin::View &d_cam_map, pangolin::OpenGlRenderState &s_cam_map, bool display_map = true);
    // Draw functions
    inline void setGLColor(const PointType& p){ glColor4ub(p.r, p.g, p.b, p.a); }
    template <typename PT> inline void setGLTranslate(const PT& p){
        glTranslatef(p.x, p.y, p.z);
    }
    void draw3DText( float x, float y, float z, const std::string &text);
    void drawWindowText( float u, float v, const std::string &text);
    void drawTriangle(const PointType& p1, const PointType& p2, const PointType& p3);
    void drawAxes(float scale = 1.0, float thickness = 5.0);
    template<typename PT> void drawPointCloud(const pcl::PointCloud<PT> &cloud,
                                              GLubyte r, GLubyte g, GLubyte b, GLubyte a = 255,
                                              float point_size = 1.0f);
    template<typename PT> void drawCloudHull(const pcl::PointCloud<PT> &cloud,
                                             GLubyte r, GLubyte g, GLubyte b, GLubyte a = 255,
                                             float line_width = 1.0f);
    void drawLandmarkPoint(const PointType &pt);


private:
    //
    std::mutex image_rgb_mutex_;
    std::list<cv::Mat> image_rgb_list_;
    cv::Mat image_rgb_;
    //
    std::mutex planes_mutex_;
    std::list< std::vector<MapPlane> > planes_list_;
    std::vector<MapPlane> planes_;
    //
    std::mutex frame_mutex_;
    std::list<Frame*> frame_list_;
    Frame *frame_;

private:
    bool stop();

    Tracking* tracker_;
    Mapping* mapper_;

    // 1/fps in ms
    float fps_;
    int cycle_;
    int image_width_,image_height_;
    int panel_width_;

    //
    int viewer_width_, viewer_height_;
    float viewpoint_x_, viewpoint_y_, viewpoint_z_, viewpoint_f_;

    bool checkFinish();
    void setFinish();
    bool finish_requested_;
    bool finished_;
    std::mutex mutex_finish_;

    bool stopped_;
    bool stop_requested_;
    std::mutex mutex_stop_;

private:
    // Draw parameters
    float landmark_point_radius_;
    int landmark_point_slices_;
    int landmark_point_stacks_;
    float cloud_point_size_;

};

}

#endif
