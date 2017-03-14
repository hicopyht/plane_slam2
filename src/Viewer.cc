#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <opencv2/highgui/highgui.hpp>
#include <mutex>

namespace plane_slam2
{

Viewer::Viewer( const std::string &setting_file)
    : cycle_(1e6/30)
    , frame_(NULL)
    , image_rgb_(cv::Mat())
    , finish_requested_(false)
    , stop_requested_(false)
    , finished_(false)
    , stopped_(false)
    , planes_(0)
{
    cout << WHITE << "Load viewer parameters." << RESET << endl;
    cv::FileStorage fs(setting_file, cv::FileStorage::READ);

    loadParam(fs, "Camera.fps", fps_, 30.0);
    cycle_ = 1e6/fps_;
    //
    loadParam(fs, "Camera.width", image_width_, 640);
    loadParam(fs, "Camera.height", image_height_, 480);
    //
    loadParam(fs, "Viewer.width", viewer_width_, 1280);
    loadParam(fs, "Viewer.height", viewer_height_, 720);
    loadParam(fs, "Viewer.viewpoint_x", viewpoint_x_, 0.0);
    loadParam(fs, "Viewer.viewpoint_y", viewpoint_y_, -0.7);
    loadParam(fs, "Viewer.viewpoint_z", viewpoint_z_, -1.8);
    loadParam(fs, "Viewer.viewpoint_f", viewpoint_f_, 500.0);
    loadParam(fs, "Viewer.panel_width", panel_width_, image_width_/2);

    // Draw parameters
    loadParam(fs, "MapDrawer.landmark_point_radius", landmark_point_radius_, 0.05);
    loadParam(fs, "MapDrawer.landmark_point_slices", landmark_point_slices_, 16);
    loadParam(fs, "MapDrawer.landmark_point_stacks", landmark_point_stacks_, 8);
    loadParam(fs, "MapDrawer.cloud_point_size", cloud_point_size_, 1.0);

}

void Viewer::run()
{
    finished_ = false;
    const int UI_WIDTH = panel_width_;

    pangolin::CreateWindowAndBind("Plane-Slam2: Map Viewer",viewer_width_,viewer_height_);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam_map(
                pangolin::ProjectionMatrix(viewer_width_-UI_WIDTH,viewer_height_,viewpoint_f_,viewpoint_f_,(viewer_width_-UI_WIDTH)/2.0f,viewer_height_/2.0f,0.1,1000),
                pangolin::ModelViewLookAt(viewpoint_x_,viewpoint_y_,viewpoint_z_, 0,0,0,0.0,-1.0, 0.0)
                );

    pangolin::OpenGlRenderState s_cam_plane(
                pangolin::ProjectionMatrix(UI_WIDTH,UI_WIDTH*0.75f,262.5,262.5,UI_WIDTH/2.0f,UI_WIDTH*0.375f,0.1,1000),
                pangolin::ModelViewLookAt(0,-0.1,-0.1, 0,0.1,2.0, 0.0,-1.0,0.0)
                );

    pangolin::OpenGlRenderState s_cam_image(
                pangolin::ProjectionMatrix(UI_WIDTH,UI_WIDTH*0.75f,262.5,262.5,UI_WIDTH/2.0f,UI_WIDTH*0.375f,0.1,1000),
                pangolin::ModelViewLookAt(0,-0.1,-0.1, 0,0.1,2.0, 0.0,-1.0,0.0)
                );


    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam_map = pangolin::Display("map")
            .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(viewer_width_-UI_WIDTH))
            .SetHandler(new pangolin::Handler3D(s_cam_map));

    pangolin::View& d_cam_plane = pangolin::Display("planes")
            .SetBounds(0.0, pangolin::Attach::Pix(UI_WIDTH*0.75), pangolin::Attach::Pix(viewer_width_-UI_WIDTH), 1.0)
            .SetHandler(new pangolin::Handler3D(s_cam_plane));

    pangolin::View& d_cam_image = pangolin::Display("image")
            .SetBounds(pangolin::Attach::Pix(UI_WIDTH*0.75+5), pangolin::Attach::Pix(UI_WIDTH*1.5), pangolin::Attach::Pix(viewer_width_-UI_WIDTH), 1.0)
            .SetHandler(new pangolin::Handler3D(s_cam_image));

    pangolin::Display("multi")
        .SetBounds(0.0, 1.0, 0.0, 1.0)
        .SetLayout(pangolin::LayoutOverlay)
        .AddDisplay(d_cam_map)
        .AddDisplay(d_cam_plane)
        .AddDisplay(d_cam_image);

    pangolin::CreatePanel("menu1").SetBounds( pangolin::Attach::Pix(10+UI_WIDTH*1.5), 1.0, pangolin::Attach::Pix(viewer_width_-UI_WIDTH), pangolin::Attach::Pix(viewer_width_-UI_WIDTH/2.0));
    pangolin::Var<bool> menu_show_frame_image("menu1.Show Image",true,true);
    pangolin::Var<bool> menu_show_frame_planes("menu1.Show Planes",true,true);
    pangolin::Var<bool> menu_show_map("menu1.Show Map",true,true);

    pangolin::CreatePanel("menu2").SetBounds( pangolin::Attach::Pix(10+UI_WIDTH*1.5), 1.0, pangolin::Attach::Pix(viewer_width_-UI_WIDTH/2.0), 1.0);
//    pangolin::Var<bool> menuFollowCamera("menu2.Follow Camera",false,true);
    pangolin::Var<bool> saveWindow("menu2.Save Window",false,false);
    pangolin::Var<bool> recordWindow("menu2.Record Window",false,false);
//    pangolin::Var<bool> menuStop("menu2.Stop", false, false);
//    pangolin::Var<bool> menuResume("menu2.Resume", false, false);
//    pangolin::Var<bool> menuReset("menu2.Reset",false,false);

    pangolin::GlTexture image_texture(640, 480, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);


    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f,1.0f,1.0f,1.0f);


        if( pangolin::Pushed(saveWindow) )
            pangolin::SaveWindowOnRender("window_"+timeToStr());

        if( pangolin::Pushed(recordWindow) )
            pangolin::DisplayBase().RecordOnRender("ffmpeg:[fps="+std::to_string(fps_)+",bps=8388608,unique_filename]//cap_"+timeToStr()+".avi");

        // Update buffer
        cv::Mat image_rgb = getImageRGB();
        std::vector<MapPlane> planes = getPlanes();

        // Image render
        d_cam_image.Activate();
        showImageRGB(d_cam_image, s_cam_image, image_rgb, image_texture, menu_show_frame_image);

        // Planes render
        d_cam_plane.Activate(s_cam_plane);
        showPlanes(d_cam_plane, s_cam_plane, planes, menu_show_frame_planes);

//        // Map render
        d_cam_map.Activate(s_cam_map);
        drawAxes(1.0, 5.0);    // origin coordinate
        showMap(d_cam_map, s_cam_map, menu_show_map);

        pangolin::FinishFrame();


        usleep(cycle_);

        if(stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(checkFinish())
            break;
    }

    setFinish();
}

/*----------------------------------------------------------------------------------------*/
void Viewer::showImageRGB(pangolin::View &d_cam_img, pangolin::OpenGlRenderState &s_cam_img,
                          cv::Mat &image_rgb, pangolin::GlTexture &image_texture, bool display_image)
{
    if(!display_image || image_rgb.empty())
        return;

    image_texture.Upload(image_rgb.data, 0, 0, 640, 480, GL_BGR, GL_UNSIGNED_BYTE);
    glColor3f(1.0f, 1.0f, 1.0f);
    image_texture.RenderToViewportFlipY();
}

void Viewer::showPlanes(pangolin::View &d_cam_plane, pangolin::OpenGlRenderState &s_cam_plane,
                       std::vector<MapPlane> &planes, bool display_plane)
{
    if(!display_plane || planes.empty())
        return;

    for(size_t i = 0; i < planes.size(); i++)
    {
        MapPlane &plane = planes[i];
        if( plane.cloud_->size() == 0)
            continue;
        drawPointCloud(*plane.cloud_, plane.color_.Red, plane.color_.Green, plane.color_.Blue, plane.color_.Alpha, 1.0f);
        drawPointCloud(*plane.boundary_, 255, 0, 0, 255, 2.0f);
    }
}

void Viewer::showMap(pangolin::View &d_cam_map, pangolin::OpenGlRenderState &s_cam_map, bool display_map)
{
    drawAxes(1.0, 5.0);    // origin coordinate

    if(!display_map)
        return;
}

/*----------------------------------------------------------------------------------------*/
void Viewer::setImageRGB(cv::Mat &rgb)
{
    unique_lock<mutex> lock(image_rgb_mutex_);
    image_rgb_list_.push_back(rgb);
}

cv::Mat Viewer::getImageRGB()
{
    unique_lock<mutex> lock(image_rgb_mutex_);
    if(image_rgb_list_.size() > 0)
    {
        image_rgb_ = image_rgb_list_.front();
        image_rgb_list_.pop_front();
    }
    return image_rgb_;
}

void Viewer::setPlanes(std::vector<MapPlane> &planes)
{
    unique_lock<mutex> lock(planes_mutex_);
    planes_list_.push_back(planes);
}

std::vector<MapPlane> Viewer::getPlanes()
{
    unique_lock<mutex> lock(planes_mutex_);
    if(planes_list_.size() > 0)
    {
        planes_ = planes_list_.front();
        planes_list_.pop_front();
    }
    return planes_;
}

void Viewer::setFrame(Frame *frame)
{
    unique_lock<mutex> lock(frame_mutex_);
    frame_list_.push_back(frame);
}

Frame* Viewer::getFrame()
{
    unique_lock<mutex> lock(frame_mutex_);
    if(frame_list_.size() > 0)
    {
        frame_ = frame_list_.front();
        frame_list_.pop_front();
    }
    return frame_;
}
/*----------------------------------------------------------------------------------------*/
void Viewer::draw3DText( float x, float y, float z, const std::string &text)
{
    pangolin::GlText gltext = pangolin::GlFont::I().Text(text);
    gltext.Draw(x, y, z);
}

void Viewer::drawWindowText( float u, float v, const std::string &text)
{
    pangolin::GlText gltext = pangolin::GlFont::I().Text(text);
    gltext.DrawWindow(u, v);
}

void Viewer::drawTriangle(const PointType& p1, const PointType& p2, const PointType& p3)
{
    setGLColor(p1);
    glVertex3f(p1.x, p1.y, p1.z);

    setGLColor(p2);
    glVertex3f(p2.x, p2.y, p2.z);

    setGLColor(p3);
    glVertex3f(p3.x, p3.y, p3.z);
}

void Viewer::drawAxes(float scale, float thickness)
{
    glLineWidth(thickness);
    pangolin::glDrawAxis(scale);
}
//
template<typename PT>
void Viewer::drawPointCloud(const pcl::PointCloud<PT> &cloud,
                            GLubyte r, GLubyte g, GLubyte b, GLubyte a,
                            float point_size)
{
    glPointSize(point_size);
    glColor4ub(r, g, b, a);
    glBegin(GL_POINTS);

    typename pcl::PointCloud<PT>::const_iterator end = cloud.end();
    for(typename pcl::PointCloud<PT>::const_iterator it = cloud.begin(); it != end; it++)
    {
//        const PT &pt = *it;
//        if(isnan(pt.z) || isnan(pt.x) || isnan(pt.y))
//            continue;
        glVertex3f(it->x, it->y, it->z);
    }

    glEnd();
}
template<typename PT>
void Viewer::drawCloudHull(const pcl::PointCloud<PT> &cloud,
                           GLubyte r, GLubyte g, GLubyte b, GLubyte a,
                           float line_width)
{
    glLineWidth(line_width);
    glColor4ub(r, g, b, a);
    glBegin(GL_LINE_LOOP);
    typename pcl::PointCloud<PT>::const_iterator end = cloud.end();
    for(typename pcl::PointCloud<PT>::const_iterator it = cloud.begin(); it != end; it++)
    {
//        const PT &pt = *it;
//        if(isnan(pt.z) || isnan(pt.x) || isnan(pt.y))
//            continue;
        glVertex3f(it->x, it->y, it->z);
    }

    glEnd();
}
void Viewer::drawLandmarkPoint(const PointType &pt)
{
    setGLColor(pt);
    glPushMatrix();
    setGLTranslate(pt);
    GLUquadricObj* pQuadric = gluNewQuadric();
    if(pQuadric != NULL)
        gluSphere(pQuadric,landmark_point_radius_,landmark_point_slices_,landmark_point_stacks_);
    glPopMatrix();
}
/*----------------------------------------------------------------------------------------*/
//
bool Viewer::stop()
{
    unique_lock<mutex> lock(mutex_stop_);
    unique_lock<mutex> lock2(mutex_finish_);

    if(finish_requested_)
        return false;
    else if(stop_requested_)
    {
        stopped_ = true;
        stop_requested_ = false;
        return true;
    }

    return false;
}

void Viewer::release()
{
    unique_lock<mutex> lock(mutex_stop_);
    stopped_ = false;
}

void Viewer::requestFinish()
{
    unique_lock<mutex> lock(mutex_finish_);
    finish_requested_ = true;
}

void Viewer::requestStop()
{
    unique_lock<mutex> lock(mutex_stop_);
    if(!stopped_)
        stopped_ = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mutex_finish_);
    return finished_;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mutex_stop_);
    return stopped_;
}

bool Viewer::checkFinish()
{
    unique_lock<mutex> lock(mutex_finish_);
    return finish_requested_;
}

void Viewer::setFinish()
{
    unique_lock<mutex> lock(mutex_finish_);
    finished_ = true;
}

}
