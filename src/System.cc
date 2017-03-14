#include "System.h"
#include "Tracking.h"
#include "Mapping.h"
#include "Viewer.h"

namespace plane_slam2
{

System::System(const std::string &file_setting)
    : file_setting_(file_setting)
{
    cv::FileStorage fs(file_setting_, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        cout << "Can't open setting file: " << file_setting_ << ". Exit."<< endl;
        exit(-1);
    }

    // Read camera parameters
    CAMERA_PARAMETERS camera(CAMERA_PARAMETERS::VGA, 640, 480, 319.5, 239.5, 525.0, 525.0, 1.0);
    loadParam(fs, "Camera.width", camera.width, camera.width);
    loadParam(fs, "Camera.height", camera.height, camera.height);
    loadParam(fs, "Camera.cx", camera.cx, camera.cx);
    loadParam(fs, "Camera.cy", camera.cy, camera.cy);
    loadParam(fs, "Camera.fx", camera.fx, camera.fx);
    loadParam(fs, "Camera.fy", camera.fy, camera.fy);
    loadParam(fs, "Camera.scale", camera.scale, camera.scale);
    int skip = 2;
    loadParam(fs, "Camera.skip_pixels", skip, skip);
    camera_params_image_ = CAMERA_PARAMETERS(camera);
    camera_params_cloud_ = CAMERA_PARAMETERS(camera_params_image_, skip);

    cout << " /------------------------------------------/" << endl;
    cout << " Camera image: " << camera_params_image_ << endl;
    cout << " Camera cloud: " << camera_params_cloud_ << endl;

    // Construct orb extractor
    int nFeatures;
    float scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;
    loadParam(fs, "OrbExtractor.nFeatures", nFeatures, 1000);
    loadParam(fs, "OrbExtractor.scaleFactor", scaleFactor, 1.2);
    loadParam(fs, "OrbExtractor.nLevels", nlevels, 8);
    loadParam(fs, "OrbExtractor.iniThFAST", iniThFAST, 20);
    loadParam(fs, "OrbExtractor.minThFAST", minThFAST, 7);



    orb_extractor_ = new OrbExtractor(nFeatures, scaleFactor, nlevels, iniThFAST, minThFAST);

    // Construt plane segmentor
    CLOUD_CAMERA_INFO *camera_cloud;
    camera_cloud = new CLOUD_CAMERA_INFO(camera_params_cloud_.width, camera_params_cloud_.height,
                                         camera_params_cloud_.cx, camera_params_cloud_.cy,
                                         camera_params_cloud_.fx, camera_params_cloud_.fy, camera_params_cloud_.scale);
    std::string seg_setting;
    loadParam(fs, "PlaneSegmentationSetting", seg_setting, seg_setting);
    if(seg_setting.empty())
        plane_segmentor_ = new PlaneSegmentor(camera_cloud);
    else
        plane_segmentor_ = new PlaneSegmentor(seg_setting, camera_cloud);


    tracker_ = new Tracking(file_setting_);
    mapper_ = new Mapping(file_setting_);
    viewer_ = new Viewer(file_setting_);

    //
    tracker_->setMapper(mapper_);
    tracker_->setViewer(viewer_);
    mapper_->setTracker(tracker_);
    mapper_->setViewer(viewer_);
    viewer_->setTracker(tracker_);
    viewer_->setMapper(mapper_);

    //
    thread_viewer_ = new thread(&Viewer::run, viewer_);
    thread_mapping_ = new thread(&Mapping::run, mapper_);
    thread_tracking_ = new thread(&Tracking::run, tracker_);

    cout << endl << BOLDGREEN << " Done initializing slam system." << RESET << endl;
}

void System::trackRGBD(const Mat &image_rgb, const Mat &image_depth)
{
    Frame *frame = new Frame(image_rgb, image_depth, camera_params_image_, camera_params_cloud_, orb_extractor_, plane_segmentor_);

    tracker_->pushFrameInQueue(frame);
}

}
