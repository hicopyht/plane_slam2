#ifndef SYSTEM_H
#define SYSTEM_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "Frame.h"
#include "Tracking.h"
#include "Mapping.h"
#include "Viewer.h"
#include "OrbExtractor.h"
#include "PlaneSegmentor.h"

using namespace std;
using namespace cv;

namespace plane_slam2
{

class System
{
public:
    System(const std::string &file_setting);

    void trackRGBD(const Mat &image_rgb, const Mat &image_depth);

    std::string &getSettingFile(){ return file_setting_; }

private:
    std::string file_setting_;
    CAMERA_PARAMETERS camera_params_image_;
    CAMERA_PARAMETERS camera_params_cloud_;

    OrbExtractor *orb_extractor_;
    PlaneSegmentor *plane_segmentor_;

    Tracking * tracker_;
    Mapping * mapper_;
    Viewer * viewer_;

    std::thread* thread_tracking_;
    std::thread* thread_mapping_;
    std::thread* thread_viewer_;

};

}

#endif
