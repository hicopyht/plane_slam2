#include "Tracking.h"
#include "Mapping.h"
#include "Viewer.h"

namespace plane_slam2
{

Tracking::Tracking( const std::string &setting_file )
{
    cout << WHITE << "Load tracking parameters." << RESET << endl;
    cv::FileStorage fs(setting_file, cv::FileStorage::READ);
    loadParam(fs, "Tracking.max_frame_queue_size", max_frame_queue_size_, 3);
}

void Tracking::run()
{
    while(1)
    {
        if( queueSize() > 0 )
        {
            Frame *frame = frame_queue_.front();
            frame_queue_.pop_front();

            viewer_->setImageRGB(frame->image_rgb_);
            viewer_->setPlanes(frame->planes_);

            //
            cout << " - Do tracking. Not implemented yet." << endl;

            //
            mapper_->pushFrameInQueue(frame);
        }

        usleep(3000);
    }
}

bool Tracking::pushFrameInQueue( Frame * frame)
{
    if( queueSize() < max_frame_queue_size_ )
    {
        frame_queue_.push_back(frame);
        return true;
    }
    else
        return false;
}

int Tracking::queueSize()
{
    unique_lock<mutex> lock(mutex_frame_queue_);
    return frame_queue_.size();
}

}
