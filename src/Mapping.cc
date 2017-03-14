#include "Mapping.h"
#include "Tracking.h"
#include "Viewer.h"

namespace plane_slam2
{

Mapping::Mapping( const std::string &setting_file )
{
    cout << WHITE << "Load mapping parameters." << RESET << endl;
    cv::FileStorage fs(setting_file, cv::FileStorage::READ);
    loadParam(fs, "Mapping.max_frame_queue_size", max_frame_queue_size_, 3);
}

void Mapping::run()
{
    while(1)
    {
        if( queueSize() > 0 )
        {
            Frame *frame = frame_queue_.front();
            frame_queue_.pop_front();

            cout << " - Do mapping. Not implemented yet." << endl;
        }

        usleep(3000);
    }
}

bool Mapping::pushFrameInQueue( Frame * frame)
{
    if( queueSize() < max_frame_queue_size_)
    {
        frame_queue_.push_back(frame);
        return true;
    }
    else
        return false;
}


int Mapping::queueSize()
{
    unique_lock<mutex> lock(mutex_frame_queue_);
    return frame_queue_.size();
}

}
