#ifndef TRACKING_H
#define TRACKING_H

#include "Frame.h"
#include <mutex>
#include <list>

using namespace std;

namespace plane_slam2
{

class Viewer;
class Mapping;

class Tracking
{
public:
    Tracking( const std::string &setting_file );

    void run();

    bool pushFrameInQueue( Frame * frame);
    int queueSize();

    void setMapper(Mapping *mapper) { mapper_ = mapper; }
    void setViewer(Viewer *viewer) { viewer_ = viewer; }

private:

    int max_frame_queue_size_;
    list<Frame*> frame_queue_;
    std::mutex mutex_frame_queue_;

    //
    Mapping* mapper_;
    Viewer* viewer_;
};

}

#endif
