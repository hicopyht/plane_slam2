#ifndef MAPPING_H
#define MAPPING_H

#include "Frame.h"
#include "Map.h"
#include <list>
#include <mutex>

using namespace std;

namespace plane_slam2
{
class Tracking;
class Viewer;

class Mapping
{
public:
    Mapping( const std::string &setting_file );

    void run();

    bool pushFrameInQueue( Frame * frame);
    inline bool isQueueFull(){ return (queueSize() >= max_frame_queue_size_);}
    inline bool isQueueEmpty() { return (queueSize() == 0); }

    int queueSize();

    //
    void setTracker(Tracking *tracker) { tracker_ = tracker; }
    void setViewer(Viewer *viewer) { viewer_ = viewer; }

private:
    //
    Viewer* viewer_;
    Tracking* tracker_;
    //
    list<Frame*> frame_queue_;
    std::mutex mutex_frame_queue_;
    int max_frame_queue_size_;

};


}

#endif
