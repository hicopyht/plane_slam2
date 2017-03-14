#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "MapPlane.h"
#include <stdlib.h>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;

namespace plane_slam2
{

class Map
{
public:
    Map();

private:

};

std::string timeToStr();

}

#endif
