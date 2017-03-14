#include "MapPlane.h"
#include <pangolin/pangolin.h>

namespace plane_slam2
{

MapPlane::MapPlane()
    : id_(-1)
    , coefficients_(0.0f, 0.0f, 0.0f, 1.0f)
    , centroid_()
    , cloud_(new PointCloudType)
    , boundary_(new PointCloudType)
    , hull_(new PointCloudType)
    , render_point_size_(1.0)
{
}

void MapPlane::render()
{
    glColor4i(color_.Red, color_.Green, color_.Blue, color_.Alpha);
    // Points
    glPointSize(render_point_size_);
    glBegin(GL_POINTS);
    PointCloudType::iterator cloud_end = cloud_->end();
    for(PointCloudType::iterator it = cloud_->begin(); it != cloud_end; it++)
    {
        glVertex3f(it->x, it->y, it->z);
    }
    glEnd();

    // Boundary

}

}
