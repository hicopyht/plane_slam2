#include "Utils.h"

namespace plane_slam2
{
/*----------------------------------------------------------------------------------------------------------*/
//
void loadParam(cv::FileStorage &fs, const std::string &name, bool &var, bool default_value)
{
    std::string bstr = fs[name];
    if(bstr.empty()){
        var = default_value;
        cout << WHITE << "  - " << BLUE << name << WHITE << " default " << YELLOW << (default_value?"true":"false") << RESET << endl;
    }else{
        if(!bstr.compare("true")){
            var = true;
            cout << WHITE << "  - " << BLUE << name << WHITE << " load " << CYAN << "true" << RESET << endl;
        }else{
            var = false;
            cout << WHITE << "  - " << BLUE << name << WHITE << " load " << CYAN << "false" << RESET << endl;
        }
    }
}
//
void loadParam(cv::FileStorage &fs, const std::string &name, int &var, int default_value)
{
    cv::FileNode fn = fs[name];
    if(fn.empty()){
        var = default_value;
        cout << WHITE << "  - " << BLUE << name << WHITE << " default " << YELLOW << default_value << RESET << endl;
    }else{
        var = (int)fn;
        cout << WHITE << "  - " << BLUE << name << WHITE << " load " << CYAN << var << RESET << endl;
    }
}
//
void loadParam(cv::FileStorage &fs, const std::string &name, float &var, float default_value)
{
    cv::FileNode fn = fs[name];
    if(fn.empty()){
        var = default_value;
        cout << WHITE << "  - " << BLUE << name << WHITE << " default " << YELLOW << default_value << RESET << endl;
    }else{
        var = (float)fn;
        cout << WHITE << "  - " << BLUE << name << WHITE << " load " << CYAN << var << RESET << endl;
    }
}
//
void loadParam(cv::FileStorage &fs, const std::string &name, double &var, double default_value)
{
    cv::FileNode fn = fs[name];
    if(fn.empty()){
        var = default_value;
        cout << WHITE << "  - " << BLUE << name << WHITE << " default " << YELLOW << default_value << RESET << endl;
    }else{
        var = (double)fn;
        cout << WHITE << "  - " << BLUE << name << WHITE << " load " << CYAN << var << RESET << endl;
    }
}
//
void loadParam(cv::FileStorage &fs, const std::string &name, std::string &var, std::string default_value)
{
    cv::FileNode fn = fs[name];
    if(fn.empty()){
        var = default_value;
        cout << WHITE << "  - " << BLUE << name << WHITE << " default " << YELLOW << default_value << RESET << endl;
    }else{
        var = (string)fn;
        cout << WHITE << "  - " << BLUE << name << WHITE << " load " << CYAN << var << RESET << endl;
    }
}
/*----------------------------------------------------------------------------------------------------------*/
void randomRGBColor(cv::RNG &rng, RGBValue &color)
{
    color.Red = rng.uniform(0, 255);
    color.Green = rng.uniform(0, 255);
    color.Blue = rng.uniform(0, 255);
    color.Alpha = 255;
}

void getPointCloudFromIndices(const PointCloudTypePtr &input,
                              const std::vector<int> &indices,
                              PointCloudTypePtr &output)
{
    output->clear();
    std::vector<int>::const_iterator indices_end = indices.end();
    for(std::vector<int>::const_iterator it = indices.begin(); it != indices_end; it++)
    {
        output->points.push_back(input->points[*it]);
    }
    output->is_dense = true;
    output->height = 1;
    output->width = output->points.size();
}

void getPointCloudFromIndices(const PointCloudTypePtr &input,
                              const std::vector<int> &indices,
                              const Eigen::Vector4f &coefficients,
                              PointCloudTypePtr &output)
{
    output->header = input->header;
    output->is_dense = input->is_dense;
    //
    Eigen::Vector4f mc(coefficients[0], coefficients[1], coefficients[2], 0);
    mc.normalize();
    Eigen::Vector4f tmp_mc = coefficients;
    tmp_mc[0] = mc[0];
    tmp_mc[1] = mc[1];
    tmp_mc[2] = mc[2];

    // Allocate enough space and copy the basics
    output->points.resize (indices.size ());
    output->width    = static_cast<uint32_t> (indices.size ());
    output->height   = 1;

    typedef typename pcl::traits::fieldList<PointType>::type FieldList;
    // Iterate over each point
    for (size_t i = 0; i < indices.size (); ++i)
        // Iterate over each dimension
        pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointType, PointType> (input->points[indices[i]], output->points[i]));

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < indices.size (); ++i)
    {
        // Calculate the distance from the point to the plane
        Eigen::Vector4f p (input->points[indices[i]].x,
                            input->points[indices[i]].y,
                            input->points[indices[i]].z,
                            1);
        // use normalized coefficients to calculate the scalar projection
        float distance_to_plane = tmp_mc.dot (p);

        pcl::Vector4fMap pp = output->points[i].getVector4fMap ();
        pp.matrix() = p - mc * distance_to_plane;        // mc[3] = 0, therefore the 3rd coordinate is safe
    }
}

}
