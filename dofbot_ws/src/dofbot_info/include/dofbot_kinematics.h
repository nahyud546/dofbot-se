#ifndef __DOFBOT_KINEMATICS_H__
#define __DOFBOT_KINEMATICS_H__

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <vector>
#include <string>
#include <iostream>

using namespace KDL;
using namespace std;

namespace dofbot_kinematics {
    bool getFK(const string &urdf_file, vector<double> &joints, vector<double> &currentPos);
    bool getIK(const string &urdf_file, vector<double> &targetXYZ, 
               vector<double> &targetRPY, vector<double> &outjoints);
}

class DOFBOT_Pro {
public:
    void readme(void);
    bool dofbot_getFK(const char *urdf_file, vector<double> &joints, vector<double> &currentPos);
    bool dofbot_getIK(const char *urdf_file, vector<double> &targetXYZ, 
                      vector<double> &targetRPY, vector<double> &outjoints);
};

#endif