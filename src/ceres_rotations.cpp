#include <stdio.h>

#include "ros_ceres_helper/ceres_rotations.h"

namespace cerise{ 

    std::random_device Rotation::rd{};
    std::mt19937 Rotation::gen{Rotation::rd()};

    void Rotation::print(const char * prefix, const char * suffix, FILE * fp) const {
        printf("%s%.3f %.3f %.3f %.3f%s",
                prefix?prefix:"",
                Q[0], Q[1], Q[2], Q[3],
                suffix?suffix:"\n");
    }

    void Rotation::fromQuaternion(const geometry_msgs::Quaternion & P) {
        Q[0]=P.w;
        Q[1]=P.x;
        Q[2]=P.y;
        Q[3]=P.z;
    }

    void Rotation::toQuaternion(geometry_msgs::Quaternion & P) const {
        P.w=Q[0];
        P.x=Q[1];
        P.y=Q[2];
        P.z=Q[3];
    }

    void printRotation(const Rotation & P, const char * prefix, const char * suffix, FILE * fp) {
        P.print(prefix,suffix, fp);
    }

    void Rotation::randomize(double sigma_rot) {
        std::normal_distribution<double> dr{0.0, sigma_rot};
        double aa[3];
        aa[0] = dr(gen);
        aa[1] = dr(gen);
        aa[2] = dr(gen);
        ceres::AngleAxisToQuaternion(aa,Q);
    }

    Rotation Rotation::random(double sigma_rot) {
        Rotation out;
        out.randomize(sigma_rot);
        return out;
    }

}

