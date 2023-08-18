#include <stdio.h>

#include "ros_ceres_helper/ceres_poses.h"

namespace cerise{ 

    void Pose::print(const char * prefix, FILE * fp) const {
        printf("%s%.3f %.3f %.3f | %.3f %.3f %.3f %.3f\n",
                prefix?prefix:"",
                T[0], T[1], T[2], Q[0], Q[1], Q[2], Q[3]);
    }

    void Pose::fromPose(const geometry_msgs::Pose & P) {
        T[0]=P.position.x;
        T[1]=P.position.y;
        T[2]=P.position.z;
        Q[0]=P.orientation.w;
        Q[1]=P.orientation.x;
        Q[2]=P.orientation.y;
        Q[3]=P.orientation.z;
    }

    void Pose::fromTransform(const geometry_msgs::Transform & P) {
        T[0]=P.translation.x;
        T[1]=P.translation.y;
        T[2]=P.translation.z;
        Q[0]=P.rotation.w;
        Q[1]=P.rotation.x;
        Q[2]=P.rotation.y;
        Q[3]=P.rotation.z;
    }

    void Pose::fromTF(const tf::Transform & P) {
        tf::Vector3 t = P.getOrigin();
        tf::Quaternion q = P.getRotation();
        T[0] = t.x();
        T[1] = t.y();
        T[2] = t.z();
        Q[0] = q.w();
        Q[1] = q.x();
        Q[2] = q.y();
        Q[3] = q.z();
    }

    void Pose::fromTF2(const tf2::Transform & P) {
        tf2::Vector3 t = P.getOrigin();
        tf2::Quaternion q = P.getRotation();
        T[0] = t.x();
        T[1] = t.y();
        T[2] = t.z();
        Q[0] = q.w();
        Q[1] = q.x();
        Q[2] = q.y();
        Q[3] = q.z();
    }

    void Pose::toPose(geometry_msgs::Pose & P) const {
        P.position.x=T[0];
        P.position.y=T[1];
        P.position.z=T[2];
        P.orientation.w=Q[0];
        P.orientation.x=Q[1];
        P.orientation.y=Q[2];
        P.orientation.z=Q[3];
    }

    void Pose::toTransform(geometry_msgs::Transform & P) const {
        P.translation.x=T[0];
        P.translation.y=T[1];
        P.translation.z=T[2];
        P.rotation.w=Q[0];
        P.rotation.x=Q[1];
        P.rotation.y=Q[2];
        P.rotation.z=Q[3];
    }

    void Pose::toTF(tf::Transform & P) const {
        P.setOrigin(tf::Vector3(T[0],T[1],T[2]));
        P.setRotation(tf::Quaternion(Q[1],Q[2],Q[3],Q[0]));
    }

    void Pose::toTF2(tf2::Transform & P) const {
        P.setOrigin(tf2::Vector3(T[0],T[1],T[2]));
        P.setRotation(tf2::Quaternion(Q[1],Q[2],Q[3],Q[0]));
    }

    void printPose(const Pose & P, const char * prefix, FILE * fp) {
        P.print(prefix,fp);
    }
}

