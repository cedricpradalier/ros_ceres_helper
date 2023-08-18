#ifndef CERES_POSES_H
#define CERES_POSES_H
#include <stdio.h>

#include <ros_ceres_helper/ceres_template_poses.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Transform.h>
#include <tf2/LinearMath/Transform.h>

namespace cerise{ 

    class Pose : public TPose<double> {
        public: 
            Pose() : TPose<double> () {}

            template <typename DTin>
                Pose(const DTin * const  q, const DTin * const t) : TPose<double>(q,t) {}

            template <typename DTin>
                Pose(const TPose<DTin> & T) : TPose<double>(T) {}

            void print(const char * prefix = NULL, FILE * fp = stdout) const ;

            void fromPose(const geometry_msgs::Pose & P) ;

            void fromTransform(const geometry_msgs::Transform & P) ;

            void fromTF(const tf::Transform & P) ;

            void fromTF2(const tf2::Transform & P) ;

            void toPose(geometry_msgs::Pose & P) const ;

            void toTransform(geometry_msgs::Transform & P) const ;

            void toTF(tf::Transform & P) const ;

            void toTF2(tf2::Transform & P) const ;
    };

    void printPose(const Pose & P, const char * prefix = NULL, FILE * fp = stdout) ;
}

#endif // CERES_POSES_H
