#ifndef CERES_POSES_H
#define CERES_POSES_H
#include <stdio.h>
#include <random>

#include <ros_ceres_helper/ceres_template_poses.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Transform.h>
#include <tf2/LinearMath/Transform.h>

namespace cerise{ 

    class Pose : public TPose<double> {
        protected:
            static std::random_device rd;
            static std::mt19937 gen;

        public: 
            Pose() : TPose<double> () {}

            template <typename DTin>
                Pose(const DTin * const  q, const DTin * const t) : TPose<double>(q,t) {}

            template <typename DTin>
                Pose(const TPose<DTin> & T) : TPose<double>(T) {}

            void print(const char * prefix = NULL, const char * suffix = "\n", FILE * fp = stdout) const ;

            void randomize(double sigma_trans, double sigma_rot);

            void fromPose(const geometry_msgs::Pose & P) ;

            void fromTransform(const geometry_msgs::Transform & P) ;

            void fromTF(const tf::Transform & P) ;

            void fromTF2(const tf2::Transform & P) ;

            void toPose(geometry_msgs::Pose & P) const ;

            void toTransform(geometry_msgs::Transform & P) const ;

            void toTF(tf::Transform & P) const ;

            void toTF2(tf2::Transform & P) const ;

            static Pose random(double sigma_trans, double sigma_rot);
    };

    void printPose(const Pose & P, const char * prefix = NULL, const char * suffix = "\n", FILE * fp = stdout) ;
}

#endif // CERES_POSES_H
