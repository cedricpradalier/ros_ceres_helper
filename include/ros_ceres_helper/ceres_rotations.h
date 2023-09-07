#ifndef CERES_ROTATIONS_H
#define CERES_ROTATIONS_H
#include <stdio.h>
#include <random>

#include <ros_ceres_helper/ceres_template_rotations.h>
#include <geometry_msgs/Quaternion.h>

namespace cerise{ 

    class Rotation : public TRotation<double> {
        protected:
            static std::random_device rd;
            static std::mt19937 gen;

        public: 
            Rotation() : TRotation<double> () {}

            template <typename DTin>
                Rotation(const DTin * const  q) : TRotation<double>(q) {}

            template <typename DTin>
                Rotation(const TRotation<DTin> & T) : TRotation<double>(T) {}

            void print(const char * prefix = NULL, const char * suffix = "\n", FILE * fp = stdout) const ;

            void randomize(double sigma_rot);

            void fromQuaternion(const geometry_msgs::Quaternion & P) ;

            void toQuaternion(geometry_msgs::Quaternion & P) const ;

            static Rotation random(double sigma_rot);
    };

    void printRotation(const Rotation & P, const char * prefix = NULL, const char * suffix = "\n", FILE * fp = stdout) ;
}

#endif // CERES_ROTATIONS_H
