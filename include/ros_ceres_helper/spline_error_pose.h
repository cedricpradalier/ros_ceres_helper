#ifndef SPLINE_ERROR_POSE_H
#define SPLINE_ERROR_POSE_H

#include <ros_ceres_helper/ref_uniform_spline.h>
#include <ros_ceres_helper/spline_types.h>
#include <ros_ceres_helper/ceres_poses.h>

namespace cerise{ 

        struct SplineErrorP {
            double u;
            cerise::Pose y;
            double weight;

            SplineErrorP(double u, const cerise::Pose & yin, double weight=1) : u(u), y(yin), weight(weight) {
            }


            template <typename T>
                bool operator()(const T *const t1, const T *const q1, 
                        const T* const t2, const T *const q2, 
                        const T* const t3, const T *const q3, 
                        const T *const t4, const T *const q4, 
                        T* residuals) const {
                    DataDescriptorPose<T> D;
                    TRefPoseUniformSpline<T> s(TPose<T>(q1,t1),
                            TPose<T>(q2,t2),
                            TPose<T>(q3,t3),
                            TPose<T>(q4,t4));
                    cerise::TPose<T> pred,diff;
                    s.cum_evaluate(T(u),pred);
                    D.sub(y.cast<T>(),pred,diff);
                    residuals[0]=diff.T[0]/T(weight);
                    residuals[1]=diff.T[1]/T(weight);
                    residuals[2]=diff.T[2]/T(weight);
                    diff.getAngleAxis(residuals+3);
                    for (int i=0;i<3;i++) {
                        residuals[i] /= T(weight);
                    }
                    return true;
                }
        };



}


#endif // SPLINE_ERROR_POSE_H
