#ifndef SPLINE_ERROR_QUATERNION_H
#define SPLINE_ERROR_QUATERNION_H

#include <ros_ceres_helper/ref_uniform_spline.h>
#include <ros_ceres_helper/spline_types.h>

namespace cerise{ 

        struct SplineErrorQ {
            double u;
            double q[4];
            double weight;

            SplineErrorQ(double u, const double *yin, double weight=1) : u(u), weight(weight) {
                std::copy(yin+0,yin+4,q+0);
            }


            template <typename T>
                bool operator()(const T *const k1, const T* const k2, const T* const k3, const T *const k4,
                        T* residuals) const {
                    DataDescriptorQuaternion<T> D;
                    TRefQuaternionUniformSpline<T> s(k1,k2,k3,k4);
                    T target[4]={T(q[0]),T(q[1]),T(q[2]),T(q[3])};
                    T pred[4],diff[4];
                    s.cum_evaluate(T(u),pred);
                    D.sub(target,pred,diff);
                    ceres::QuaternionToAngleAxis(diff,residuals);

                    for (int i=0;i<3;i++) {
                        residuals[i] /= T(weight);
                    }
                    return true;
                }
        };



}


#endif // SPLINE_ERROR_QUATERNION_H
