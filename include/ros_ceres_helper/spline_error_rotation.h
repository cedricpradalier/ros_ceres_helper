#ifndef SPLINE_ERROR_ROTATION_H
#define SPLINE_ERROR_ROTATION_H

#include <ros_ceres_helper/ref_uniform_spline.h>
#include <ros_ceres_helper/spline_types.h>
#include <ros_ceres_helper/ceres_rotations.h>

namespace cerise{ 

        struct SplineErrorR {
            double u;
            cerise::Rotation y;
            double weight;

            SplineErrorR(double u, const double *yin, double weight=1) : u(u), y(yin), weight(weight) {
            }

            SplineErrorR(double u, const cerise::Rotation & yin, double weight=1) : u(u), y(yin), weight(weight) {
            }


            template <typename T>
                bool operator()(const T *const k1, const T* const k2, const T* const k3, const T *const k4,
                        T* residuals) const {
                    DataDescriptorRotation<T> D;
                    TRefRotationUniformSpline<T> s(k1,k2,k3,k4);
                    cerise::TRotation<T> pred,diff;
                    s.cum_evaluate(T(u),pred);
                    D.sub(y.cast<T>(),pred,diff);
                    diff.getAngleAxis(residuals);
                    for (int i=0;i<3;i++) {
                        residuals[i] /= T(weight);
                    }
                    return true;
                }
        };



}


#endif // SPLINE_ERROR_ROTATION_H
