#ifndef SPLINE_ERROR_H
#define SPLINE_ERROR_H

#include <ros_ceres_helper/ref_uniform_spline.h>
#include <ros_ceres_helper/spline_types.h>


namespace cerise{ 

    template <int dim> 
        struct SplineError {
            double u;
            double y[dim];
            double weight;
            bool cum_eval;

            SplineError(double u, const double *yin, double weight=1, bool cum_eval=false) : u(u), weight(weight), cum_eval(cum_eval) {
                std::copy(yin+0,yin+dim,y+0);
            }


            template <typename T>
                bool operator()(const T *const k1, const T* const k2, const T* const k3, const T *const k4,
                        T* residuals) const {
                    TRefPtrUniformSpline<T,dim> s(k1,k2,k3,k4);
                    T pred[dim];
                    if (cum_eval) {
                        s.cum_evaluate(T(u),pred);
                    } else {
                        s.evaluate(T(u),pred);
                    }

                    for (int i=0;i<dim;i++) {
                        residuals[i] = (pred[i] - T(y[i]))/T(weight);
                    }
                    return true;
                }
        };

}


#endif // SPLINE_ERROR_H
