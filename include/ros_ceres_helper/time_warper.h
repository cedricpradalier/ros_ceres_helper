#ifndef SPLINE_TIME_WARPER_H
#define SPLINE_TIME_WARPER_H

#include <algorithm>


namespace cerise{ 


    struct TimeWarper {
        double t0;
        double delta;
        size_t n_knots;
        TimeWarper(double t0, double delta, size_t n_knots) :
            t0(t0), delta(delta), n_knots(n_knots) {}

        // Scaling factor for derivative of u(t) 
        double dudt_scale() const {
            return 1/delta;
        }

        // Scaling factor for 2nd derivative of u(t)
        double d2udt2_scale() const {
            return 1/(delta*delta);
        }

        template <typename IT> 
            std::pair<size_t, IT> operator()(IT t) const {
                int i = floor(t);
                i = std::min<int>(std::max<int>(i,1),n_knots-3);
                IT u = t - IT(i);
                return std::pair<size_t, IT>(i,u);
            }

        template <typename IT> 
            size_t iknot(IT t) const {
                int i = floor(t);
                i = std::min<int>(std::max<int>(i,1),n_knots-3);
                return i;
            }

        double knot(size_t i) const {
            return t0 + std::min(i,n_knots-1)*delta;
        }

        double min() const {
            return t0;
        }
        double max() const {
            return t0 + n_knots*delta;
        }
    };


}


#endif // SPLINE_TIME_WARPER_H
