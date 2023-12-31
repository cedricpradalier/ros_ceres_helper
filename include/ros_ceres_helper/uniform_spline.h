#ifndef CERES_UNIFORM_SPLINES_H
#define CERES_UNIFORM_SPLINES_H

#include <ros_ceres_helper/time_warper.h>
#include <vector>


namespace cerise{ 

    template <class SplineType> 
        struct UniformSpline {
            std::vector<typename SplineType::VarType> knots;
            TimeWarper warper;
            std::vector<bool> used;

            UniformSpline(double tmin, double tmax, size_t n_knots) : knots(n_knots), warper(tmin, (tmax-tmin)/(n_knots-1) , n_knots), used(n_knots,false) {}
            UniformSpline(double tmin, size_t n_knots, double delta) : knots(n_knots), warper(tmin, delta , n_knots), used(n_knots,false) {}
            UniformSpline(TimeWarper warper) : knots(warper.n_knots), warper(warper), used(warper.n_knots,false) {}
            
            template <class IT>
                void import(IT begin) {
                    std::copy(begin,begin+knots.size(),knots.begin());
                }

            void resetUsed() {
                for(size_t i=0;i<used.size();i++) {
                    used[i]=false;
                }
            }

            void recordUsed(size_t iknots) {
                if ((iknots>=1) && (iknots<used.size()-2)) {
                    used[iknots-1]=true;
                    used[iknots]=true;
                    used[iknots+1]=true;
                    used[iknots+2]=true;
                }
            }

            bool evaluate(typename SplineType::DataType t, typename SplineType::WritableType fu) const {
                std::pair<size_t,typename SplineType::DataType> iu = warper(t);
                // printf("S Evaluate t=%f, i=%d, u=%f\n",t,int(iu.first),iu.second);
                SplineType s(knots[iu.first-1],knots[iu.first],knots[iu.first+1],knots[iu.first+2]);
                return s.evaluate(iu.second, fu);
            }

            bool evaluate(typename SplineType::DataType t, typename SplineType::WritableType fu,
                    typename SplineType::LogWritableType dfudt,  typename SplineType::LogWritableType d2fudt2) const {
                std::pair<size_t,typename SplineType::DataType> iu = warper(t);
                // printf("S Evaluate t=%f, i=%d, u=%f\n",t,int(iu.first),iu.second);
                SplineType s(knots[iu.first-1],knots[iu.first],knots[iu.first+1],knots[iu.first+2]);
                if (!s.evaluate(iu.second, fu, dfudt, d2fudt2)) {
                    return false;
                }
                dfudt *= warper.dudt_scale();
                d2fudt2 *= warper.d2udt2_scale();
                return true;
            }

            bool cum_evaluate(typename SplineType::DataType t, typename SplineType::WritableType fu) const {
                std::pair<size_t,typename SplineType::DataType> iu = warper(t);
                SplineType s(knots[iu.first-1],knots[iu.first],knots[iu.first+1],knots[iu.first+2]);
                return s.cum_evaluate(iu.second, fu);
            }


        };


}


#endif // CERES_UNIFORM_SPLINES_H
