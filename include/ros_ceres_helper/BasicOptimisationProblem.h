#ifndef CERES_BASIC_OPTIMISATION_H
#define CERES_BASIC_OPTIMISATION_H

#include <stdlib.h>
#include <stdio.h>
#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include <Eigen/Core>

#include "ros_ceres_helper/ceres_flags.h"



namespace cerise{ 

    class BasicOptimisationProblem {
        protected:

            ceres::Problem problem;
            void SetLinearSolver(ceres::Solver::Options* options) ;

            void SetMinimizerOptions(ceres::Solver::Options* options) ;

        public:
            BasicOptimisationProblem() {
            }


            void optimise() ;

    };
}


#endif // CERES_BASIC_OPTIMISATION_H

