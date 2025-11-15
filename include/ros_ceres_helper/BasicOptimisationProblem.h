#ifndef CERES_BASIC_OPTIMISATION_H
#define CERES_BASIC_OPTIMISATION_H

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include <Eigen/Core>

#include "ros_ceres_helper/ceres_flags.h"



namespace cerise{ 

    class BasicOptimisationProblem {
        protected:

            std::shared_ptr<ceres::Problem> problem;
            ceres::Solver::Summary summary;

            void SetLinearSolver(ceres::Solver::Options* options) ;

            void SetMinimizerOptions(ceres::Solver::Options* options) ;

            virtual void updateOptions(ceres::Solver::Options* /*options*/) {}

        public:
            BasicOptimisationProblem() {
                problem.reset(new ceres::Problem);
            }

            virtual ~BasicOptimisationProblem() {}

            void reset() {
                problem.reset(new ceres::Problem);
            }

            bool optimise() ;
            void evaluate(std::ostream & s = std::cout) ;

            const ceres::Solver::Summary & getSummary() const {
                return summary;
            }
    };
}


#endif // CERES_BASIC_OPTIMISATION_H

