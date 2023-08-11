#ifndef CERES_HELPER_FLAGS_H
#define CERES_HELPER_FLAGS_H

#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

namespace cerise {

    DECLARE_bool(robustify_trilateration);
    DECLARE_string(trust_region_strategy);
    DECLARE_string(dogleg);
    DECLARE_bool(inner_iterations);
    DECLARE_string(blocks_for_inner_iterations);
    DECLARE_string(linear_solver);
    DECLARE_string(preconditioner);
    DECLARE_string(sparse_linear_algebra_library);
    DECLARE_string(ordering);
    DECLARE_bool(robustify);
    DECLARE_double(eta);
    DECLARE_double(function_tolerance);
    DECLARE_int32(num_threads);
    DECLARE_int32(num_iterations);
    DECLARE_double(max_solver_time);
    DECLARE_bool(nonmonotonic_steps);
    DECLARE_string(solver_log);


}

#endif // CERES_HELPER_FLAGS_H
