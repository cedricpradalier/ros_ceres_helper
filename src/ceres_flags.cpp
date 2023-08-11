#include "ros_ceres_helper/ceres_flags.h"

namespace cerise {

    DEFINE_bool(robustify_trilateration, false, "Use a robust loss function for trilateration.");

    DEFINE_string(trust_region_strategy, "levenberg_marquardt",
            "Options are: levenberg_marquardt, dogleg.");
    DEFINE_string(dogleg, "traditional_dogleg", "Options are: traditional_dogleg,"
            "subspace_dogleg.");

    DEFINE_bool(inner_iterations, false, "Use inner iterations to non-linearly "
            "refine each successful trust region step.");

    DEFINE_string(blocks_for_inner_iterations, "automatic", "Options are: "
            "automatic, cameras, points, cameras,points, points,cameras");

    DEFINE_string(linear_solver, "sparse_normal_cholesky", "Options are: "
            "sparse_schur, dense_schur, iterative_schur, sparse_normal_cholesky, "
            "dense_qr, dense_normal_cholesky and cgnr.");

    DEFINE_string(preconditioner, "jacobi", "Options are: "
            "identity, jacobi, schur_jacobi, cluster_jacobi, "
            "cluster_tridiagonal.");

    DEFINE_string(sparse_linear_algebra_library, "suite_sparse",
            "Options are: suite_sparse and cx_sparse.");

    DEFINE_string(ordering, "automatic", "Options are: automatic, user.");

    DEFINE_bool(robustify, false, "Use a robust loss function.");
    DEFINE_double(eta, 1e-2, "Default value for eta. Eta determines the "
            "accuracy of each linear solve of the truncated newton step. "
            "Changing this parameter can affect solve performance.");
    DEFINE_double(function_tolerance, 1e-6, "Default value for the function tolerance, i.e. the cost value to aim for.");

    DEFINE_int32(num_threads, 1, "Number of threads.");
    DEFINE_int32(num_iterations, 5000, "Number of iterations.");
    DEFINE_double(max_solver_time, 1e32, "Maximum solve time in seconds.");
    DEFINE_bool(nonmonotonic_steps, false, "Trust region algorithm can use"
            " nonmonotic steps.");
    DEFINE_string(solver_log, "", "File to record the solver execution to.");

}

