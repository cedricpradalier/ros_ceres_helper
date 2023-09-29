
#include "ros_ceres_helper/BasicOptimisationProblem.h"


using namespace cerise;
using namespace ceres;

void BasicOptimisationProblem::SetLinearSolver(Solver::Options* options) {
    CHECK(StringToLinearSolverType(FLAGS_linear_solver,
                &options->linear_solver_type));
    CHECK(StringToPreconditionerType(FLAGS_preconditioner,
                &options->preconditioner_type));
    CHECK(StringToSparseLinearAlgebraLibraryType(
                FLAGS_sparse_linear_algebra_library,
                &options->sparse_linear_algebra_library_type));
    // options->num_linear_solver_threads = FLAGS_num_threads;
}

void BasicOptimisationProblem::SetMinimizerOptions(Solver::Options* options) {
    options->max_num_iterations = FLAGS_num_iterations;
    options->minimizer_progress_to_stdout = true;
    options->num_threads = FLAGS_num_threads;
    options->eta = FLAGS_eta;
    options->function_tolerance = FLAGS_function_tolerance;
    options->max_solver_time_in_seconds = FLAGS_max_solver_time;
    options->use_nonmonotonic_steps = FLAGS_nonmonotonic_steps;
    CHECK(StringToTrustRegionStrategyType(FLAGS_trust_region_strategy,
                &options->trust_region_strategy_type));
    CHECK(StringToDoglegType(FLAGS_dogleg, &options->dogleg_type));
    options->use_inner_iterations = FLAGS_inner_iterations;


    // options->callbacks.push_back(&display);
    // options->update_state_every_iteration = true; 

}

void BasicOptimisationProblem::optimise() {
    Solver::Options options;
    SetMinimizerOptions(&options);
    SetLinearSolver(&options);
    this->updateOptions(&options);
    Solver::Summary summary;
#if 0
    CRSMatrix jacobian;
    double cost;
    std::vector<double> residuals;
    std::vector<double> gradient;
    problem.Evaluate(Problem::EvaluateOptions(),&cost,&residuals,&gradient,&jacobian);
    Eigen::MatrixXd ejacobian = Eigen::MatrixXd::Zero(jacobian.num_rows,jacobian.num_cols);
    for (int row=0;row<jacobian.num_rows;row++) {
        for (int icol=jacobian.rows[row];icol<jacobian.rows[row+1];icol++) {
            ejacobian(row,jacobian.cols[icol])=jacobian.values[icol];
        }
    }
    std::cout << "Jacobian\n" << ejacobian << std::endl << "Gradient\n";
    for (size_t i=0;i<gradient.size();i++) {
        printf("%f ",gradient[i]);
    }
    printf("\nResiduals\n");
    for (size_t i=0;i<residuals.size();i++) {
        printf("%f ",residuals[i]);
    }
    printf("\n");
#endif
    Solve(options, &(*problem), &summary);
    std::cout << summary.FullReport() << "\n";

}


void BasicOptimisationProblem::evaluate() {
    CRSMatrix jacobian;
    double cost;
    std::vector<double> residuals;
    std::vector<double> gradient;
    problem->Evaluate(Problem::EvaluateOptions(),&cost,&residuals,&gradient,&jacobian);
    Eigen::MatrixXd ejacobian = Eigen::MatrixXd::Zero(jacobian.num_rows,jacobian.num_cols);
    for (int row=0;row<jacobian.num_rows;row++) {
        for (int icol=jacobian.rows[row];icol<jacobian.rows[row+1];icol++) {
            ejacobian(row,jacobian.cols[icol])=jacobian.values[icol];
        }
    }
    std::cout << "Jacobian\n" << ejacobian << std::endl << "Gradient\n";
    for (size_t i=0;i<gradient.size();i++) {
        printf("%f ",gradient[i]);
    }
    printf("\nResiduals\n");
    for (size_t i=0;i<residuals.size();i++) {
        printf("%f ",residuals[i]);
    }
    printf("\n");

}
