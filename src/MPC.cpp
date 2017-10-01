https://github.com/jeremy-shannon/CarND-MPC-Project/edit/master/src/MPC.cpp
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10; //0
double dt = 0.1; //0

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double ref_v = 70;
size_t x_ini = 0;
size_t y_ini = x_ini + N;
size_t psi_ini = y_ini + N;
size_t v_ini = psi_ini + N;
size_t cte_ini = v_ini + N;
size_t epsi_ini = cte_ini + N;
size_t delta_ini = epsi_ini + N;
size_t a_ini = delta_ini + N - 1;

class FG_eval {
    public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
    
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) {
        // TODO: implement MPC
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.
        
        // The cost is stored is the first element of `fg`, any additions to the cost should be added to `fg[0]`
        fg[0] = 0;
        
        // Reference State Cost
        for (int i = 0; i < N; i++) {
            fg[0] += 3000 * CppAD::pow(vars[cte_ini + i], 2);
            fg[0] += 3000 * CppAD::pow(vars[epsi_ini + i], 2);
            fg[0] += CppAD::pow(vars[v_ini + i] - ref_v, 2);
        }
        
        for (int i = 0; i < N - 1; i++) {
            fg[0] += 5 * CppAD::pow(vars[delta_ini + i], 2);
            fg[0] += 5 * CppAD::pow(vars[a_ini + i], 2);
            // Try adding penalty for speed + steer
            fg[0] += 700 * CppAD::pow(vars[delta_ini + i] * vars[v_ini + i], 2);
        }
        
        for (int i = 0; i < N - 2; i++) {
            fg[0] += 200 * CppAD::pow(vars[delta_ini + i + 1] - vars[delta_ini + i], 2);
            fg[0] += 10 * CppAD::pow(vars[a_ini + i + 1] - vars[a_ini + i], 2);
        }
        
        // Initial constraints
        // Add 1 to each of the starting indices due to cost being located at index 0 of `fg`
        // This bumps up the position of all the other values.
        fg[x_ini + 1] = vars[x_ini];
        fg[y_ini + 1] = vars[y_ini];
        fg[psi_ini + 1] = vars[psi_ini];
        fg[v_ini + 1] = vars[v_ini];
        fg[cte_ini + 1] = vars[cte_ini];
        fg[epsi_ini + 1] = vars[epsi_ini];
        
        // The rest of the constraints
        for (int t = 1; t < N; t++) {
            AD<double> x0 = vars[x_ini + t - 1];
            AD<double> y0 = vars[y_ini + t - 1];
            AD<double> psi0 = vars[psi_ini + t - 1];
            AD<double> v0 = vars[v_ini + t - 1];
            AD<double> cte0 = vars[cte_ini + t - 1];
            AD<double> epsi0 = vars[epsi_ini + t - 1];
            
            AD<double> x1 = vars[x_ini + t];
            AD<double> y1 = vars[y_ini + t];
            AD<double> psi1 = vars[psi_ini + t];
            AD<double> v1 = vars[v_ini + t];
            AD<double> cte1 = vars[cte_ini + t];
            AD<double> epsi1 = vars[epsi_ini + t];
            
            // Use previous actuations (to account for latency)
            AD<double> delta = vars[delta_ini + t - 1];
            AD<double> a = vars[a_ini + t - 1];
            
            if (t > 1) {
                delta = vars[delta_ini + t - 2];
                a = vars[a_ini + t - 2];
            }
            
            AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
            AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
            
            // Setup the rest of the model constraints
            fg[x_ini + t + 1] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[y_ini + t + 1] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[psi_ini + t + 1] = psi1 - (psi0 - v0/Lf * delta * dt);
            fg[v_ini + t + 1] = v1 - (v0 + a * dt);
            fg[cte_ini + t + 1] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[epsi_ini + t + 1] = epsi1 - ((psi0 - psides0) - v0 / Lf * delta * dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    
    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];
    
    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9
    size_t n_vars = N * 6 + (N - 1) * 2; //0
    // TODO: Set the number of constraints
    size_t n_constraints = N * 6; //0
    
    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }
    
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // TODO: Set lower and upper limits for variables.
    
    // Set the initial variable values
    vars[x_ini] = x;
    vars[y_ini] = y;
    vars[psi_ini] = psi;
    vars[v_ini] = v;
    vars[cte_ini] = cte;
    vars[epsi_ini] = epsi;
    
    // Set all non-actuators upper and lowerlimits to the max negative and positive values.
    for (int i = 0; i < delta_ini; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    
    // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
    for (int i = delta_ini; i < a_ini; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }
    
    // Acceleration / decceleration upper and lower limits.
    for (int i = a_ini; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }
    
    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    
    constraints_lowerbound[x_ini] = x;
    constraints_lowerbound[y_ini] = y;
    constraints_lowerbound[psi_ini] = psi;
    constraints_lowerbound[v_ini] = v;
    constraints_lowerbound[cte_ini] = cte;
    constraints_lowerbound[epsi_ini] = epsi;
    
    constraints_upperbound[x_ini] = x;
    constraints_upperbound[y_ini] = y;
    constraints_upperbound[psi_ini] = psi;
    constraints_upperbound[v_ini] = v;
    constraints_upperbound[cte_ini] = cte;
    constraints_upperbound[epsi_ini] = epsi;
    
    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);
    
    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";
    
    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;
    
    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval> (options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);
    
    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    
    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;
    
    // TODO: Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.
    //return {};
    vector<double> result;
    
    result.push_back(solution.x[delta_ini]);
    result.push_back(solution.x[a_ini]);
    
    for (int i = 0; i < N - 1; i++) {
        result.push_back(solution.x[x_ini + i + 1]);
        result.push_back(solution.x[y_ini + i + 1]);
    }
    
    return result;
}