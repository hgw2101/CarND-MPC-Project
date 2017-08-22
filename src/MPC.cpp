#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <iostream>

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 16;
double dt = 0.1;

// setup global position variables of the state variables and input controls
// so we can easily set the variables using these positions
size_t x_start = 0;
size_t y_start = x_start + N;

size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t steer_start = epsi_start + N;
size_t throttle_start = steer_start + N - 1;

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
const double ref_v = 100.0;

// this class helps us set up fg, a vector containing cost, fg[0] and constraints, fg[N] and 
// a vector containing the variables, i.e. state and accuators, vars
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

    // *1) set up cost
    // the cost is stored in the first element of fg
    fg[0] = 0;

    // add reference state related cost
    for (int t=0; t<N; t++) {
      // set cost related to cte
      fg[0] += 1000 * CppAD::pow(vars[cte_start + t], 2);
      // set cost related to epsi
      fg[0] += 1000 * CppAD::pow(vars[epsi_start + t], 2);
      // set cost for not reaching reference velocity
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
      // TODO: come up with other cost metrics
    }

    // minimize sudden steers and throttles
    for (int t=0; t<N-1; t++) {
      fg[0] += 200 * CppAD::pow(vars[steer_start + t], 2);
      fg[0] += CppAD::pow(vars[throttle_start + t], 2);
    }

    // minimize the change rate (similar to Kd in PID)
    for (int t=0; t<N-1; t++) {
      fg[0] += 400 * CppAD::pow(vars[steer_start + t] - vars[steer_start + t - 1], 2);
      fg[0] += CppAD::pow(vars[throttle_start + t] - vars[throttle_start + t - 1], 2);
      // TODO: change those to vars[steer_start + t + 1] - vars[steer_start + t], 2);
    }

    // cout<<"inside solver, after rate of change, this is fg[0]: "<<fg[0]<<endl;

    // *2) set up constraints

    // *2a) set up initial constraints separately, since we need prior values to calculate certain
    // variables
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (int t=1; t<N; t++) {
      // *2b) set the state at t+1
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // *2c) set the state at t
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // *2d) accuators at t, no need to consider t+1
      //      TODO, why not t instead t+1
      AD<double> steer_value0 = vars[steer_start + t - 1];
      AD<double> throttle_value0 = vars[throttle_start + t - 1];

      // *2e) set actual constraints using state model

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0/Lf * steer_value0 * dt); // it's - here because in the simulator, - means turning left
      fg[1 + v_start + t] = v1 - (v0 + throttle_value0 * dt);

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt); // TODO: why not 'cte1 - (cte0 + v0 * sin(epsi0) * dt);'

      AD<double> psides0 = CppAD::atan(3*coeffs[3]*x0*x0 + 2*coeffs[2]*x0 + coeffs[1]);
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0/Lf * steer_value0 * dt); //minus here because simulator steering angle is reversed
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
  size_t i; //TODO: what's this for?
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // *1) set the number of variables and constraints (includes both states and inputs)
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6 * N + 2 * (N-1); // we always have one fewer accuator than states, since we don't have accuators for the initial state
  size_t n_constraints = N * 6;

  // *2) set the initial state and accuators
  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // Initial value of the independent variables. even initial states :D
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // *3) set upper and lower bounds for variables
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // except for accuators, everything else, i.e. state variables can take on 
  // any value they like
  for (int i=0; i<steer_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // set loewr and upper bounds for accuators
  // TODO: experiment with changing those values to see if they can drive the vehicle better
  // TODO2: question, can we setup constraints on accuators in the main.cpp instead of here?
  for (int i=steer_start; i<throttle_start; i++) {
    vars_lowerbound[i] = -0.436332 * Lf;
    vars_upperbound[i] = 0.436332 * Lf;
  }

  for (int i=throttle_start; i<n_vars; i++) {
    vars_lowerbound[i] = -1;
    vars_upperbound[i] = 1;
  }

  // *4) set upper and lower bounds for variables and constraints
  // set lower and upper bounds for constraints, all should be zero except for initial values
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i=0; i<n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  
  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // cout<<"finish constraints bounds"<<endl;
  // *5) object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // cout<<"finish setting up fg_eval"<<endl;

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
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // *6) return the first value of the solution
  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  vector<double> result = {solution.x[steer_start], solution.x[throttle_start]};

  // grab the predicted x/y points, i=1 because we don't want the initial x/y, which represent the vehicle's current position
  // but can experiment with that and see how it looks
  for (int i=1; i<N; i++) {
    result.push_back(solution.x[x_start + i]);
    result.push_back(solution.x[y_start + i]);
  }

  return result;
}
