#include <SFML/Graphics.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Dense>
#include "inverted_pendulum.h"
#include "tools.h"
#include "pid.h"
#include "lqr.h"
#include <mpc/NLMPC.hpp>
#include <nlohmann/json.hpp>
#include <iomanip> // Include for setprecision
#include <cstdlib> // for exit()
#include "dr_api.h"

using json = nlohmann::json;

bool my_setenv(const char *var, const char *value)
{
#ifdef UNIX
    return setenv(var, value, 1 /*override*/) == 0;
#else
    return SetEnvironmentVariable(var, value) == TRUE;
#endif
}

void removeLastRows_saveToCSV(const Eigen::MatrixXd &matrix, const std::string &filename, int start_idx)
{
    // Read existing content (first start_idx rows)
    std::ifstream existingFile(filename);
    std::vector<std::string> existingContent;

    if (existingFile.is_open())
    {
        std::string line;
        for (size_t i = 0; i < start_idx; ++i)
        {
            std::getline(existingFile, line);
            existingContent.push_back(line);
        }
        existingFile.close();
    }
    else
    {
        std::cerr << "Error opening file for reading, will create a new file if not exists: " << filename << std::endl;
    }

    // Open the file in truncate mode to clear existing content
    std::ofstream file(filename, std::ios::trunc);

    if (!file.is_open())
    {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }

    // Ensure high precision for floating-point numbers
    file << std::fixed << std::setprecision(15);

    // Write back the existing content (first start_idx rows)
    for (const auto& line : existingContent)
    {
        file << line << "\n";
    }

    // Write the new matrix content to the file
    for (int i = 0; i < matrix.rows(); ++i)
    {
        for (int j = 0; j < matrix.cols(); ++j)
        {
            if (j > 0)
                file << ",";
            file << matrix(i, j);
        }
        file << "\n";
    }

    file.close();
}

Eigen::MatrixXd loadFromCSV(const std::string &filename)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return Eigen::MatrixXd(0, 0);
    }

    std::vector<std::vector<double>> values;
    std::string line;

    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::vector<double> row;
        std::string value;

        while (std::getline(ss, value, ','))
        {
            row.push_back(std::stod(value));
        }

        values.push_back(row);
    }

    int rows = values.size();
    int cols = (rows > 0) ? values[0].size() : 0;

    Eigen::MatrixXd matrix(rows, cols);

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            matrix(i, j) = values[i][j];
        }
    }

    file.close();
    return matrix;
}

int main(int argc, char *argv[])
{
    /* We also test -rstats_to_stderr */
    if (!my_setenv("DYNAMORIO_OPTIONS",
                   "-stderr_mask 0xc -rstats_to_stderr "
                   "-client_lib ';;-offline'"))
        std::cerr << "failed to set env var!\n";

    // Open the parameters file
    std::ifstream file("../params.json");

    // Read the contents of the file into a string
    std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    // Parse the JSON string
    json params = json::parse(str);

    // filename to save matrix
    std::string filename = "../state_and_input_matrix.csv";
    std::string global_filename = "../../state_and_input_matrix.csv";

    // Access the parameters and create variables for them
    std::string controller = params["controller"];
    const double control_sampling_time = params["control_sampling_time"];
    int simulation_steps = params["simulation_steps"];
    const double p_0 = params["p_0"];
    const double theta_0 = params["theta_0"];
    // Constants
    double M = params["M"];
    double m = params["m"];
    double J = params["J"];
    double l = params["l"];
    double c = params["c"];
    double gamma = params["gamma"];
    double g = params["g"];

    bool record_trace = false;
    bool restart = false;
    // Create a vector to store x_t, u_t in each iteration
    std::vector<Eigen::VectorXd> xu_vector;
    Eigen::VectorXd x_0(4);
    int start_idx = 0;
    double u_start = 0;
    // If we start from an initial condition and not from csv
    if (argc == 1)
    {
        x_0 << p_0, to_radians(theta_0), 0, 0;
    }
    else if (argc >= 2)
    {
        if (argc >= 4 && std::stoi(argv[3])==1)
        {
            // If this argument is given then trace
            record_trace = true;
            global_filename = "../../../state_and_input_matrix.csv";
        }
        if (argc >= 5 && std::stoi(argv[4])==1)
        {
            // If this argument is given then restart from the last step
            restart = true;
        }
        Eigen::MatrixXd xu_matrix_tmp;
        start_idx = std::stoi(argv[1]);
        if (start_idx == 0)
        {
            x_0 << p_0, to_radians(theta_0), 0, 0;
        }
        else
        {
            xu_matrix_tmp = loadFromCSV(global_filename);
            x_0 << xu_matrix_tmp(start_idx-1, 1), xu_matrix_tmp(start_idx-1, 2),
                xu_matrix_tmp(start_idx-1, 3), xu_matrix_tmp(start_idx-1, 4);
            u_start = xu_matrix_tmp(start_idx-1, 5);
        }

        if (argc >= 3)
        {
            simulation_steps = std::stoi(argv[2]);
        }
    }
    std::cout << "Doing simulation for this number of timesteps: " << simulation_steps << std::endl;
    // Create a model with default parameters
    InvertedPendulum *ptr = new InvertedPendulum(M, m, J, l, c, gamma, x_0);

    // Design MPC controller
    // MPC Parameters
    constexpr int num_states = 4;
    constexpr int num_output = 0;
    constexpr int num_inputs = 1;
    constexpr int pred_hor = 25;
    constexpr int ctrl_hor = 5;
    constexpr int ineq_c = 0;
    constexpr int eq_c = 0;
    double sampling_time = params["control_sampling_time"];
    double input_cost_weight = params["MPC"]["input_cost_weight"];

    mpc::NLMPC<num_states, num_inputs, num_output, pred_hor, ctrl_hor, ineq_c, eq_c> optsolver;

    optsolver.setLoggerLevel(mpc::Logger::log_level::NORMAL);
    optsolver.setContinuosTimeModel(sampling_time);

    auto stateEq = [&](
                       mpc::cvec<num_states> &x_dot_,
                       const mpc::cvec<num_states> &x_,
                       const mpc::cvec<num_inputs> &u)
    {
        // Constants
        double M_t = M + m;
        double J_t = J + m * std::pow(l, 2);
        // Recover state parameters
        double x = x_(0);     // position of the base
        double theta = x_(1); // angle of the pendulum
        double vx = x_(2);    // velocity of the base
        double omega = x_(3); // angular rate of the pendulum

        // Compute common terms
        double s_t = std::sin(theta);
        double c_t = std::cos(theta);
        double o_2 = std::pow(omega, 2);
        double l_2 = std::pow(l, 2);

        // Calculate derivatives
        x_dot_(0) = vx;
        x_dot_(1) = omega;
        x_dot_(2) = (-m * l * s_t * o_2 + m * g * (m * l_2 / J_t) * s_t * c_t -
                     c * vx - (gamma / J_t) * m * l * c_t * omega + u(0)) /
                    (M_t - m * (m * l_2 / J_t) * c_t * c_t);
        x_dot_(3) =
            (-m * l_2 * s_t * c_t * o_2 + M_t * g * l * s_t - c * l * c_t * vx -
             gamma * (M_t / m) * omega + l * c_t * u(0)) /
            (J_t * (M_t / m) - m * (l * c_t) * (l * c_t));
    };

    optsolver.setStateSpaceFunction([&](
                                        mpc::cvec<num_states> &dx,
                                        const mpc::cvec<num_states> &x,
                                        const mpc::cvec<num_inputs> &u,
                                        const unsigned int &)
                                    { stateEq(dx, x, u); });
    optsolver.setObjectiveFunction([&](
                                       const mpc::mat<pred_hor + 1, num_states> &x,
                                       const mpc::mat<pred_hor + 1, num_output> &,
                                       const mpc::mat<pred_hor + 1, num_inputs> &u,
                                       double)
                                   { 
                                    std::vector<double> w = params["MPC"]["state_cost_weights"].get<std::vector<double>>();
                                    Eigen::VectorXd state_cost_weights = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(w.data(), w.size()); // create a weight vector
                                    return (x * state_cost_weights.asDiagonal()).array().square().sum() + u.array().square().sum() * input_cost_weight; });

    // MPC
    mpc::cvec<num_states> modelX, modeldX;
    modelX.resize(num_states);
    modeldX.resize(num_states);

    modelX(0) = p_0;
    modelX(1) = to_radians(theta_0);
    modelX(2) = 0.0;
    modelX(3) = 0.0;

    auto r = optsolver.getLastResult();

    //  Design PID controller
    const double kp = params["PID"]["kp"];
    const double ki = params["PID"]["ki"];
    const double kd = params["PID"]["kd"];
    PID *c_ptr = new PID();
    c_ptr->Init(kp, ki, kd);

    // Design LQR controller
    LQR optimal;
    ptr->Linearize();
    optimal.A_ = ptr->A_;
    optimal.B_ = ptr->B_;
    optimal.Q_ = Eigen::MatrixXd::Identity(4, 4);
    optimal.Q_(0, 0) = 10;
    optimal.R_ = Eigen::MatrixXd::Identity(1, 1);
    optimal.Compute();

    // Create a clock to run the simulation
    sf::Clock clock;
    float time = 0;
    int timestep = 0;
    double u = u_start;

    // Get last state
    Eigen::VectorXd x = ptr->GetState();

    // Record state and input x_t,u_t
    Eigen::VectorXd xu(x.size() + 1);
    xu << x, u;
    // Add the zero input to the beginning
    if(start_idx==0)
        xu_vector.push_back(xu);
    // Apply input to the system according to last timestep measurements
    ptr->Update(time, u);

    // Simulation loop
    while (timestep < simulation_steps)
    {
        // Control calculations starts here
        if (record_trace)
        {
            dr_app_setup_and_start();
        }
        if (restart)
        {
            u=u_start;
        }
        else if (controller == "PID")
        {
            double angle = x(1);
            double error = 0.0F - angle;
            c_ptr->UpdateError(time, error);
            u = c_ptr->TotalError();
        }
        else if (controller == "LQR")
        {
            u = optimal.Control(x)(0, 0);
        }
        else if (controller == "MPC")
        {
            modelX = x;
            r = optsolver.step(modelX, r.cmd);
            u = r.cmd(0);
        }
        if (record_trace)
        {
            dr_app_stop_and_cleanup();
            exit(0);
        }
        // Control calculations ends here

        // Update the simulation
        time += control_sampling_time;
        timestep++;
        std::cout << "Timestep: " << timestep << std::endl;
        // Apply input to the system according to last timestep measurements
        ptr->Update(time, u);
        x = ptr->GetState();

        // Record state and input x_t,u_t
        Eigen::VectorXd xu(x.size() + 1);
        xu << x, u;
        xu_vector.push_back(xu);
    }
    // Record the last state
    // Get state
    // Eigen::VectorXd x = ptr->GetState();
    // Eigen::VectorXd xu(x.size() + 1);
    // xu << x, 0.0;
    // xu_vector.push_back(xu);
    Eigen::MatrixXd xu_matrix(xu_vector.size(), xu_vector[0].size() + 1); // +1 for the timestep column

    for (size_t i = 0; i < xu_vector.size(); ++i)
    {
        xu_matrix(i, 0) = static_cast<double>(start_idx + i); // Writing row index at the first column
        xu_matrix.block(i, 1, 1, xu_vector[0].size()) = xu_vector[i].transpose();
    }
    // Print the resulting matrix
    std::cout << "Resulting matrix:\n"
              << xu_matrix << std::endl;

    // Save to CSV
    removeLastRows_saveToCSV(xu_matrix, filename, start_idx);
    removeLastRows_saveToCSV(xu_matrix, global_filename, start_idx);
    return 0;
}
