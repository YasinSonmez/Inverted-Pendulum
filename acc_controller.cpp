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
//1 #include <mpc/NLMPC.hpp>
#include <mpc/LMPC.hpp>
#include <nlohmann/json.hpp>
#include <iomanip> // Include for setprecision
#include <cstdlib> // for exit()
#include "dr_api.h"

using json = nlohmann::json;
using namespace mpc;

// Define a macro to print a value 
#define PRINT(x) std::cout << x << std::endl;

#ifdef PREDICTION_HORIZON
#else
  #define PREDICTION_HORIZON 5
#endif
#ifdef CONTROL_HORIZON
#else
  #define CONTROL_HORIZON 3
#endif

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    // From https://stackoverflow.com/a/26221725/6651650
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size_s <= 0 ){ 
      throw std::runtime_error( "string_format(): Error during formatting." ); 
    }
    auto size = static_cast<size_t>( size_s );
    std::unique_ptr<char[]> buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

template<int rows, int cols>
void loadMatrixValuesFromJson(mat<rows, cols>& mat_out, json json_data, std::string key) {
      if (json_data[key].size() != rows*cols)
      {
        throw std::invalid_argument(string_format(
              "loadMatrixValuesFromJson(): The number of entries (%d) in json_data[\"%s\"] does not match the expected number of entries in mat_out (%dx%d).", 
              json_data[key].size(),  key.c_str(), rows, cols));
      }

      int i = 0;
      for (auto& element : json_data[key]) {
        mat_out[i] = element;
        i++;
      }
}

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

    // filename to save matrix
    std::string filename = "../state_and_input_matrix.csv";
    std::string global_filename = "../../state_and_input_matrix.csv";

    bool record_trace = false;
    bool restart = false;
    // Create a vector to store x_t, u_t in each iteration
    std::vector<Eigen::VectorXd> xu_vector;
    int start_idx = 0;
    int simulation_steps = 0;
    Eigen::VectorXd x_0(5);
    Eigen::VectorXd u_start = Eigen::VectorXd::Constant(1, 0);
    // If we start from an initial condition and not from csv
    if (argc >= 2)
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
        if (argc >= 4 && std::stoi(argv[3])==1)
        {
            // If this argument is 1 then trace
            record_trace = true;
            global_filename = "../../../state_and_input_matrix.csv";
        }
        Eigen::MatrixXd xu_matrix_tmp;
        start_idx = std::stoi(argv[1]);
        if (start_idx != 0)
        {
            xu_matrix_tmp = loadFromCSV(global_filename);
            x_0 << xu_matrix_tmp(start_idx-1, 1), xu_matrix_tmp(start_idx-1, 2),
                xu_matrix_tmp(start_idx-1, 3), xu_matrix_tmp(start_idx-1, 4), xu_matrix_tmp(start_idx-1, 5);
            u_start(0) = xu_matrix_tmp(start_idx-1, 6); // length zero vector
        }
        if (argc >= 3)
        {
            simulation_steps = std::stoi(argv[2]);
        }
    }
    std::cout << "Doing simulation for this number of timesteps: " << simulation_steps << std::endl;

    // Vector sizes.
    const int Tnx = 5;  // State dimension
    const int Tnu = 1;  // Control dimension
    const int Tndu = 1; // Exogenous control (disturbance) dimension
    const int Tny = 2;  // Output dimension

    // MPC options.
    const int prediction_horizon = PREDICTION_HORIZON;
    const int control_horizon = CONTROL_HORIZON;

    std::ifstream config_json_file("../params.json");
    json json_data = json::parse(config_json_file);
    config_json_file.close();

    // Discretization Options.
    double sample_time = json_data["control_sampling_time"];

    // Set whether the evolution of the dyanmics are computed extenally 
    // (with Python) or are done within this process, using libmpc.
    bool use_external_dynamics_computation = json_data["use_external_dynamics_computation"];

    PRINT("Creating Matrices");
    // Continuous-time Matrices for \dot{x} = Ax + bu. 
    // Model is the HCW equations for relative satellite motion.
    mat<Tnx, Tnx> Ac;
    mat<Tnx, Tnu> Bc;
    mat<Tny, Tnx> Cd;
    
    double lead_car_input = json_data["lead_car_input"];
    double tau = json_data["tau"];
    // Define continuous-time state matrix A_c
    Ac << 0,      1, 0,  0,      0, // v1
          0, -1/tau, 0,  0,      0, // a1
          1,      0, 0, -1,      0, // d2
          0,      0, 0,  0,      1, // v2
          0,      0, 0,  0, -1/tau; // a2

    // Define continuous-time input matrix B_c
    Bc <<    0,
            0,
            0, 
            0, 
          -1/tau;

    // State to output matrix
    Cd << 0, 0, 1, 0, 0, 
          1, 0, 0,-1, 0;

    // Set input disturbance matrix.
    mat<Tnx, Tndu> Bc_disturbance;
    mat<Tnx, Tndu> Bd_disturbance;
    Bc_disturbance << 0, 1/tau, 0, 0, 0;

    mat<Tnx, Tnx> Ad;
    mat<Tnx, Tnu> Bd;
    // Compute the discretization of Ac and Bc, storing them in Ad and Bd.
    discretization<Tnx, Tnu, Tndu>(Ac, Bc, Bc_disturbance, sample_time, Ad, Bd, Bd_disturbance);

    PRINT("Finished creating Matrices");

    PRINT("Creating LMPC object...");
    LMPC<Tnx, Tnu, Tndu, Tny, prediction_horizon, control_horizon> lmpc;
    PRINT("Finished creating LMPC object.");


    LParameters params;
    // ADMM relaxation parameter (see https://osqp.org/docs/solver/index.html#algorithm)
    params.alpha = 1.6;
    // ADMM rho step (see https://osqp.org/docs/solver/index.html#algorithm)
    params.rho = 1e-6;
    params.adaptive_rho = true;
    // Relative tolerance
    params.eps_rel = json_data["osqp_rel_tolerance"];
    // Absolute tolerance
    params.eps_abs = json_data["osqp_abs_tolerance"];
    // Primal infeasibility tolerance
    params.eps_prim_inf = json_data["osqp_primal_infeasibility_tolerance"];
    // Dual infeasibility tolerance
    params.eps_dual_inf = json_data["osqp_dual_infeasibility_tolerance"];
    // Runtime limit in seconds
    params.time_limit = json_data["osqp_time_limit"];
    params.maximum_iteration = json_data["osqp_maximum_iteration"];
    params.enable_warm_start = json_data["enable_mpc_warm_start"];
    params.verbose = json_data["osqp_verbose"];
    params.polish = true;

    PRINT("Set parameters...");
    lmpc.setOptimizerParameters(params);
    PRINT("Finshed setting parameters");

    lmpc.setStateSpaceModel(Ad, Bd, Cd);
    lmpc.setDisturbances(Bd_disturbance, mat<Tny, Tndu>::Zero());

    // ======== Weights ========== //

    // Output Weights
    cvec<Tny> OutputW;
    OutputW.setOnes();
    OutputW *= json_data["output_cost_weight"];

    // Input Weights
    cvec<Tnu> InputW;
    InputW.setOnes();
    InputW *= json_data["input_cost_weight"];

    // Input change weights
    cvec<Tnu> DeltaInputW;
    DeltaInputW.setZero();

    lmpc.setObjectiveWeights(OutputW, InputW, DeltaInputW, {0, prediction_horizon});

    // Set 
    mat<Tndu, prediction_horizon> disturbance_input;
    disturbance_input.setOnes();
    disturbance_input *= lead_car_input;
    lmpc.setExogenuosInputs(disturbance_input);
    // printMat("Disturbance input", disturbance_input);
    // printMat("Disturbance matirx", Bd_disturbance);
    auto disturbance_vec = Bd_disturbance;
    disturbance_vec *= disturbance_input(0);
    // printMat("Disturbance vector", disturbance_vec);

    // ======== Constraints ========== //

    // State constraints.
    cvec<Tnx> xmin, xmax;
    loadMatrixValuesFromJson(xmin, json_data, "xmin");
    loadMatrixValuesFromJson(xmax, json_data, "xmax");

    // Output constraints
    cvec<Tny> ymin, ymax;
    loadMatrixValuesFromJson(ymin, json_data, "ymin");
    loadMatrixValuesFromJson(ymax, json_data, "ymax");

    // Control constraints.
    cvec<Tnu> umin, umax;
    loadMatrixValuesFromJson(umin, json_data, "umin");
    loadMatrixValuesFromJson(umax, json_data, "umax");

    lmpc.setConstraints(xmin, umin, ymin, xmax, umax, ymax, {0, prediction_horizon});

    // Output reference point
    cvec<Tny> yRef;
    loadMatrixValuesFromJson(yRef, json_data, "yref");

    lmpc.setReferences(yRef, cvec<Tnu>::Zero(), cvec<Tnu>::Zero(), {0, prediction_horizon});

    // State vector state.
    mpc::cvec<Tnx> modelX, modeldX;
    // Set the initial value.
    if (start_idx == 0)
      loadMatrixValuesFromJson(modelX, json_data, "x0");
    else
      modelX = x_0;

    // Output vector.
    mpc::cvec<Tny> y;

    // Create a vector for storing the control input from the previous time step.
    mpc::cvec<Tnu> u;
    u = cvec<Tnu>::Zero();

    int timestep = 0;
    u = u_start;

    // Record state and input x_t,u_t
    Eigen::VectorXd xu(x_0.size() + 1);
    xu << modelX, u;
    // Add the zero input to the beginning
    if(start_idx==0)
        xu_vector.push_back(xu);

    modelX = Ad * modelX + Bd * u + Bd_disturbance*lead_car_input;
    y = Cd * modelX;
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
        else
        {
            mpc::Result r = lmpc.step(modelX, u);
            u = r.cmd;
        }
        if (record_trace)
        {
            dr_app_stop_and_cleanup();
            exit(0);
        }
        // Control calculations ends here

        // Update the simulation
        timestep++;
        std::cout << "Timestep: " << timestep << std::endl;

        modelX = Ad * modelX + Bd * u + Bd_disturbance*lead_car_input;
        y = Cd * modelX;

        // Record state and input x_t,u_t
        Eigen::VectorXd xu(x_0.size() + 1);
        xu << modelX, u;
        xu_vector.push_back(xu);
    }

    // Create a CSV file to store the data
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
