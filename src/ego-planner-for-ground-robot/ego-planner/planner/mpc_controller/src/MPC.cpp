#include "MPC.hpp"
#include "math.h"
#include "iostream"

#define PI 3.1415926
#define T 0.03
#define w0 1.0
#define w1 0.5
#define Ku 1
#define Kl 1


using namespace Eigen;
using namespace std;
USING_NAMESPACE_QPOASES

namespace
{
/* Accepts both `MPC/v_max: 1.8` and `MPC/v_max: 2` in the parameter file. */
double getDoubleParam(const rclcpp::Node::SharedPtr &nh, const std::string &name, double default_value)
{
    if (nh->has_parameter(name)) {
        const auto p = nh->get_parameter(name);
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            return static_cast<double>(p.as_int());
        }
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            return p.as_double();
        }
        return default_value;
    }
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    const rclcpp::ParameterValue value =
        nh->declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
    const rclcpp::Parameter p(name, value);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        return static_cast<double>(p.as_int());
    }
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return p.as_double();
    }
    return default_value;
}
}  // namespace

MatrixXd MPC_controller::MPC_Solve_qp(Eigen::Vector3d X_k,std::vector<Eigen::Vector3d >X_r,std::vector<Eigen::Vector2d >U_r,const int N)
{
    if (N <= 0 || X_r.size() != static_cast<size_t>(N) ||
        U_r.size() != static_cast<size_t>(N) || !X_k.allFinite())
        return MatrixXd::Zero(2, std::max(0, N));
    for (int k = 0; k < N; ++k)
        if (!X_r[k].allFinite() || !U_r[k].allFinite())
            return MatrixXd::Zero(2, N);
    //cout<<"current w : "<<w_max<<endl;
    ////根据参考输入计算出的系数矩阵
    vector<MatrixXd> A_r(N),B_r(N),A_multiply1(N);
    MatrixXd O_r(3*N,1);
    MatrixXd A_bar(3*N,3);
    MatrixXd X_ref(3*N,1);
    VectorXd U_ref(2*N);
    MatrixXd A_multiply2;
    MatrixXd B_bar = MatrixXd::Zero(3*N,2*N);
    MatrixXd C_bar = MatrixXd::Identity(3*N,3*N);
    MatrixXd A_r_init = MatrixXd::Zero(3,3);
    MatrixXd B_r_init = MatrixXd::Zero(3,2);
    MatrixXd eye_3 = MatrixXd::Identity(3,3);
    MatrixXd Q = MatrixXd::Identity(3*N,3*N)*omega0;
    for (int k = 0; k < N; ++k) {
        Q(3*k,3*k) = Q(3*k+1,3*k+1) = position_weight;
        Q(3*k+2,3*k+2) = heading_weight;
    }
    MatrixXd R = MatrixXd::Identity(2*N,2*N)*omega1;
//    cout<<"Ur 1 : "<<endl<<U_r[0]<<endl;
//    cout<<"Xr 1 : "<<endl<<X_r[0]<<endl;
//    cout<<"Xk : "<<endl<<X_k<<endl;
    for(int k=0;k<N;k++)
    {
        A_r[k] = A_r_init;
        B_r[k] = B_r_init;
        A_r[k](0,2) = -U_r[k](0)*sin(X_r[k](2));
        A_r[k](1,2) = U_r[k](0)*cos(X_r[k](2));
        Vector3d temp_vec = -T*A_r[k]*X_r[k];
        O_r.block<3,1>(k*3,0) = temp_vec;
        A_r[k] = eye_3+T*A_r[k];
        B_r[k](0,0) = cos(X_r[k](2))*T;
        B_r[k](1,0) = sin(X_r[k](2))*T;
        B_r[k](2,1) = T;
        X_ref.block<3,1>(k*3,0) = X_r[k];
        U_ref.segment<2>(2*k) = U_r[k];
//        cout<<"B_r "<<k+1<<" : "<<endl<<B_r[k]<<endl;

        if(k==0) A_multiply1[k] = A_r[k];
        else A_multiply1[k] = A_r[k]*A_multiply1[k-1];
        A_bar.block<3,3>(3*k,0) = A_multiply1[k];
    }
//    cout<<"Ar 1 : "<<endl<<A_r[0]<<endl;
//    cout<<"Br 1 : "<<endl<<B_r[0]<<endl;
//    cout<<"Or 1 : "<<endl<<O_r.block<3,1>(0,0)<<endl;
//    cout<<"O_r : "<<endl<<O_r<<endl;

    for(int k=0;k<N;k++)
    {
        B_bar.block<3,2>(3*k,2*k) = B_r[k];
        A_multiply2 = eye_3;
        for(int i=0;i<k;i++)
        {
            A_multiply2 = A_multiply2*A_r[k-i];
            C_bar.block<3,3>(3*k,3*(k-1-i)) = A_multiply2;
            B_bar.block<3,2>(3*k,2*(k-1-i)) = A_multiply2*B_r[k-1-i];
        }
    }

    //cout<<"C bar : "<<endl<<C_bar<<endl;

    MatrixXd E =A_bar*X_k+C_bar*O_r-X_ref;
    MatrixXd Hesse = 2*(B_bar.transpose()*Q*B_bar+R);      ////Hesse矩阵
    // Penalize deviation from the spline's velocity/turn rate. Penalizing U
    // against zero discards the reference input and creates tracking lag,
    // especially around bends, even when already exactly on the reference.
    VectorXd gradient = 2*(B_bar.transpose()*Q*E - R*U_ref);


    real_t H[2*N*2*N],g[2*N],A[2*N],lb[2*N],ub[2*N],lbA[1],ubA[1];
    lbA[0] = N*(v_min+w_min)/Ku;
    ubA[0] = N*(v_max+w_max)/Kl;
    for(int i=0;i<2*N;i++)
    {
        g[i] = gradient(i);
        A[i] = 1;
        if(i%2==0)
        {
            lb[i] = v_min;
            ub[i] = v_max;
        }
        else
        {
            lb[i] = w_min;
            ub[i] = w_max;
        }
        for(int j=0;j<2*N;j++)
        {
            H[i*2*N+j] = Hesse(i,j);
        }
    }

    int_t nWSR = 800;

    QProblem mpc_qp_solver(2*N,1);
    Options options;
    options.printLevel = PL_NONE;
    mpc_qp_solver.setOptions(options);
    if (mpc_qp_solver.init(H,g,A,lb,ub,lbA,ubA,nWSR) != SUCCESSFUL_RETURN)
    {
        RCLCPP_WARN(rclcpp::get_logger("mpc_controller"), "MPC QP failed; returning zero velocity");
        return MatrixXd::Zero(2,N);
    }

    real_t x_solution[2*N] = {};
    if (mpc_qp_solver.getPrimalSolution(x_solution) != SUCCESSFUL_RETURN)
        return MatrixXd::Zero(2,N);


    Vector2d u_k;
    MatrixXd U_result = MatrixXd::Zero(2,N);
    for(int i=0;i<N;i++)
    {
        u_k(0) = x_solution[2*i];
        u_k(1) = x_solution[2*i+1];
        U_result.col(i) = u_k;
//        std::cout<<"U "<<i+1<<" : "<<endl<<u_k<<endl;
    }
    //cout<<"N : "<<N<<endl;
    return U_result;
}

void MPC_controller::MPC_init(const rclcpp::Node::SharedPtr &nh)
{
    // In ROS 2 a node can only read its own parameters, so `traj_server` declares
    // the MPC weights itself (see config/ego_planner.yaml).
    v_max = getDoubleParam(nh, "MPC.v_max", 1.8);
    w_max = getDoubleParam(nh, "MPC.w_max", 1.2);
    omega0 = getDoubleParam(nh, "MPC.omega0", 1.0);
    omega1 = getDoubleParam(nh, "MPC.omega1", 0.5);
    position_weight = getDoubleParam(nh, "MPC.position_weight", omega0);
    heading_weight = getDoubleParam(nh, "MPC.heading_weight", omega0);

    RCLCPP_INFO(nh->get_logger(), "v_max : %f , w_max : %f",v_max,w_max);
    w_min = -w_max;
    v_min=0;

}
