#ifndef MPC_H_
#define MPC_H_

#include <qpOASES.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <rclcpp/rclcpp.hpp>

class MPC_controller
{
public:
    /****
     * 使用线性误差模型，用qpOASES求解
     * @param X_c 当前状态
     * @param X_r 未来 N 步的参考状态，第一项对应 t + T
     * @param U_r 每一步起始时刻的参考输入 [v, w]，第一项对应 t
     * @param N   预测步数
     * @return    预测输出序列
     */
    Eigen::MatrixXd MPC_Solve_qp(Eigen::Vector3d X_k,std::vector<Eigen::Vector3d >X_r,std::vector<Eigen::Vector2d >U_r,const int N);
    void MPC_init(const rclcpp::Node::SharedPtr &nh);
private:
    double v_max;
    double v_min;
    double w_max;
    double w_min;
    double omega0,omega1;
    double position_weight, heading_weight;
};

#endif
