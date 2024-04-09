#ifndef UWB_LOKA_OBSERVER_H
#define UWB_LOKA_OBSERVER_H

#include <Eigen/Core>
#include <memory>

namespace uwb_loka 
{

// /// @brief HX_X：表示测量函数 h(x) 对状态变量 x 的雅可比矩阵。
// ///        NEGATIVE_RX_X：表示残差函数 -r(x) 对状态变量 x 的雅可比矩阵。
// enum JACOBIAN_MEASUREMENT 
// { 
//     HX_X, 
//     NEGATIVE_RX_X 
// };  // h(x)/delta X, -r(x)/delta X

class Observer
{
public:
    Observer() = default;

    Observer( const Observer & ) = delete;

    virtual ~Observer() {}

    virtual Eigen::MatrixXd measurement_function( const Eigen::MatrixXd &mat_x ) = 0;

    virtual Eigen::MatrixXd measurement_residual( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ) = 0;

    virtual Eigen::MatrixXd measurement_jacobian( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ) = 0;

    virtual void check_jacobian( const Eigen::MatrixXd &mat_x, const Eigen::MatrixXd &mat_z ) = 0;
};

using ObserverPtr = std::shared_ptr<Observer>;

}  // namespace uwb_loka

#endif // UWB_LOKA_OBSERVER_H
