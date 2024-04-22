#include <Eigen/Eigen>

namespace eadrc_longitudinal_controller
{

class ESO
{
public:
    ESO(double dt, double estimatedGainOfSystem, uint32_t bandwidthOfESO);
    Eigen::RowVector2d calculateStateOfESO(double error, double controlSignal);

private:
    Eigen::Matrix2d m_A;
    Eigen::RowVector2d m_B(2);
    Eigen::RowVector2d m_C;
    Eigen::RowVector2d m_L(2);

    Eigen::RowVector2d m_lastStateVector(2);
    double m_lastControlSignal;
    double m_lastLogitudinalError;
}



}  // namespace eadrc_longitudinal_controller