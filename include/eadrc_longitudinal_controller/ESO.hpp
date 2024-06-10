#ifndef ESO_HPP
#define ESO_HPP

#include <Eigen/Dense>

namespace autoware::motion::control::eadrc_longitudinal_controller
{

class ESO
{
public:
    ESO(double estimatedGainOfSystem, uint32_t bandwidthOfESO);
    Eigen::Vector2d calculateStateOfESO(double error, double dt, double controlSignal);
    void setLastStateVector(Eigen::RowVector2d stateVector);
    Eigen::Vector2d getStateVector();

private:
    Eigen::Matrix2d m_A;
    Eigen::Vector2d m_B;
    Eigen::RowVector2d m_C;
    Eigen::Vector2d m_L;

    Eigen::Vector2d m_lastStateVector;
    double m_lastControlSignal;
    double m_lastLogitudinalError;
};

}  // namespace autoware::motion::control::eadrc_longitudinal_controller

#endif  // ESO_HPP
