#include "eadrc_longitudinal_controller/ESO.hpp"
#include <iostream>

namespace autoware::motion::control::eadrc_longitudinal_controller
{

bool calculateGainsOfLVector(double* p_L1Value, double* p_L2Value, double bandwidthOfESO)
{
    if(bandwidthOfESO < static_cast<double>(0.0))
    {
        // Exit, negative value of bandwidth of ESO
        return false;
    }

    *p_L1Value = 2 * bandwidthOfESO;
    *p_L2Value = bandwidthOfESO * bandwidthOfESO;

    return true;
}

ESO::ESO(double estimatedGainOfSystem, uint32_t bandwidthOfESO)
{
    m_A << 0, 1,
            0, 0;

    m_B << estimatedGainOfSystem, 0;

    m_C << 1, 0;

    double L1Value = 0;
    double L2Value = 0; 

    bool resultOfCalculate = calculateGainsOfLVector(&L1Value, &L2Value, bandwidthOfESO);

    if (resultOfCalculate == true)
    {
        m_L << L1Value, L2Value;
    }
    else
    {
        m_L << 0, 0;  // Set to default values
    }

    m_lastControlSignal = 0;
    m_lastLogitudinalError = 0;
    m_lastStateVector << 0, 0;
}

Eigen::Vector2d ESO::calculateStateOfESO(double error, double dt, double controlSignal)
{
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();  // Ensure identity matrix initialization

    Eigen::Vector2d stateVector = (m_A * dt - dt * m_L * m_C + I) * m_lastStateVector + dt * m_B * controlSignal + dt * m_L * error;

    m_lastStateVector = stateVector;

    return stateVector;
}

Eigen::Vector2d ESO::getStateVector()
{
    return m_lastStateVector;
}

void ESO::setLastStateVector(Eigen::RowVector2d stateVector)  // Changed RowVector2d to Vector2d for type consistency
{
    m_lastStateVector = stateVector;  // Simplified assignment
}

}  // namespace autoware::motion::control::eadrc_longitudinal_controller
