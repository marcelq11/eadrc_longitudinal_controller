#include "eadrc_longitudinal_controller/ESO.hpp"

#include <iostream>


namespace eadrc_longitudinal_controller
{

    ESO::ESO(double dt, double estimatedGainOfSystem, uint32_t bandwidthOfESO)//TODO fill after : preinit
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

        m_stateVectorOld << 0, 0;
        m_stateVectorNew << 0, 0;
    }

    bool calculateGainsOfLVector(double* p_L1Value, double* p_L2Value, double bandwidthOfESO)
    {
        /*
        choose gains based on multiple pole placement
        det(lambda*I -H_1) = (labda + omega_0)^2
        */

        if(bandwidthOfESO < static_cast<double>(0.0))
        {
            //TODO: exit, negative value of bandwidth of ESO
            return false;
        }

        *p_L1Value = 2*bandwidthOfESO;
        *p_L2Value = bandwidthOfESO*bandwidthOfESO;

        return true;
    }

    void ESO::calculateStateOfESO(double error, double controlSignal)
    {
        double lastMeasuredSpeed = 0; // TODO fill it
        double dt = 0; // TODO fill it

        Eigen::Matrix2d I;
        I << 1, 0,
             0, 1;

        m_stateVectorNew = (m_A*dt-dt*m_L*m_C+I)*m_stateVectorOld+dt*m_B*controlSignal+dt*m_L*error;
    }

}  // namespace eadrc_longitudinal_controller