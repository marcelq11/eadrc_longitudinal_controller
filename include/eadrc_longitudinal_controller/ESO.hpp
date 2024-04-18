namespace eadrc_longitudinal_controller
{

class ESO
{
public:
    ESO(double dt_, uint16_t orderOfESO_, double estimatedGainOfSystem_, uint32_t bandwidthOfESO_);
    void calculateStateOfESO(double error, double controlSignal);

private:
    uint16_t m_orderOfESO;
    double m_estimatedGainOfSystem;
    uint32_t m_bandwidthOfESO;
    double m_dt;
    Eigen::VectorXd m_stateVector(3);//ile???
}



}  // namespace eadrc_longitudinal_controller