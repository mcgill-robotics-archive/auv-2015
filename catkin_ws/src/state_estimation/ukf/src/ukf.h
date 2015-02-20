#ifndef UKF_H_
#define UKF_H_

#include <eigen3/Eigen/Dense>
// Can't use <functional> because target is C03
#include <boost/function.hpp>

using namespace Eigen;

typedef const Ref<const VectorXd> constVector;

class ukf
{
    public:
        ukf(int dim);
        void predict(boost::function<void (Ref<Eigen::VectorXd>)>);
        void correct(constVector, MatrixXd(*)(MatrixXd));
        const int DIM;
        VectorXd state;
        MatrixXd covariance;
        MatrixXd processCovariance;
        MatrixXd measurementCovariance;
    
    private:
        void generateSigmas();
        void recoverPrediction();
        void recoverCorrection(constVector, MatrixXd);
        MatrixXd sigmas;
};

#endif 
