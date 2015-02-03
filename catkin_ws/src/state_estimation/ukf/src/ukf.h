#ifndef UKF_H_
#define UKF_H_

#include <eigen3/Eigen/Dense>

using namespace Eigen;

typedef Matrix<double, 3, 6> Matrix3X6d;
typedef const Ref<const VectorXd> constVector;


class ukf
{
    public:
        ukf(int dim);
        void predict(constVector, void (*)(Eigen::VectorXd,Ref<Eigen::VectorXd>));
        void correct(constVector, void (*)(Eigen::VectorXd,Ref<Eigen::VectorXd>));
        const int DIM;
        VectorXd state;
        MatrixXd covariance;
        MatrixXd processCovariance;
        MatrixXd measurementCovariance;
    
    private:
        void generateSigmas();
        void recoverPrediction();
        void recoverCorrection(constVector);
        Matrix3X6d sigmas;
        Matrix3X6d gammas;
};


#endif 
