# Kalman Filter

## Overview

## The Code Explained

### Step 1:

![](https://latex.codecogs.com/svg.image?%5Cbegin%7Barray%7D%7Bll%7D%5Ctextbf%7BPrediciton%7D(%7B%5Cmu_%7Bt-1%7D,%20%7B%5Ctextstyle%5Csum%7D_%7Bt-1%7D,%20u_t%7D)%20%5C%5C%5Cquad%5Cquad%5Coverline%7B%5Cmu%7D_t%20=%20A_t%5Cmu_%7Bt-1%7D%20&plus;%20B_t%20u_t%20%5C%5C%5Cquad%5Cquad%5Coverline%7B%5Ctextstyle%5Csum%7D_t%20=%20A_t%20%7B%5Ctextstyle%5Csum%7D_%7Bt-1%7D%20A_t%5ET&plus;%20Q_t%5C%5C%5Ctextbf%7BReturn%7D%20%5Cleft(%5Coverline%7B%5Cmu%7D_t,%20%5Coverline%7B%5Ctextstyle%5Csum%7D_t%5Cright)%5Cend%7Barray%7D)

code reference: 
```cpp
void LinearSystem::time_update(const Eigen::MatrixXd& u){

    mu_hat  = A * mu_hat + B * u;
    Sigma_hat = A * Sigma_hat * A.transpose() + Q;
}
```

### Step 2:

![](https://latex.codecogs.com/svg.image?%5Cbegin%7Barray%7D%7Bll%7D%5Ctextbf%7BMeasurement%20Update%7D(%5Coverline%7B%5Cmu%7D_%7Bt%7D,%20%5Coverline%7B%5Ctextstyle%5Csum%7D_%7Bt%7D,%20z_t):%20%5C%5C%5Cquad%5Cquad%20K_t%20=%20%5Coverline%7B%5Ctextstyle%5Csum%7D_tC_t%5ET(C_t%5Coverline%7B%5Ctextstyle%5Csum%7D_tC_t%5ET&plus;R_t)%5E%7B-1%7D%5C%5C%5Cquad%5Cquad%20%5Cmu_t%20%20=%20%5Coverline%7B%5Cmu%7D_t%20&plus;%20K_t(z_t%20-C_t%5Coverline%5Cmu_t)%5C%5C%5Cquad%5Cquad%20%5Ctextstyle%5Csum_t%20=%20(I-K_tC_t)%5Coverline%7B%5Ctextstyle%5Csum%7D_t%5C%5C%5Ctextbf%7BReturn%7D%20%5Cleft(%5Cmu_t,%20%5Ctextstyle%5Csum_t%5Cright)%5Cend%7Barray%7D)

code reference: 
```cpp
void KFilter::measurement_update(const Eigen::MatrixXd& z){

     auto K = Sigma_hat * C.transpose() * (C * Sigma_hat * C.transpose() + R).inverse();
     mu_hat += K * (z - C * mu_hat);
     Sigma_hat = (I - K * C) * Sigma_hat;
}
```

## Installation and Usage

To run it, use CMake:

    $ cd kalman-filter
    $ mkdir build
    $ cd build
    $ cmake .. && $ make -j`nproc`
    $ ./kf-test


## References

- Sebastian Thrun, Wolfram Burgard, Dieter Fox. **Probabilistic Robotics** (Intelligent Robotics and Autonomous Agents series). The MIT Press, 2005.

