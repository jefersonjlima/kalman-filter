# Kalman Filter

# Overview

# Kalman Filter Algorithm:


## The Code Explained

### Step 1:

**Prediciton** $`({\mu_{t-1}, {\textstyle\sum}_{t-1}, u_t})`$

$`\overline{\mu}_t = A_t\mu_{t-1} + B_t u_t`$

$`\overline{\textstyle\sum}_t = A_t {\textstyle\sum}_{t-1} A_t^T+ Q_t`$

$`\textbf{Return} \left(\overline{\mu}_t, \overline{\textstyle\sum}_t\right)`$

code reference: 
```cpp
void LinearSystem::time_update(const Eigen::MatrixXd& u){

    mu_hat  = A * mu_hat + B * u;
    Sigma_hat = A * Sigma_hat * A.transpose() + Q;
}
```


### Step 2:

**Measurement Update** $`(\overline{\mu}_{t}, \overline{\textstyle\sum}_{t}, z_t)`$:

$`K_t = \overline{\textstyle\sum}_tC_t^T(C_t\overline{\textstyle\sum}_tC_t^T+R_t)^{-1}`$

$`\mu_t  = \overline{\mu}_t + K_t(z_t -C_t\overline\mu_t)`$

$`\textstyle\sum_t = (I-K_tC_t)\overline{\textstyle\sum}_t`$

$`\textbf{Return} \left(\mu_t, \textstyle\sum_t\right)`$

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
    $ cmake ..
    $ make -j4
    $ ./kf-test
