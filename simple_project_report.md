# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./home/workspace/CarND-Extended-Kalman-Filter-Project/dataset1_accuracy.JPG "Accuracy"

## Equation Used
Kalman Filter Prediction: x' = F * x + v
						  P' = F * P * F^T + Q

Kalman Filter Update: y = z - H * x'
					  S = H * P' * H^T + R
                      K = P' * H^T * S^-1
                      x = x' + K * y
                      P = (I - K * H) * P'
EKF has different F and Q matrix, and h function directly to map predicted locations x' from cartesian to polar coordinates
EKF : dt = measurement timestamp - previous timestamp
	  F = 0 0 dt 0
	      0 0 0 dt
          0 0 0 0
          0 0 0 0   
      Q = dt^4/4 * noise_ax          0           dt^3/2 * noise_ax         0
                 0            dt^4/4 * noise_ay         0           dt^3/2 * noist_ay
          dt^3/2 * noise_ax          0             dt^2 * noise_ax         0
                 0            dt^3/2 * noise_ay         0             dt^2 * noist_ay
Convert radar cartesian coordinates to polar coordinates (h function):
		rho     = sqrt(px^2 + py^2)
        phi     = arctan(py / px)
        rho_dot = (px * vx + py * vy) / rho
        
## Evaluation
Jacobian matrix =          px / sqrt(px^2 + py^2)                px / sqrt(px^2 + py^2)   			 0     				    0
                              -py / (px^2 + py^2)                    py / (px^2 + py^2)              0         				0
                  py(vxpy - vypx) / (px^2 + py^2)^3/2   px(vxpy - vypx) / (px^2 + py^2)^3/2   px / sqrt(px^2 + py^2) py / sqrt(px^2 + py^2)
RMSE = sqrt( mean( (estimation - ground truth)^2 ) )

## Accuracy
Testing the EKF on dataset 1: RMSE = [0.0974, 0.0855, 0.4517, 0.4404], see image below:

![Accuracy][image1]
