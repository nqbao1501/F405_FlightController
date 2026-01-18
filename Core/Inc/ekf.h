/*
 * ekf.h
 *
 *  Created on: 19 Dec 2025
 *      Author: Admin
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

#define G ((float) 9.81f)

typedef struct {
    // State (minimal public outputs)
	float X[6];
	//Process noise matrix Q [6x6], diag
	float Q[6][6];

	//Measurement noise matrix R [3x3], diag
	float R[3][3];

	//Error covariance matrix P [6x6], diag
	float P[6][6];
}EKF_t;

void EKF_Init(EKF_t *ekf, float bp0, float bq0);
void EKF_Get_dynamics_matrices(const float X[6], float gyro_rad[3], float A[6][6], float f[6]);
void EKF_Predict(EKF_t *ekf, float gyro_rad[3], float dt);
void EKF_AccelModel(const float X[6], float h[3], float C[3][6]);
void EKF_UpdateAccel(EKF_t *ekf, float accel[3]);
void quad_to_euler(EKF_t *ekf, float *roll, float *pitch);


#endif /* INC_EKF_H_ */
