/*
 * ekf.c
 *
 *  Created on: 19 Dec 2025
 *      Author: Admin
 */

#include "ekf.h"
#include <string.h>
#include <math.h>


static inline void quat_normalize(float q[4])
{
    float n = sqrtf(
        q[0]*q[0] +
        q[1]*q[1] +
        q[2]*q[2] +
        q[3]*q[3]
    );

    if (n > 0.0f) {
        float inv = 1.0f / n;
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
    }
}
static inline void mat_transpose_3x6(const float A[3][6], float At[6][3]){
	for (int i = 0; i < 3; i++){
        const float *Ai = A[i];
		for (int j = 0; j < 6; j++){
            At[j][i] = Ai[j];
		}
	}
}
static inline void mat_transpose_6x3(const float A[6][3], float At[3][6]){
	for (int i = 0; i < 6; i++){
        const float *Ai = A[i];
		for (int j = 0; j < 3; j++){
            At[j][i] = Ai[j];
		}
	}
}
static inline void mat_transpose_6x6(const float A[6][6], float At[6][6]){
	for (int i = 0; i < 6; i++){
        const float *Ai = A[i];
		for (int j = 0; j < 6; j++){
            At[j][i] = Ai[j];
		}
	}
}
static inline void mat_mul_6x6X6x6(
    const float A[6][6],
    const float B[6][6],
    float C[6][6])
{
    for (int i = 0; i < 6; i++) {
        const float *Ai = A[i];
        for (int j = 0; j < 6; j++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) {
                s += Ai[k] * B[k][j];
            }
            C[i][j] = s;
        }
    }
}
static inline void mat_mul_6x6X6x3(
    const float A[6][6],
    const float B[6][3],
    float C[6][3])
{
    for (int i = 0; i < 6; i++) {
        const float *Ai = A[i];
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) {
                s += Ai[k] * B[k][j];
            }
            C[i][j] = s;
        }
    }
}
static inline void mat_mul_6x6X6x1(
    const float A[6][6],
    const float B[6],
    float C[6])
{
    for (int i = 0; i < 6; i++) {
        const float *Ai = A[i];
        float s = 0.0f;
        for (int k = 0; k < 6; k++) {
            s += Ai[k] * B[k];
        }
        C[i] = s;
    }
}
static inline void mat_mul_6x3X3x6(
    const float A[6][3],
    const float B[3][6],
    float C[6][6])
{
    for (int i = 0; i < 6; i++) {
        const float *Ai = A[i];
        for (int j = 0; j < 6; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                s += Ai[k] * B[k][j];
            }
            C[i][j] = s;
        }
    }
}
static inline void mat_mul_6x3X3x3(
    const float A[6][3],
    const float B[3][3],
    float C[6][3])
{
    for (int i = 0; i < 6; i++) {
        const float *Ai = A[i];
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                s += Ai[k] * B[k][j];
            }
            C[i][j] = s;
        }
    }
}
static inline void mat_mul_6x3X3x1(
    const float A[6][3],
    const float B[3],
    float C[6])
{
    for (int i = 0; i < 6; i++) {
        const float *Ai = A[i];
        float s = 0.0f;
        for (int k = 0; k < 3; k++) {
            s += Ai[k] * B[k];
        }
        C[i] = s;
    }
}
static inline void mat_mul_3x6X6x6(
    const float A[3][6],
    const float B[6][6],
    float C[3][6])
{
    for (int i = 0; i < 3; i++) {
        const float *Ai = A[i];
        for (int j = 0; j < 6; j++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) {
                s += Ai[k] * B[k][j];
            }
            C[i][j] = s;
        }
    }
}
static inline void mat_mul_3x6X6x3(
    const float A[3][6],
    const float B[6][3],
    float C[3][3])
{
    for (int i = 0; i < 3; i++) {
        const float *Ai = A[i];
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 6; k++) {
                s += Ai[k] * B[k][j];
            }
            C[i][j] = s;
        }
    }
}
static inline void mat_symmetrize_6x6(float P[6][6])
{
    for (int i = 0; i < 6; i++) {
        for (int j = i + 1; j < 6; j++) {
            float v = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = v;
            P[j][i] = v;
        }
    }
}





static inline void mat6_add(const float A[6][6],
                            const float B[6][6],
                            float C[6][6])
{
    for (int i=0;i<6;i++)
        for (int j=0;j<6;j++)
            C[i][j] = A[i][j] + B[i][j];
}

void EKF_Init(EKF_t *ekf, float bp0, float bq0)
{


    memset(ekf, 0, sizeof(EKF_t));

    ekf->X[0] = 1.0f; // identity quaternion
    ekf->X[4] = bp0;
    ekf->X[5] = bq0;

    for (int i=0;i<6;i++)
        ekf->P[i][i] = 1e-3f;

    for (int i=0;i<4;i++)
        ekf->Q[i][i] = 1e-8f;

    ekf->Q[4][4] = 1e-8f;
    ekf->Q[5][5] = 1e-8f;

    for (int i=0;i<3;i++)
        ekf->R[i][i] = 0.01f;
}

void EKF_get_dynamics_matrices(const float X[6], float gyro_rad[3], float A[6][6], float f[6]){
	float q0 = X[0];
	float q1 = X[1];
	float q2 = X[2];
	float q3 = X[3];
	float bp = X[4];
	float bq = X[5];

	float wx = gyro_rad[0] - bp;
	float wy = gyro_rad[1] - bq;
	float wz = gyro_rad[2];

	f[0] = 0.5*(-q1*wx - q2*wy - q3*wz);
	f[1] = 0.5*(q0*wx + q3*wy - q2*wz);
	f[2] = 0.5*(-q3*wx + q0*wy - q1*wz);
	f[3] = 0.5*(q2*wx - q1*wy + q0*wz);
	f[4] = 0;
	f[5] = 0;

    memset(A, 0, sizeof(float)*36);

    A[0][1] = -0.5f*wx;  A[0][2] = -0.5f*wy;  A[0][3] = -0.5f*wz;
    A[0][4] =  0.5f*q1;  A[0][5] =  0.5f*q2;

    A[1][0] =  0.5f*wx;  A[1][2] = -0.5f*wz;  A[1][3] =  0.5f*wy;
    A[1][4] = -0.5f*q0;  A[1][5] = -0.5f*q3;

    A[2][0] =  0.5f*wy;  A[2][1] =  0.5f*wz;  A[2][3] = -0.5f*wx;
    A[2][4] =  0.5f*q3;  A[2][5] = -0.5f*q0;

    A[3][0] =  0.5f*wz;  A[3][1] = -0.5f*wy;  A[3][2] =  0.5f*wx;
    A[3][4] = -0.5f*q2;  A[3][5] =  0.5f*q1;
}

/*
 * EKF predict step
 * X = X + f*dt;
 * P = P + (A*P + P*A' + Q)*dt;
 */
void EKF_Predict(EKF_t *ekf, float gyro_rad[3], float dt){
	float A[6][6];
	float f[6];

	if (dt <= 0.0f || dt > 0.1f)
	    return;

	//Normalize quaternion in X before using it
	quat_normalize(ekf->X);

	//Compute f and A_cont
	EKF_get_dynamics_matrices(ekf->X, gyro_rad, A, f);

	//X = X + f*dt;
	for (int i = 0; i < 6; i++){
		ekf->X[i] += f[i] * dt;
	}
	quat_normalize(ekf->X);

	//P = P + (A*P + P*A' + Q)*dt
    float AP[6][6];
    float At[6][6];
    float PAt[6][6];

    memset(AP, 0, sizeof(AP));
    mat_transpose_6x6(A, At);
    memset(PAt, 0, sizeof(PAt));

    mat_mul_6x6X6x6(A, ekf->P, AP);
    mat_mul_6x6X6x6(ekf->P, At, PAt);

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            ekf->P[i][j] += (AP[i][j] + PAt[i][j] + ekf->Q[i][j]) * dt;
}

void EKF_AccelModel(const float X[6], float h[3], float C[3][6]){
	float q0 = X[0];
    float q1 = X[1];
    float q2 = X[2];
    float q3 = X[3];


    h[0] = G * (2*(q1*q3 - q0*q2));
    h[1] = G * (2*(q2*q3 + q0*q1));
    h[2] = G * (q0*q0 - q1*q1 - q2*q2 + q3*q3);

    memset(C, 0, sizeof(float)*18);

    C[0][0] = -2*G*q2;  C[0][1] =  2*G*q3;  C[0][2] = -2*G*q0;  C[0][3] =  2*G*q1;
    C[1][0] =  2*G*q1;  C[1][1] =  2*G*q0;  C[1][2] =  2*G*q3;  C[1][3] =  2*G*q2;
    C[2][0] =  2*G*q0;  C[2][1] = -2*G*q1;  C[2][2] = -2*G*q2;  C[2][3] =  2*G*q3;
}

void EKF_UpdateAccel(EKF_t *ekf, float accel[3])
{
    float h[3], y[3];
    float C[3][6];

    float CP[3][6];
    float Ct[6][3];
    float S[3][3];
    float S_inv[3][3];

    float PCt[6][3];
    float K[6][3];

    float KC[6][6];
    float A[6][6];
    float AP[6][6];
    float APA[6][6];

    float KR[6][3];
    float KRKt[6][6];

    EKF_AccelModel(ekf->X, h, C);

    // Y = Z - h
    for (int i=0;i<3;i++)
        y[i] = accel[i] - h[i];

    // S = C * P_pred * C' + R;
    mat_mul_3x6X6x6(C, ekf->P, CP);
    mat_transpose_3x6(C, Ct);
    mat_mul_3x6X6x3(CP, Ct, S);
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            S[i][j] += ekf->R[i][j];

    // invert S (analytic 3x3)
    float det =
        S[0][0]*(S[1][1]*S[2][2]-S[1][2]*S[2][1]) -
        S[0][1]*(S[1][0]*S[2][2]-S[1][2]*S[2][0]) +
        S[0][2]*(S[1][0]*S[2][1]-S[1][1]*S[2][0]);

    if (fabsf(det) < 1e-6f)
        return;

    float invdet = 1.0f / det;

    S_inv[0][0]=(S[1][1]*S[2][2]-S[1][2]*S[2][1])*invdet;
    S_inv[0][1]=(S[0][2]*S[2][1]-S[0][1]*S[2][2])*invdet;
    S_inv[0][2]=(S[0][1]*S[1][2]-S[0][2]*S[1][1])*invdet;
    S_inv[1][0]=(S[1][2]*S[2][0]-S[1][0]*S[2][2])*invdet;
    S_inv[1][1]=(S[0][0]*S[2][2]-S[0][2]*S[2][0])*invdet;
    S_inv[1][2]=(S[0][2]*S[1][0]-S[0][0]*S[1][2])*invdet;
    S_inv[2][0]=(S[1][0]*S[2][1]-S[1][1]*S[2][0])*invdet;
    S_inv[2][1]=(S[0][1]*S[2][0]-S[0][0]*S[2][1])*invdet;
    S_inv[2][2]=(S[0][0]*S[1][1]-S[0][1]*S[1][0])*invdet;

    // K = (P_pred * C') / S;
    mat_mul_6x6X6x3(ekf->P, Ct, PCt);
    mat_mul_6x3X3x3(PCt, S_inv, K);

    // X = X + K*y
    for (int i=0;i<6;i++)
        for (int j=0;j<3;j++)
            ekf->X[i] += K[i][j]*y[j];

    // P_corr = (I - K*C) * P_pred * (I - K*C)' + K * R * K';
    quat_normalize(ekf->X);

    mat_mul_6x3X3x6(K, C, KC); //K*C
    for (int i=0;i<6;i++)
        for (int j=0;j<6;j++)
            A[i][j] = (i==j ? 1.0f : 0.0f) - KC[i][j]; //(I - K*C)

    mat_mul_6x6X6x6(A, ekf->P, AP); //(I - K*C) * P_pred
    float At[6][6];
    mat_transpose_6x6(A, At);
    mat_mul_6x6X6x6(AP, At, APA); // (I - K*C) * P_pred * (I - K*C)'

    //K * R * K'
    mat_mul_6x3X3x3(K, ekf->R, KR);
    float Kt[3][6];
    mat_transpose_6x3(K, Kt);
    mat_mul_6x3X3x6(KR, Kt, KRKt);

    for (int i=0;i<6;i++)
        for (int j=0;j<6;j++)
            ekf->P[i][j] = APA[i][j] + KRKt[i][j];

    mat_symmetrize_6x6(ekf->P);

}

void quad_to_euler(EKF_t *ekf, float *roll, float *pitch){
	float q0 = ekf->X[0];
	float q1 = ekf->X[1];
	float q2 = ekf->X[2];
	float q3 = ekf->X[3];

	*roll = atan2f(2*(q0 * q1 + q2 * q3), 1 - 2*(q1 * q1 + q2 * q2));
	*pitch = asinf(2*(q0 * q2 - q3 * q1));

}


