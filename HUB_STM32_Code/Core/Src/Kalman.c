#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Kalman.h"

// Ð?nh nghia c?u trúc b? l?c Kalman cho t?a d? x, y
uint8_t kalman_Buffer[65];

// Kh?i t?o b? l?c Kalman
void initKalmanFilterXY(KalmanFilterXY *kf) {
    // Tr?ng thái ban d?u
    kf->x[0] = 0;  // x
    kf->x[1] = 0;  // y

    // Ma tr?n hi?p phuong sai ban d?u
    kf->P[0][0] = 1; kf->P[0][1] = 0;
    kf->P[1][0] = 0; kf->P[1][1] = 1;

    // Ma tr?n hi?p phuong sai nhi?u quá trình
    kf->Q[0][0] = 0.1; kf->Q[0][1] = 0;
    kf->Q[1][0] = 0; kf->Q[1][1] = 0.1;

    // Ma tr?n hi?p phuong sai nhi?u do lu?ng
    kf->R[0][0] = 0.1; kf->R[0][1] = 0;
    kf->R[1][0] = 0; kf->R[1][1] = 0.1;

    // Ma tr?n chuy?n d?i tr?ng thái (ví d?: gi? s? không có s? thay d?i tr?ng thái)
    kf->A[0][0] = 1; kf->A[0][1] = 0;
    kf->A[1][0] = 0; kf->A[1][1] = 1;

    // Ma tr?n do lu?ng (ví d?: do lu?ng tr?c ti?p các bi?n x, y)
    kf->H[0][0] = 1; kf->H[0][1] = 0;
    kf->H[1][0] = 0; kf->H[1][1] = 1;
}

// Bu?c d? doán c?a b? l?c Kalman
void predictXY(KalmanFilterXY *kf) {
    // D? doán tr?ng thái
    double x_pred[2];
    for (int i = 0; i < 2; ++i) {
        x_pred[i] = 0;
        for (int j = 0; j < 2; ++j) {
            x_pred[i] += kf->A[i][j] * kf->x[j];
        }
    }

    // D? doán ma tr?n hi?p phuong sai
    double P_pred[2][2];
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            P_pred[i][j] = 0;
            for (int k = 0; k < 2; ++k) {
                P_pred[i][j] += kf->A[i][k] * kf->P[k][j];
            }
            P_pred[i][j] += kf->Q[i][j];
        }
    }

    // C?p nh?t tr?ng thái và ma tr?n hi?p phuong sai
    for (int i = 0; i < 2; ++i) {
        kf->x[i] = x_pred[i];
        for (int j = 0; j < 2; ++j) {
            kf->P[i][j] = P_pred[i][j];
        }
    }
}

// Bu?c c?p nh?t c?a b? l?c Kalman
void updateXY(KalmanFilterXY *kf, double z[2]) {
    // Tính toán d?i m?i
    double y[2];
    for (int i = 0; i < 2; ++i) {
        y[i] = z[i];
        for (int j = 0; j < 2; ++j) {
            y[i] -= kf->H[i][j] * kf->x[j];
        }
    }

    // Tính toán ma tr?n hi?p phuong sai d?i m?i
    double S[2][2];
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            S[i][j] = kf->H[i][0] * kf->P[0][j] + kf->H[i][1] * kf->P[1][j] + kf->R[i][j];
        }
    }

    // Tính toán h? s? Kalman
    double K[2][2];
    K[0][0] = (kf->P[0][0] * kf->H[0][0] + kf->P[0][1] * kf->H[1][0]) / S[0][0];
    K[0][1] = (kf->P[0][0] * kf->H[0][1] + kf->P[0][1] * kf->H[1][1]) / S[0][0];
    K[1][0] = (kf->P[1][0] * kf->H[0][0] + kf->P[1][1] * kf->H[1][0]) / S[1][1];
    K[1][1] = (kf->P[1][0] * kf->H[0][1] + kf->P[1][1] * kf->H[1][1]) / S[1][1];

    // C?p nh?t tr?ng thái
    for (int i = 0; i < 2; ++i) {
        kf->x[i] += K[i][0] * y[0] + K[i][1] * y[1];
    }

    // C?p nh?t ma tr?n hi?p phuong sai
    kf->P[0][0] -= K[0][0] * kf->H[0][0] * kf->P[0][0] + K[0][1] * kf->H[1][0] * kf->P[0][0];
    kf->P[0][1] -= K[0][0] * kf->H[0][0] * kf->P[0][1] + K[0][1] * kf->H[1][0] * kf->P[0][1];
    kf->P[1][0] -= K[1][0] * kf->H[0][1] * kf->P[1][0] + K[1][1] * kf->H[1][1] * kf->P[1][0];
    kf->P[1][1] -= K[1][0] * kf->H[0][1] * kf->P[1][1] + K[1][1] * kf->H[1][1] * kf->P[1][1];
}

void processGPSData(double lat, double lon) {
    static KalmanFilterXY kf;
    static int initialized = 0;

    if (!initialized) {
        initKalmanFilterXY(&kf);
        initialized = 1;
    }

    double z[2] = {lat, lon};

    predictXY(&kf);
    updateXY(&kf, z);

    // S? d?ng u?c tính tr?ng thái dã l?c (kf.x)
    sprintf( (char *)kalman_Buffer,"(%f, %f)\n", kf.x[0], kf.x[1]);
}


