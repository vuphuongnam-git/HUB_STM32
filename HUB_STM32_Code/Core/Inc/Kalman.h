#include "main.h"

typedef struct {
    double x[2];       // Vector tr?ng thái [x, y]
    double P[2][2];    // Ma tr?n hi?p phuong sai
    double Q[2][2];    // Ma tr?n hi?p phuong sai nhi?u quá trình
    double R[2][2];    // Ma tr?n hi?p phuong sai nhi?u do lu?ng
    double A[2][2];    // Ma tr?n chuy?n d?i tr?ng thái
    double H[2][2];    // Ma tr?n do lu?ng
    double K[2][2];    // H? s? Kalman
} KalmanFilterXY;

void initKalmanFilterXY(KalmanFilterXY *kf);
void predictXY(KalmanFilterXY *kf);
void updateXY(KalmanFilterXY *kf, double z[2]);
void processGPSData(double lat, double lon);
