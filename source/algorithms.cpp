#include "algorithms.h"
#include <cstdlib>
#include <cmath>

// 내부 상태
static bool initialized = false;
static float x=0, y=0, theta=0;
static float P[3][3] = {
    {1,0,0},
    {0,1,0},
    {0,0,1}
};

static const float R_cov[2][2] = {{10,0},{0,10}};
static const float Q_cov[3][3] = {
    {0.1f, 0,     0    },
    {0,     0.1f, 0    },
    {0,     0,    0.05f}
};

static void matMul3(const float A[3][3], const float B[3][3], float C[3][3]) {
    for(int i=0;i<3;i++) for(int j=0;j<3;j++){
        float s=0;
        for(int k=0;k<3;k++) s+=A[i][k]*B[k][j];
        C[i][j]=s;
    }
}
static void matAdd3(const float A[3][3], const float B[3][3], float C[3][3]) {
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) C[i][j]=A[i][j]+B[i][j];
}

// 2×2 행렬 역행렬
static void inv2(const float M[2][2], float out[2][2]) {
    float det = M[0][0]*M[1][1] - M[0][1]*M[1][0];
    if(fabs(det)<1e-6f) det=1e-6f;
    out[0][0] =  M[1][1]/det;
    out[1][1] =  M[0][0]/det;
    out[0][1] = -M[0][1]/det;
    out[1][0] = -M[1][0]/det;
}

void updateKalmanFilter(float dt, float trueX, float trueY, float *estX, float *estY) {
    // 1) 초기화
    if(!initialized) {
        x = trueX;
        y = trueY;
        theta = 0;
        initialized = true;
    }

    // 2) 예측 단계: 직선 속도 v=0.05, 회전 속도 ω=0
    float v = 0.05f;
    float A[3][3] = {
        {1, 0, -v*dt*sinf(theta)},
        {0, 1,  v*dt*cosf(theta)},
        {0, 0, 1}
    };
    // x_pred
    float x_pred = x + v*dt*cosf(theta);
    float y_pred = y + v*dt*sinf(theta);
    float theta_pred = theta;

    // P_pred = A*P*Aᵀ + Q
    float AP[3][3], APA[3][3];
    matMul3(A,P,AP);
    // compute APA = AP * Aᵀ
    for(int i=0;i<3;i++) for(int j=0;j<3;j++){
        float s=0;
        for(int k=0;k<3;k++) s+=AP[i][k]*A[j][k];
        APA[i][j]=s;
    }
    matAdd3(APA, Q_cov, APA);

    // 3) 업데이트 단계: 측정 z = (trueX,trueY)
    // H = [[1,0,0],[0,1,0]]
    float H[2][3] = {{1,0,0},{0,1,0}};
    // S = H*P_pred*Hᵀ + R
    float HP[2][3] = {{APA[0][0],APA[0][1],APA[0][2]},
                      {APA[1][0],APA[1][1],APA[1][2]}};
    float S[2][2]={
        {HP[0][0]*H[0][0] + HP[0][1]*H[0][1] + R_cov[0][0],
         HP[0][0]*H[1][0] + HP[0][1]*H[1][1] + R_cov[0][1]},
        {HP[1][0]*H[0][0] + HP[1][1]*H[0][1] + R_cov[1][0],
         HP[1][0]*H[1][0] + HP[1][1]*H[1][1] + R_cov[1][1]}
    };
    float Sinv[2][2];
    inv2(S, Sinv);

    // K = P_pred * Hᵀ * S⁻¹  -> 3×2
    float PHt[3][2];
    for(int i=0;i<3;i++){
        PHt[i][0] = APA[i][0];
        PHt[i][1] = APA[i][1];
    }
    float K[3][2];
    for(int i=0;i<3;i++) for(int j=0;j<2;j++){
        K[i][j] = PHt[i][0]*Sinv[0][j] + PHt[i][1]*Sinv[1][j];
    }

    // 혁신 y = z - H*x_pred
    float y_innov[2] = { trueX - x_pred, trueY - y_pred };

    // 상태 업데이트
    x = x_pred + K[0][0]*y_innov[0] + K[0][1]*y_innov[1];
    y = y_pred + K[1][0]*y_innov[0] + K[1][1]*y_innov[1];
    theta = theta_pred;

    // 공분산 업데이트: P = (I - K*H)*P_pred
    float KH[3][3]={0}, I_KH[3][3], newP[3][3];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++){
        for(int k=0;k<2;k++) KH[i][j] += K[i][k]*H[k][j];
    }
    for(int i=0;i<3;i++) for(int j=0;j<3;j++){
        I_KH[i][j] = (i==j?1.0f:0) - KH[i][j];
    }
    matMul3(I_KH, APA, newP);
    // 복사
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) P[i][j]=newP[i][j];

    // 최종 추정치 반환
    *estX = x;
    *estY = y;
}
