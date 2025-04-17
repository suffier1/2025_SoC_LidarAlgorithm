#ifndef ALGORITHMS_H
#define ALGORITHMS_H

// 칼만 필터 하나만 함수로 선언
void updateKalmanFilter(float dt, float trueX, float trueY, float *estX, float *estY);

#endif // ALGORITHMS_H
