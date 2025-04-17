#include "mbed.h"
#include "algorithms.h"
#include "ThisThread.h"
#include <chrono>

using namespace std::chrono_literals;

// 시간 측정용 타이머
Timer timer;

// 실제 vs 추정 위치
float trueX = 0, trueY = 0, estX = 0, estY = 0;

// 루프 주기 20ms
static constexpr auto PERIOD = 20ms;

int main() {
    // semihosting(printf) 로 출력
    printf("soc_lidar_real (KF) 시작\r\n");

    timer.start();

    while (true) {
        float dt = PERIOD.count() / 1000.0f;  // ms → s

        // 1) 실제 위치 갱신 (dead reckoning, heading=0 고정)
        const float speed = 0.05f;
        trueX += speed * dt;
        // trueY 은 항상 0 으로 직진

        // 2) 칼만 필터 실행
        updateKalmanFilter(dt, trueX, trueY, &estX, &estY);

        // 3) semihosting 으로 CSV 출력
        // printf 의 %lld 로 long long 출력
        printf("%lld,%.3f,%.3f,%.3f,%.3f\r\n",
            static_cast<long long>(timer.elapsed_time().count()),
            trueX, trueY,
            estX, estY
        );

        ThisThread::sleep_for(PERIOD);
    }
}
