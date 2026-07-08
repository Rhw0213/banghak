#include <wiringPi.h>
#include <stdio.h>
#include "lidar.h"

#define TRIG_PIN  27   // D3 SIG (신호 보내기)
#define ECHO_PIN  22   // D2 SIG (신호 받기)

void lidar_init(void) {   // 인터페이스 이름 유지 (팀 계약)
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
    delay(30);
}

int lidar_read_distance(void) {   // cm 반환, 오류 시 -1
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long timeout = 30000;
    long count = 0;
    while (digitalRead(ECHO_PIN) == LOW) {
        if (++count > timeout) return -1;
        delayMicroseconds(1);
    }

    long start = micros();
    count = 0;
    while (digitalRead(ECHO_PIN) == HIGH) {
        if (++count > timeout) return -1;
        delayMicroseconds(1);
    }
    long travel_time = micros() - start;

    int distance = (int)(travel_time * 0.034 / 2);
    if (distance <= 0 || distance > 400) return -1;
    return distance;
}
