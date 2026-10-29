#pragma once
#include <Arduino.h>
#include "config.h"

class StaLtaDetector {
public:
    static constexpr int MAX_BUF = 2000;
    float buf[MAX_BUF];
    int head = 0;
    int count = 0;
    float sta = 0.0f;
    float lta = 0.0f;
    float ratio = 0.0f;
    bool active = false;
    float triggerThresh    = STA_LTA_TRIGGER;
    float deactivateThresh = STA_LTA_DEACTIVATE;

    void init() {
        head = 0; count = 0;
        sta = lta = ratio = 0.0f;
        active = false;
        for (int i = 0; i < MAX_BUF; i++) buf[i] = 0.0f;
    }
    void setThresholds(float trigger, float deactivate) {
        triggerThresh    = trigger;
        deactivateThresh = deactivate;
    }
    void push(float v) {
        buf[head] = v * v;
        head = (head + 1) % MAX_BUF;
        if (count < MAX_BUF) count++;
    }
    bool update() {
        int staLen = (int)(STA_WINDOW_SEC * SAMPLE_RATE_HZ);
        int ltaLen = (int)(LTA_WINDOW_SEC * SAMPLE_RATE_HZ);
        if (count < ltaLen) { ratio = 0.0f; return false; }
        sta = 0.0f;
        for (int i = 0; i < staLen; i++) {
            int p = (head - 1 - i + MAX_BUF) % MAX_BUF;
            sta += buf[p];
        }
        sta /= staLen;
        lta = 0.0f;
        for (int i = staLen; i < staLen + ltaLen; i++) {
            int p = (head - 1 - i + MAX_BUF) % MAX_BUF;
            lta += buf[p];
        }
        lta /= ltaLen;
        ratio = (lta > 1e-9f) ? (sta / lta) : 0.0f;
        if (!active && ratio > triggerThresh)      active = true;
        if (active && ratio < deactivateThresh)    active = false;
        return active;
    }
};

// ==================== 統合判定器（4票対応）====================
class FusionJudge {
public:
    bool quakeConfirmed = false;
    int votes = 0;
    float pgaTriggerGal  = PGA_TRIGGER_GAL;
    float seisIntTrigger = SEIS_INT_TRIGGER;
    int   requiredVotes  = REQUIRED_VOTES;

    void setThresholds(float pga, float intensity, int reqVotes) {
        pgaTriggerGal  = pga;
        seisIntTrigger = intensity;
        requiredVotes  = reqVotes;
    }

    // 4票目（fftVote）を追加
    bool update(bool staActive, bool fftVote, float pga, float intensity) {
        votes = 0;
        if (staActive)                  votes++;   // 1 STA/LTA
        if (fftVote)                    votes++;   // 2 FFT帯域エネルギー
        if (pga > pgaTriggerGal)        votes++;   // 3 PGA
        if (intensity > seisIntTrigger) votes++;   // 4 計測震度
        quakeConfirmed = (votes >= requiredVotes);
        return quakeConfirmed;
    }
};