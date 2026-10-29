#pragma once
#include <Arduino.h>
#include <math.h>
#include <arduinoFFT.h>
#include "config.h"

struct Biquad {
    float a0 = 1.0f, a1 = 0.0f, a2 = 0.0f;
    float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f;
    float x1 = 0.0f, x2 = 0.0f, y1 = 0.0f, y2 = 0.0f;

    void reset() { x1 = x2 = y1 = y2 = 0.0f; }

    float process(float x) {
        float y = (b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2) / a0;
        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        return y;
    }
};

class Kunugi2012Filter {
public:
    Biquad stages[6];
    float  gain = 1.262f;

    void init(float fs) {
        const float PI2 = 2.0f * PI;
        const float dt  = 1.0f / fs;
        const float dt2 = dt * dt;

        {
            float w01 = PI2 * K2012_F0;
            float w02 = PI2 * K2012_F1;
            Biquad& bq = stages[0];
            bq.a0 = 8.0f/dt2 + (4.0f*w01 + 2.0f*w02)/dt + w01*w02;
            bq.a1 = 2.0f*w01*w02 - 16.0f/dt2;
            bq.a2 = 8.0f/dt2 - (4.0f*w01 + 2.0f*w02)/dt + w01*w02;
            bq.b0 = 4.0f/dt2 + 2.0f*w02/dt;
            bq.b1 = -8.0f/dt2;
            bq.b2 = 4.0f/dt2 - 2.0f*w02/dt;
            bq.reset();
        }
        {
            float w03 = PI2 * K2012_F1;
            Biquad& bq = stages[1];
            bq.a0 = 16.0f/dt2 + 17.0f*w03/dt + w03*w03;
            bq.a1 = 2.0f*w03*w03 - 32.0f/dt2;
            bq.a2 = 16.0f/dt2 - 17.0f*w03/dt + w03*w03;
            bq.b0 = 4.0f/dt2 + 8.5f*w03/dt + w03*w03;
            bq.b1 = 2.0f*w03*w03 - 8.0f/dt2;
            bq.b2 = 4.0f/dt2 - 8.5f*w03/dt + w03*w03;
            bq.reset();
        }
        {
            float w0  = PI2 * K2012_F2;
            float h01 = K2012_H2A;
            float h02 = K2012_H2B;
            Biquad& bq = stages[2];
            bq.a0 = 12.0f/dt2 + 12.0f*h02*w0/dt + w0*w0;
            bq.a1 = 10.0f*w0*w0 - 24.0f/dt2;
            bq.a2 = 12.0f/dt2 - 12.0f*h02*w0/dt + w0*w0;
            bq.b0 = 12.0f/dt2 + 12.0f*h01*w0/dt + w0*w0;
            bq.b1 = 10.0f*w0*w0 - 24.0f/dt2;
            bq.b2 = 12.0f/dt2 - 12.0f*h01*w0/dt + w0*w0;
            bq.reset();
        }
        const float fcs[] = {K2012_F3, K2012_F4, K2012_F5};
        const float hcs[] = {K2012_H3, K2012_H4, K2012_H5};
        for (int i = 0; i < 3; i++) {
            float wc = PI2 * fcs[i];
            float hc = hcs[i];
            Biquad& bq = stages[3 + i];
            bq.a0 = 12.0f/dt2 + 12.0f*hc*wc/dt + wc*wc;
            bq.a1 = 10.0f*wc*wc - 24.0f/dt2;
            bq.a2 = 12.0f/dt2 - 12.0f*hc*wc/dt + wc*wc;
            bq.b0 = wc*wc;
            bq.b1 = 10.0f*wc*wc;
            bq.b2 = wc*wc;
            bq.reset();
        }
        gain = K2012_GAIN;
    }

    float process(float x) {
        float y = x;
        for (int i = 0; i < 6; i++) y = stages[i].process(y);
        return y * gain;
    }

    void reset() {
        for (int i = 0; i < 6; i++) stages[i].reset();
    }
};

class SeismicIntensity {
public:
    Kunugi2012Filter filter;
    float peakAccel = 0.0f;
    float currentIntensity = 0.0f;
    float noiseFloor = 2.0f;
    float lastFilteredComp = 0.0f;

    static constexpr int RING_SIZE  = RING_BUFFER_SIZE;
    static constexpr int SORT_INDEX = RING_SORT_INDEX;

    float ringBuf[RING_SIZE];
    float sortedBuf[RING_SIZE];
    int   sampleCount = 0;
    int   head = 0;

    void init(float fs) {
        filter.init(fs);
        filter.reset();
        peakAccel = 0.0f; currentIntensity = 0.0f; noiseFloor = 2.0f;
        lastFilteredComp = 0.0f; sampleCount = 0; head = 0;
        for (int i = 0; i < RING_SIZE; i++) { ringBuf[i] = 0.0f; sortedBuf[i] = 0.0f; }
    }

    void measureNoiseFloor(float ox, float oy, float oz) {
        float compAvg = 0.0f, maxAvg = 0.0f;
        filter.reset();
        unsigned long t0 = millis();
        while (millis() - t0 < 1000) {
            if (M5.Imu.update()) {
                auto d = M5.Imu.getImuData();
                float fx = filter.process(d.accel.x*980.665f - ox);
                float fy = filter.process(d.accel.y*980.665f - oy);
                float fz = filter.process(d.accel.z*980.665f - oz);
                compAvg = 0.9f * compAvg + 0.1f * sqrtf(fx*fx + fy*fy + fz*fz);
            }
            delay(10); yield();
        }
        t0 = millis();
        while (millis() - t0 < 3000) {
            if (M5.Imu.update()) {
                auto d = M5.Imu.getImuData();
                float fx = filter.process(d.accel.x*980.665f - ox);
                float fy = filter.process(d.accel.y*980.665f - oy);
                float fz = filter.process(d.accel.z*980.665f - oz);
                float comp = sqrtf(fx*fx + fy*fy + fz*fz);
                compAvg = 0.9f * compAvg + 0.1f * comp;
                if (compAvg > maxAvg) maxAvg = compAvg;
            }
            delay(10); yield();
        }
        noiseFloor = maxAvg * 2.0f;
        if (noiseFloor < 0.5f) noiseFloor = 0.5f;
    }

    void insertSorted(float value) {
        int pos = RING_SIZE - 1;
        while (pos > 0 && sortedBuf[pos - 1] < value) { sortedBuf[pos] = sortedBuf[pos - 1]; pos--; }
        sortedBuf[pos] = value;
    }
    void removeSorted(float value) {
        int pos = -1;
        for (int i = 0; i < RING_SIZE; i++) if (fabsf(sortedBuf[i] - value) < 1e-6f) { pos = i; break; }
        if (pos < 0) return;
        for (int i = pos; i < RING_SIZE - 1; i++) sortedBuf[i] = sortedBuf[i + 1];
        sortedBuf[RING_SIZE - 1] = 0.0f;
    }

    float processSample(float ax, float ay, float az) {
        float fx = filter.process(ax);
        float fy = filter.process(ay);
        float fz = filter.process(az);
        float comp = sqrtf(fx * fx + fy * fy + fz * fz);
        lastFilteredComp = comp;

        if (sampleCount < RING_SIZE) {
            ringBuf[sampleCount] = comp;
            insertSorted(comp);
            sampleCount++;
        } else {
            float oldest = ringBuf[head];
            removeSorted(oldest);
            ringBuf[head] = comp;
            insertSorted(comp);
            head = (head + 1) % RING_SIZE;
        }

        if (sampleCount >= RING_SIZE) {
            float a = sortedBuf[SORT_INDEX];
            if (a < noiseFloor) { currentIntensity = 0.0f; peakAccel = 0.0f; }
            else {
                peakAccel = a;
                currentIntensity = 2.0f * log10f(a) + 0.94f;
                if (currentIntensity < 0.0f) currentIntensity = 0.0f;
            }
        } else { currentIntensity = 0.0f; peakAccel = 0.0f; }
        return currentIntensity;
    }

    float getInstantIntensity() const {
        if (lastFilteredComp <= noiseFloor) return 0.0f;
        float i = 2.0f * log10f(lastFilteredComp) + 0.94f;
        return (i < 0.0f) ? 0.0f : i;
    }
    void resetPeak() { peakAccel = 0.0f; currentIntensity = 0.0f; }
};

class SpectrumAnalyzer {
public:
    float vReal[FFT_SAMPLES];
    float vImag[FFT_SAMPLES];
    int   idx = 0;
    int   intervalCounter = 0;
    float bandEnergy   = 0.0f;
    float dominantFreq = 0.0f;
    bool  fftVote      = false;
    arduinoFFT* FFT = nullptr;

    void init() {
        FFT = new arduinoFFT(vReal, vImag, FFT_SAMPLES, SAMPLE_RATE_HZ);
        idx = 0; intervalCounter = 0; bandEnergy = 0.0f; dominantFreq = 0.0f; fftVote = false;
        for (int i = 0; i < FFT_SAMPLES; i++) { vReal[i] = 0.0f; vImag[i] = 0.0f; }
    }

    void pushSample(float v) {
        vReal[idx] = v; vImag[idx] = 0.0f;
        idx++; intervalCounter++;
        if (idx >= FFT_SAMPLES) { idx = 0; analyze(); intervalCounter = 0; }
        else if (intervalCounter >= FFT_INTERVAL && idx >= FFT_SAMPLES / 2) { intervalCounter = 0; }
    }

    void analyze() {
        if (!FFT) return;
        FFT->Windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
        FFT->Compute(FFT_FORWARD);
        FFT->ComplexToMagnitude();
        int bins = FFT_SAMPLES / 2;
        float binHz = (float)SAMPLE_RATE_HZ / (float)FFT_SAMPLES;
        bandEnergy = 0.0f; float maxMag = 0.0f; dominantFreq = 0.0f;
        for (int i = 1; i < bins; i++) {
            float freq = i * binHz;
            float mag = vReal[i];
            if (freq >= QUAKE_FREQ_LOW && freq <= QUAKE_FREQ_HIGH) bandEnergy += mag * mag;
            if (mag > maxMag) { maxMag = mag; dominantFreq = freq; }
        }
        bandEnergy = sqrtf(bandEnergy);
        fftVote = (bandEnergy > FFT_NOISE_THRESHOLD);
    }
    void reset() { init(); }
};

inline const char* intensityToShindo(float intensity) {
    if (intensity < 0.5f) return "0";
    if (intensity < 1.5f) return "1";
    if (intensity < 2.5f) return "2";
    if (intensity < 3.5f) return "3";
    if (intensity < 4.5f) return "4";
    if (intensity < 5.0f) return "5-";
    if (intensity < 5.5f) return "5+";
    if (intensity < 6.0f) return "6-";
    if (intensity < 6.5f) return "6+";
    return "7";
}

inline uint16_t shindoColor(float intensity) {
    if (intensity < 3.5f) return TFT_GREEN;
    if (intensity < 4.5f) return TFT_YELLOW;
    if (intensity < 5.0f) return TFT_ORANGE;
    if (intensity < 6.0f) return TFT_RED;
    return TFT_MAGENTA;
}
