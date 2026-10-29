// ╔═══════════════════════════════════════════════════════════════════════╗
// ║  PoMaS (Pocket Math Station) ver.0.92 ─ 完全実装版                    ║
// ║  Target : M5Stack Cardputer-Adv (ESP32-S3FN8, 8MB Flash)              ║
// ║  Created: YNGMT (original spec ver.7.1)                               ║
// ║  Part  1 / 20 ─ 共通ヘッダ・型・GState・補助関数                      ║
// ╚═══════════════════════════════════════════════════════════════════════╝

#include <M5Cardputer.h>
#include <M5GFX.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include <ctype.h>
#include <stdarg.h>
#include <string.h>

#include "splash_data.h"

#define POMAS_VER  "0.92"
#define POMAS_NAME "PoMaS"
#define POMAS_CRED "Created by YNGMT"

struct LawEntry {
    const char* name;
    const char* formula;
    const char* notes;
    const char* cat;
};

#define SD_CS    12
#define SD_MOSI  14
#define SD_SCK   40
#define SD_MISO  39

struct SdGuard {
    SdGuard() { M5Cardputer.Display.endWrite(); }
};

static const int LCD_W   = 240;
static const int LCD_H   = 135;
static const int HDR_H   = 16;
static const int FTR_H   = 14;
static const int BODY_X  = 2;

#define C_BG       0x3186
#define C_PANEL    0x18C3
#define C_HDR      0xD69A
#define C_HDRTX    0x0000
#define C_HDRDIM   0x4228
#define C_FG       0xFFFF
#define C_DIM      0xB5B6
#define C_ACC      0x07FF
#define C_ACC2     0xFB40
#define C_OK       0x2FE5
#define C_WARN     0xFEE0
#define C_ERR      0xF804
#define C_SEL_BG   0x0256
#define C_SEL_BAR  0x07FF
#define C_GRID     0x52AA

#define S_BG       0x10A4
#define S_HDR      0x18E3
#define S_FG       0xEF5D
#define S_DIM      0x6B6D
#define S_ACC      0x07FF
#define S_ACC2     0xFD20

#define KEY_BS    0x08
#define IS_ESC    M5Cardputer.Keyboard.isKeyPressed('`')
#define IS_UP     M5Cardputer.Keyboard.isKeyPressed(';')
#define IS_DOWN   M5Cardputer.Keyboard.isKeyPressed('.')
#define IS_LEFT   M5Cardputer.Keyboard.isKeyPressed(',')
#define IS_RIGHT  M5Cardputer.Keyboard.isKeyPressed('/')

#define LIST_MAX        999
#define HIST_MAX        100
#define UNDO_MAX        20
#define VAR_COUNT       10
#define MAT_MAX         10
#define VEC_MAX         10
#define STAT_LISTS      8
#define ANS_BANK_MAX    20
#define MULTIREG_MAX    5
#define META_MAX        50
#define SURVIVAL_MAX    500
#define OUTBREAK_MAX    100
#define HCLUST_MAX      200
#define KMEANS_MAX      500
#define FA_MAX_VAR      10
#define INTEG_MAX_N     1000
#define PRIME_LIMIT     500000
#define SS_ROWS         45
#define SS_COLS         26
#define ULISP_CELLS     2500

enum AppMode   { MODE_MENU=0, MODE_CALC, MODE_CAS, MODE_STAT, MODE_SCIREF, MODE_GRAPH };
enum AngleMode { DEG=0, RAD=1, GRAD=2 };
enum DispMode  { DISP_NORM=0, DISP_FIX, DISP_SCI, DISP_ENG };
enum BaseMode  { BASE_DEC=10, BASE_HEX=16, BASE_OCT=8, BASE_BIN=2 };
enum VarScale  { SCALE_RATIO=0, SCALE_INTERVAL, SCALE_ORDINAL, SCALE_NOMINAL };
enum InputMode { INPUT_ALG=0, INPUT_RPN };
enum TestSide  { TS_TWO=0, TS_UPPER, TS_LOWER };

struct Fraction {
    long long num;
    long long den;
    Fraction() : num(0), den(1) {}
    Fraction(long long n) : num(n), den(1) {}
    Fraction(long long n, long long d) {
        if (d == 0) { num = 0; den = 1; return; }
        if (d < 0) { n = -n; d = -d; }
        num = n; den = d;
        reduce();
    }
    static long long gcdLL(long long a, long long b) {
        if (a < 0) a = -a;
        if (b < 0) b = -b;
        while (b) { long long t = a % b; a = b; b = t; }
        return a ? a : 1;
    }
    void reduce() {
        if (den < 0) { num = -num; den = -den; }
        long long g = gcdLL(num, den);
        if (g > 1) { num /= g; den /= g; }
        if (num == 0) den = 1;
    }
    Fraction operator+(const Fraction& o) const { return Fraction(num*o.den + o.num*den, den*o.den); }
    Fraction operator-(const Fraction& o) const { return Fraction(num*o.den - o.num*den, den*o.den); }
    Fraction operator*(const Fraction& o) const { return Fraction(num*o.num, den*o.den); }
    Fraction operator/(const Fraction& o) const { return Fraction(num*o.den, den*o.num); }
    bool     operator==(const Fraction& o) const { return num*o.den == o.num*den; }
    Fraction neg() const { return Fraction(-num, den); }
    double   toDouble() const { return (double)num / (double)den; }
    void toString(char* out, int n, bool mixed = false) const {
        if (den == 1) { snprintf(out, n, "%lld", num); return; }
        if (mixed && (num >= den || num <= -den)) {
            long long w = num / den;
            long long r = num % den; if (r < 0) r = -r;
            snprintf(out, n, "%lld %lld/%lld", w, r, den);
        } else {
            snprintf(out, n, "%lld/%lld", num, den);
        }
    }
    static Fraction fromDouble(double v, double tol = 1e-9, long long maxDen = 1000000LL) {
        if (isnan(v) || isinf(v)) return Fraction(0,1);
        bool neg = v < 0; if (neg) v = -v;
        long long h0=0, h1=1, k0=1, k1=0;
        double x = v;
        for (int i = 0; i < 30; i++) {
            long long a = (long long)floor(x);
            long long h2 = a*h1 + h0;
            long long k2 = a*k1 + k0;
            if (k2 > maxDen) break;
            h0 = h1; h1 = h2;
            k0 = k1; k1 = k2;
            double diff = x - a;
            if (diff < tol) break;
            x = 1.0 / diff;
            if (x > 1e15) break;
        }
        return Fraction(neg ? -h1 : h1, k1);
    }
};

struct Complex {
    double re, im;
    Complex() : re(0), im(0) {}
    Complex(double r) : re(r), im(0) {}
    Complex(double r, double i) : re(r), im(i) {}
    Complex operator+(const Complex& o) const { return Complex(re+o.re, im+o.im); }
    Complex operator-(const Complex& o) const { return Complex(re-o.re, im-o.im); }
    Complex operator*(const Complex& o) const { return Complex(re*o.re-im*o.im, re*o.im+im*o.re); }
    Complex operator/(const Complex& o) const {
        double d = o.re*o.re + o.im*o.im;
        return Complex((re*o.re+im*o.im)/d, (im*o.re-re*o.im)/d);
    }
    Complex conj() const { return Complex(re, -im); }
    double  abs()  const { return sqrt(re*re + im*im); }
    double  arg()  const { return atan2(im, re); }
    static Complex fromPolar(double r, double theta) {
        return Complex(r*cos(theta), r*sin(theta));
    }
    Complex powN(double n) const { return fromPolar(::pow(abs(), n), arg()*n); }
    Complex sqrtC()        const { return fromPolar(sqrt(abs()), arg()*0.5); }
    Complex expC()         const { return Complex(::exp(re)*cos(im), ::exp(re)*sin(im)); }
    Complex logC()         const { return Complex(::log(abs()), arg()); }
    Complex sinC() const { return Complex(::sin(re)*::cosh(im), ::cos(re)*::sinh(im)); }
    Complex cosC() const { return Complex(::cos(re)*::cosh(im), -::sin(re)*::sinh(im)); }
    void toString(char* out, int n, bool polar = false) const {
        if (polar) {
            snprintf(out, n, "%g\xE2\x88\xA0%g", abs(), arg());
        } else {
            if (fabs(im) < 1e-12)      snprintf(out, n, "%g", re);
            else if (fabs(re) < 1e-12) snprintf(out, n, "%gi", im);
            else if (im > 0)            snprintf(out, n, "%g+%gi", re, im);
            else                        snprintf(out, n, "%g%gi", re, im);
        }
    }
};

#define POLY_MAX 16
struct Polynomial {
    double c[POLY_MAX+1];
    int    deg;
    Polynomial() : deg(0) { for (int i=0;i<=POLY_MAX;i++) c[i]=0; }
    void   normalize() {
        deg = 0;
        for (int i=POLY_MAX; i>=0; i--) if (fabs(c[i]) > 1e-15) { deg=i; break; }
    }
    double  eval(double x) const {
        double r = 0;
        for (int i=deg; i>=0; i--) r = r*x + c[i];
        return r;
    }
    Complex evalC(Complex x) const {
        Complex r;
        for (int i=deg; i>=0; i--) r = r*x + Complex(c[i],0);
        return r;
    }
    Polynomial diff() const {
        Polynomial r;
        for (int i=1; i<=deg; i++) r.c[i-1] = c[i] * i;
        r.normalize();
        return r;
    }
    Polynomial integ() const {
        Polynomial r;
        for (int i=0; i<=deg && i<POLY_MAX; i++) r.c[i+1] = c[i] / (i+1.0);
        r.normalize();
        return r;
    }
    Polynomial operator+(const Polynomial& o) const {
        Polynomial r;
        int m = (deg > o.deg) ? deg : o.deg;
        for (int i=0;i<=m;i++) r.c[i] = c[i] + o.c[i];
        r.normalize();
        return r;
    }
    Polynomial operator-(const Polynomial& o) const {
        Polynomial r;
        int m = (deg > o.deg) ? deg : o.deg;
        for (int i=0;i<=m;i++) r.c[i] = c[i] - o.c[i];
        r.normalize();
        return r;
    }
    Polynomial operator*(const Polynomial& o) const {
        Polynomial r;
        for (int i=0; i<=deg; i++)
            for (int j=0; j<=o.deg && i+j<=POLY_MAX; j++)
                r.c[i+j] += c[i] * o.c[j];
        r.normalize();
        return r;
    }
    void toString(char* out, int n) const {
        out[0] = 0;
        bool first = true;
        for (int i=deg; i>=0; i--) {
            if (fabs(c[i]) < 1e-12) continue;
            char term[40];
            const char* sg = "";
            double v = c[i];
            if (first) { sg = (v<0)?"-":""; first = false; }
            else       { sg = (v<0)?" - ":" + "; }
            double a = fabs(v);
            if (i == 0)            snprintf(term,sizeof(term),"%s%g", sg, a);
            else if (i == 1) {
                if (fabs(a-1) < 1e-9) snprintf(term,sizeof(term),"%sx", sg);
                else                  snprintf(term,sizeof(term),"%s%gx", sg, a);
            } else {
                if (fabs(a-1) < 1e-9) snprintf(term,sizeof(term),"%sx^%d", sg, i);
                else                  snprintf(term,sizeof(term),"%s%gx^%d", sg, a, i);
            }
            strncat(out, term, n - strlen(out) - 1);
        }
        if (first) snprintf(out, n, "0");
    }
};

struct HistoryEntry {
    char    expr[80];
    double  value;
    bool    ok;
    bool    isComplex;
    double  imag;
    HistoryEntry() : value(0), ok(true), isComplex(false), imag(0) { expr[0]=0; }
};

struct StatColumn {
    char     name[12];
    VarScale scale;
    int      count;
    double*  data;
    double*  freq;
};

struct GState {
    AppMode    mode = MODE_MENU;
    AngleMode  angle = DEG;
    DispMode   disp = DISP_NORM;
    int        dispDigits = 4;
    int        normMode = 1;
    BaseMode   base = BASE_DEC;
    bool       rpn = false;
    bool       sdOK = false;
    bool       psramOK = false;
    bool       wifiOK = false;
    bool       fracDisplay = false;
    char       wifiSSID[33] = {0};
    char       wifiPASS[33] = {0};

    double  ans  = 0;
    Complex ansC = {0,0};
    bool    ansComplex = false;
    Fraction ansF;
    bool    ansFracValid = false;
    double  mem  = 0;
    double  vars[VAR_COUNT] = {0};

    double bank[ANS_BANK_MAX] = {0};
    char   bankLabel[ANS_BANK_MAX][16] = {{0}};
    int    bankN = 0;

    HistoryEntry hist[HIST_MAX];
    int          histN = 0;

    double rpnStk[4] = {0,0,0,0};
    int    rpnSp = 0;

    double  vecA[VEC_MAX] = {0};
    double  vecB[VEC_MAX] = {0};
    int     vecAdim = 3, vecBdim = 3;
    double  matA[MAT_MAX*MAT_MAX] = {0};
    double  matB[MAT_MAX*MAT_MAX] = {0};
    int     matAr = 0, matAc = 0;
    int     matBr = 0, matBc = 0;

    StatColumn stat[STAT_LISTS];
    int        statCapacity = 0;

    char*  ssData[SS_ROWS][SS_COLS] = {{nullptr}};
    bool   ssInitialized = false;

    char   cwd[64] = "/";

    bool allocStat(int cap = LIST_MAX) {
        freeStat();
        for (int i = 0; i < STAT_LISTS; i++) {
            stat[i].data = (double*)ps_malloc(cap * sizeof(double));
            stat[i].freq = nullptr;
            if (!stat[i].data) {
                stat[i].data = (double*)malloc(cap * sizeof(double));
            }
            if (!stat[i].data) { freeStat(); return false; }
            memset(stat[i].data, 0, cap * sizeof(double));
            snprintf(stat[i].name, sizeof(stat[i].name), "L%d", i+1);
            stat[i].scale = SCALE_RATIO;
            stat[i].count = 0;
        }
        statCapacity = cap;
        return true;
    }
    void freeStat() {
        for (int i = 0; i < STAT_LISTS; i++) {
            if (stat[i].data) { free(stat[i].data); stat[i].data = nullptr; }
            if (stat[i].freq) { free(stat[i].freq); stat[i].freq = nullptr; }
            stat[i].count = 0;
        }
        statCapacity = 0;
    }
    void freeSS() {
        for (int r=0; r<SS_ROWS; r++)
            for (int c=0; c<SS_COLS; c++)
                if (ssData[r][c]) { free(ssData[r][c]); ssData[r][c] = nullptr; }
        ssInitialized = false;
    }
    void setSS(int r, int c, const char* expr) {
        if (r<0 || r>=SS_ROWS || c<0 || c>=SS_COLS) return;
        if (ssData[r][c]) { free(ssData[r][c]); ssData[r][c] = nullptr; }
        if (expr && *expr) {
            int n = strlen(expr) + 1;
            ssData[r][c] = (char*)malloc(n);
            if (ssData[r][c]) strcpy(ssData[r][c], expr);
        }
    }
    void pushHist(const char* expr, double v, bool ok = true,
                  bool isC = false, double imag = 0) {
        int idx;
        if (histN < HIST_MAX) idx = histN++;
        else {
            for (int i=1; i<HIST_MAX; i++) hist[i-1] = hist[i];
            idx = HIST_MAX - 1;
        }
        strncpy(hist[idx].expr, expr, 79); hist[idx].expr[79] = 0;
        hist[idx].value = v;
        hist[idx].ok = ok;
        hist[idx].isComplex = isC;
        hist[idx].imag = imag;
    }
    void resetVars() { for (int i=0;i<VAR_COUNT;i++) vars[i]=0; mem=0; }
    void resetStat() { for (int i=0;i<STAT_LISTS;i++) stat[i].count=0; }
    void resetAll()  { resetVars(); resetStat(); histN=0; rpnSp=0; ans=0; }
};
GState G;

inline double toRad(double x) {
    if (G.angle == DEG)  return x * M_PI / 180.0;
    if (G.angle == GRAD) return x * M_PI / 200.0;
    return x;
}
inline double fromRad(double x) {
    if (G.angle == DEG)  return x * 180.0 / M_PI;
    if (G.angle == GRAD) return x * 200.0 / M_PI;
    return x;
}

inline const char* siPrefix(int em) {
    static const char* P[] = {"y","z","a","f","p","n","u","m","","k","M","G","T","P","E","Z","Y"};
    int k = em / 3 + 8;
    if (k < 0) k = 0;
    if (k > 16) k = 16;
    return P[k];
}

inline void formatNum(double v, char* out, int n) {
    if (isnan(v)) { snprintf(out,n,"NaN"); return; }
    if (isinf(v)) { snprintf(out,n,v<0?"-Inf":"Inf"); return; }
    int d = G.dispDigits;
    if (d < 0) d = 0; if (d > 9) d = 9;
    switch (G.disp) {
    case DISP_FIX:
        snprintf(out, n, "%.*f", d, v);
        break;
    case DISP_SCI:
        snprintf(out, n, "%.*e", d, v);
        break;
    case DISP_ENG: {
        if (v == 0) { snprintf(out,n,"0"); break; }
        double a = fabs(v);
        int ex = (int)floor(log10(a));
        int em = (ex>=0) ? (ex/3)*3 : -(((-ex)+2)/3)*3;
        double m = v / pow(10.0, em);
        const char* sp = siPrefix(em);
        if (*sp)  snprintf(out, n, "%.*f%s",  d, m, sp);
        else      snprintf(out, n, "%.*fe%d", d, m, em);
    } break;
    default: {
        double lim_lo = (G.normMode == 2) ? 1e-9 : 1e-2;
        double lim_hi = 1e10;
        double a = fabs(v);
        if (a == 0 || (a >= lim_lo && a < lim_hi))
            snprintf(out, n, "%.*g", d+6, v);
        else
            snprintf(out, n, "%.*e", d+2, v);
    } break;
    }
}
inline String numStr(double v) { char b[40]; formatNum(v,b,40); return String(b); }

inline void formatInt(long long v, char* out, int n, BaseMode b) {
    switch (b) {
    case BASE_HEX: snprintf(out, n, "0x%llX", (unsigned long long)v); break;
    case BASE_OCT: snprintf(out, n, "0o%llo", (unsigned long long)v); break;
    case BASE_BIN: {
        unsigned long long u = (unsigned long long)v;
        char buf[68]; int p = 0;
        if (u == 0) { strncpy(out, "0b0", n); return; }
        while (u && p < 64) { buf[p++] = '0' + (int)(u & 1ULL); u >>= 1; }
        int o = 0;
        if (o < n-1) out[o++] = '0';
        if (o < n-1) out[o++] = 'b';
        while (p-- && o < n-1) out[o++] = buf[p];
        out[o] = 0;
    } break;
    default: snprintf(out, n, "%lld", v); break;
    }
}

inline bool tryDecimalToSpecial(double v, char* out, int n) {
    if (!isfinite(v) || v == 0) return false;
    double a = fabs(v);
    double rp = a / M_PI;
    if (a < 1e6 && fabs(rp - round(rp)) < 1e-9) {
        long long k = (long long)round(rp);
        if (k != 0) {
            if (v < 0)  snprintf(out, n, "-%lld\xCF\x80", k);
            else        snprintf(out, n,  "%lld\xCF\x80", k);
            return true;
        }
    }
    double rp2 = (2*a) / M_PI;
    if (a < 1e6 && fabs(rp2 - round(rp2)) < 1e-9 && (long long)round(rp2)%2!=0) {
        long long k = (long long)round(rp2);
        if (v < 0) snprintf(out, n, "-%lld\xCF\x80/2", k);
        else       snprintf(out, n,  "%lld\xCF\x80/2", k);
        return true;
    }
    for (int rt = 2; rt < 100; rt++) {
        double sq = sqrt((double)rt);
        double k = a / sq;
        if (k < 100 && fabs(k - round(k)) < 1e-9) {
            long long ki = (long long)round(k);
            if (ki != 0) {
                if (v < 0)  snprintf(out, n, "-%lld\xE2\x88\x9A%d", ki, rt);
                else        snprintf(out, n,  "%lld\xE2\x88\x9A%d", ki, rt);
                return true;
            }
        }
    }
    Fraction f = Fraction::fromDouble(v, 1e-9, 100000);
    if (fabs(f.toDouble() - v) < 1e-9 && f.den > 1) {
        f.toString(out, n);
        return true;
    }
    return false;
}

inline int textWidth(const char* s) {
    return strlen(s) * 6;
}

struct KSum {
    double s = 0.0, c = 0.0;
    inline void add(double x) {
        double t = s + x;
        if (fabs(s) >= fabs(x)) c += (s - t) + x;
        else                    c += (x - t) + s;
        s = t;
    }
    inline double get() const { return s + c; }
};

class DisplayMgr;
class ExprEval;
extern DisplayMgr DM;
extern ExprEval   gEv;
extern unsigned long gLastInputT;

// ╔═══════════════════════════════════════════════════════════════════════╗
// ║  Part 2 / 20 ─ DisplayMgr                                             ║
// ╚═══════════════════════════════════════════════════════════════════════╝

class DisplayMgr {
public:
    inline bool pollKeys() {
        M5Cardputer.update();
        if (M5Cardputer.Keyboard.isChange()) {
            gLastInputT = millis();
            return true;
        }
        return false;
    }

    void begin() {
        auto& d = M5Cardputer.Display;
        d.setRotation(1);
        d.setColorDepth(16);
        d.fillScreen(C_BG);
        d.setFont(&fonts::Font0);
        d.setTextSize(1);
        d.setTextColor(C_FG, C_BG);
    }

    void splash() {
        auto& d = M5Cardputer.Display;
#ifdef SPLASH_RLE
  #if SPLASH_RLE == 1
        int px = 0, py = 0;
        for (int i = 0; i < SPLASH_RLE_LEN; i++) {
            uint8_t cnt = pgm_read_byte(&splash_rle_count[i]);
            uint16_t val = pgm_read_word(&splash_rle_value[i]);
            for (int k = 0; k < cnt; k++) {
                if (px < SPLASH_W && py < SPLASH_H) d.drawPixel(px, py, val);
                px++;
                if (px >= SPLASH_W) { px = 0; py++; }
            }
        }
  #else
        for (int y = 0; y < SPLASH_H; y++)
            for (int x = 0; x < SPLASH_W; x++) {
                uint16_t v = pgm_read_word(&splash_data[y * SPLASH_W + x]);
                d.drawPixel(x, y, v);
            }
  #endif
        d.setTextDatum(bottom_left);
        d.setTextSize(1);
        d.setTextColor(S_FG, S_BG);
        d.drawString("Ver. " POMAS_VER, LCD_W - 236, LCD_H - 2);
        d.setTextDatum(top_left);
#else
        d.fillScreen(S_BG);
        for (int y = 0; y < HDR_H; y++)        d.drawFastHLine(0, y,            LCD_W, S_HDR);
        for (int y = LCD_H-FTR_H; y < LCD_H; y++) d.drawFastHLine(0, y,         LCD_W, S_HDR);
        d.setTextDatum(middle_center);
        d.setTextSize(2);
        d.setTextColor(S_ACC, S_BG);
        char title[32]; snprintf(title, sizeof(title), POMAS_NAME " ver." POMAS_VER);
        d.drawString(title, LCD_W/2, LCD_H/2 - 12);
        d.setTextSize(1);
        d.setTextColor(S_DIM, S_BG);
        d.drawString(POMAS_CRED, LCD_W/2, LCD_H/2 + 8);
        d.drawFastHLine(LCD_W/2 - 30, LCD_H/2 + 22, 60, S_ACC2);
        d.drawFastHLine(LCD_W/2 - 20, LCD_H/2 + 25, 40, S_ACC);
        d.setTextDatum(top_left);
#endif
        delay(2500);
        d.fillScreen(C_BG);
    }

    void header(const char* title) {
        auto& d = M5Cardputer.Display;
        d.fillRect(0, 0, LCD_W, HDR_H, C_HDR);
        d.fillRect(0, HDR_H-1, LCD_W, 1, C_ACC);
        d.setTextColor(C_HDRTX, C_HDR);
        d.setTextDatum(top_left);
        d.drawString(title, 4, 3);
        char bd[40];
        const char* ang = (G.angle==DEG)?"DEG":(G.angle==RAD?"RAD":"GRA");
        const char* dsp =
            (G.disp==DISP_FIX)?"FIX":
            (G.disp==DISP_SCI)?"SCI":
            (G.disp==DISP_ENG)?"ENG":"NRM";
        snprintf(bd, sizeof(bd), "%s|%s%s%s",
                 ang, dsp, G.rpn?"|RPN":"",
                 G.base==BASE_DEC?"":(G.base==BASE_HEX?"|HEX":(G.base==BASE_OCT?"|OCT":"|BIN")));
        int bw = textWidth(bd);
        d.setTextColor(C_HDRDIM, C_HDR);
        d.drawString(bd, LCD_W - bw - 4, 3);
    }

    void footer(const char* l = "", const char* c = "", const char* r = "") {
        auto& d = M5Cardputer.Display;
        d.fillRect(0, LCD_H - FTR_H, LCD_W, FTR_H, C_HDR);
        d.fillRect(0, LCD_H - FTR_H, LCD_W, 1, C_ACC);
        d.setTextColor(C_HDRDIM, C_HDR);
        d.setTextDatum(top_left);
        if (l && *l) d.drawString(l, 4, LCD_H - FTR_H + 3);
        if (c && *c) {
            int cw = textWidth(c);
            d.drawString(c, (LCD_W - cw)/2, LCD_H - FTR_H + 3);
        }
        if (r && *r) {
            int rw = textWidth(r);
            d.drawString(r, LCD_W - rw - 4, LCD_H - FTR_H + 3);
        }
    }

    void clearBody() {
        auto& d = M5Cardputer.Display;
        d.fillRect(0, HDR_H, LCD_W, LCD_H - HDR_H - FTR_H, C_BG);
    }

    void notify(const char* msg, uint16_t color = C_ACC, int ms = 800) {
        auto& d = M5Cardputer.Display;
        int w = textWidth(msg) + 16;
        if (w > LCD_W - 8) w = LCD_W - 8;
        int x = (LCD_W - w)/2, y = LCD_H/2 - 10;
        d.fillRect(x, y, w, 22, C_PANEL);
        d.drawRect(x, y, w, 22, color);
        d.setTextColor(color, C_PANEL);
        d.setTextDatum(middle_center);
        d.drawString(msg, LCD_W/2, y + 10);
        d.setTextDatum(top_left);
        delay(ms);
    }

    bool confirm(const char* msg) {
        auto& d = M5Cardputer.Display;
        int w = textWidth(msg) + 24;
        if (w > LCD_W - 8) w = LCD_W - 8;
        int x = (LCD_W - w)/2, y = LCD_H/2 - 24;
        d.fillRect(x, y, w, 50, C_PANEL);
        d.drawRect(x, y, w, 50, C_WARN);
        d.setTextColor(C_FG, C_PANEL);
        d.setTextDatum(middle_center);
        d.drawString(msg, LCD_W/2, y + 14);
        d.setTextColor(C_OK, C_PANEL);
        d.drawString("[Y] Yes", LCD_W/2 - 28, y + 36);
        d.setTextColor(C_ERR, C_PANEL);
        d.drawString("[N] No",  LCD_W/2 + 28, y + 36);
        d.setTextDatum(top_left);
        while (true) {
            if (pollKeys()) {
                if (M5Cardputer.Keyboard.isKeyPressed('y') ||
                    M5Cardputer.Keyboard.isKeyPressed('Y')) return true;
                if (M5Cardputer.Keyboard.isKeyPressed('n') ||
                    M5Cardputer.Keyboard.isKeyPressed('N')) return false;
                if (IS_ESC) return false;
            }
            delay(15);
        }
    }

    void waitEsc(const char* hint = "ESC: back") {
        footer("", "", hint);
        while (true) {
            if (pollKeys() && IS_ESC) break;
            delay(15);
        }
    }

    bool inputLine(const char* prompt, char* buf, int maxLen, int y = 0) {
        auto& d = M5Cardputer.Display;
        int yy = (y > 0) ? y : LCD_H/2 - 18;
        int w = LCD_W - 8;
        d.fillRect(4, yy, w, 32, C_PANEL);
        d.drawRect(4, yy, w, 32, C_ACC);
        d.setTextColor(C_DIM, C_PANEL);
        d.setTextDatum(top_left);
        d.drawString(prompt, 8, yy + 3);
        bool blink = false;
        unsigned long lastBlink = millis();
        int cur = strlen(buf);
        while (true) {
            d.fillRect(8, yy + 14, w - 12, 14, C_PANEL);
            d.setTextColor(C_FG, C_PANEL);
            d.drawString(buf, 8, yy + 16);
            int cx = 8 + textWidth(buf);
            if (cx > w - 6) cx = w - 6;
            if (blink) d.drawFastVLine(cx, yy + 14, 12, C_ACC);
            unsigned long t0 = millis();
            while (millis() - t0 < 80) {
                if (pollKeys()) {
                    auto st = M5Cardputer.Keyboard.keysState();
                    if (st.fn && IS_ESC) return false;
                    if (st.enter) return true;
                    if (st.del && cur > 0) { buf[--cur] = 0; goto rd; }
                    for (auto c : st.word) {
                        if (cur < maxLen-1 && c >= ' ' && c < 127) {
                            buf[cur++] = c; buf[cur] = 0;
                        }
                    }
                    if (!st.word.empty()) goto rd;
                }
                delay(15);
            }
            if (millis() - lastBlink > 400) { blink = !blink; lastBlink = millis(); }
            rd:;
        }
    }

    int menu(const char* title, const char* items[], int n, int init = 0) {
        auto& d = M5Cardputer.Display;
        int sel = init;
        if (sel < 0) sel = 0;
        if (sel >= n) sel = n-1;
        int rowH = 12;
        int viewH = LCD_H - HDR_H - FTR_H;
        int rows = viewH / rowH;
        int top = 0;
        while (true) {
            if (sel < top) top = sel;
            if (sel >= top + rows) top = sel - rows + 1;
            d.fillScreen(C_BG);
            header(title);
            footer("ENT:OK", "", "ESC:back");
            for (int i = 0; i < rows; i++) {
                int idx = top + i;
                if (idx >= n) break;
                int y = HDR_H + i * rowH;
                bool isSel = (idx == sel);
                uint16_t bg = isSel ? C_SEL_BG : ((i & 1) ? C_BG : C_PANEL);
                d.fillRect(0, y, LCD_W - 4, rowH, bg);
                if (isSel) d.fillRect(0, y, 3, rowH, C_SEL_BAR);
                d.setTextColor(isSel ? C_FG : C_DIM, bg);
                char num[4]; snprintf(num, sizeof(num), "%d", idx+1);
                d.drawString(num, 6, y + 2);
                d.drawString(items[idx], 22, y + 2);
            }
            if (n > rows) {
                int sbx = LCD_W - 4;
                d.fillRect(sbx, HDR_H, 4, viewH, C_PANEL);
                int thumb = (rows * viewH) / n;
                if (thumb < 8) thumb = 8;
                int ty = HDR_H + (top * (viewH - thumb)) / (n - rows);
                d.fillRect(sbx, ty, 4, thumb, C_ACC);
            }
            while (true) {
                if (!pollKeys()) { delay(15); continue; }
                auto st = M5Cardputer.Keyboard.keysState();
                if (st.fn && IS_ESC) return -1;
                if (st.enter) return sel;
                if (st.fn && IS_UP)   { if (sel > 0)   { sel--; break; } }
                if (st.fn && IS_DOWN) { if (sel < n-1) { sel++; break; } }
                for (auto c : st.word) {
                    if (c >= '1' && c <= '9') {
                        int k = c - '1';
                        if (k < n) return k;
                    }
                }
                if (!st.word.empty()) {
                    bool found = false;
                    for (auto c : st.word) if (c >= '1' && c <= '9') found = true;
                    if (found) break;
                }
            }
        }
    }

    int drawWrapped(const char* text, int x, int y, int w, uint16_t col = C_FG) {
        auto& d = M5Cardputer.Display;
        d.setTextColor(col, C_BG);
        int maxCh = w / 6;
        int len = strlen(text), p = 0, yy = y;
        char line[80];
        while (p < len && yy < LCD_H - FTR_H - 2) {
            int sz = (len - p < maxCh) ? (len - p) : maxCh;
            if (sz < len - p) {
                int sp = sz;
                while (sp > maxCh/2 && line[sp] != ' ' && p+sp < len && text[p+sp] != ' ') sp--;
                if (sp > maxCh/2) sz = sp;
            }
            memcpy(line, text + p, sz); line[sz] = 0;
            d.drawString(line, x, yy);
            yy += 11; p += sz;
            while (p < len && text[p] == ' ') p++;
        }
        return yy;
    }

    bool saveBMP(const char* path) {
        if (!G.sdOK) return false;
        auto& d = M5Cardputer.Display;
        SdGuard guard;
        File f = SD.open(path, FILE_WRITE);
        if (!f) return false;
        const int W = LCD_W, H = LCD_H;
        const int rowSize = ((W * 3 + 3) / 4) * 4;
        uint32_t imgSize = rowSize * H;
        uint32_t fileSize = 54 + imgSize;
        uint8_t hdr[54];
        memset(hdr, 0, sizeof(hdr));
        hdr[0]='B'; hdr[1]='M';
        hdr[2]=fileSize & 0xFF; hdr[3]=(fileSize>>8)&0xFF;
        hdr[4]=(fileSize>>16)&0xFF; hdr[5]=(fileSize>>24)&0xFF;
        hdr[10]=54; hdr[14]=40;
        hdr[18]=W & 0xFF; hdr[19]=(W>>8)&0xFF;
        hdr[22]=H & 0xFF; hdr[23]=(H>>8)&0xFF;
        hdr[26]=1; hdr[28]=24;
        hdr[34]=imgSize & 0xFF; hdr[35]=(imgSize>>8)&0xFF;
        hdr[36]=(imgSize>>16)&0xFF; hdr[37]=(imgSize>>24)&0xFF;
        f.write(hdr, 54);
        uint8_t* row = (uint8_t*)malloc(rowSize);
        if (!row) { f.close(); return false; }
        for (int y = H-1; y >= 0; y--) {
            memset(row, 0, rowSize);
            for (int x = 0; x < W; x++) {
                uint16_t px = d.readPixel(x, y);
                uint8_t r5 = (px >> 11) & 0x1F;
                uint8_t g6 = (px >> 5)  & 0x3F;
                uint8_t b5 =  px        & 0x1F;
                row[x*3 + 0] = (b5 * 255) / 31;
                row[x*3 + 1] = (g6 * 255) / 63;
                row[x*3 + 2] = (r5 * 255) / 31;
            }
            f.write(row, rowSize);
        }
        free(row);
        f.close();
        return true;
    }

    bool saveBMPAuto(const char* prefix = "pomas") {
        char path[48];
        snprintf(path, sizeof(path), "/%s_%lu.bmp", prefix, millis());
        bool ok = saveBMP(path);
        notify(ok ? path : "BMP save failed", ok ? C_OK : C_ERR, 1200);
        return ok;
    }
};
DisplayMgr DM;

// ╔═══════════════════════════════════════════════════════════════════════╗
// ║  Part 3 / 20 ─ ExprEval                                               ║
// ╚═══════════════════════════════════════════════════════════════════════╝

class ExprEval {
public:
    struct Subst { char var; double val; };
    struct Result { double val; bool ok; const char* err; };

    Result eval(const char* expr, const Subst* subs = nullptr, int nSubs = 0) {
        _src = expr; _pos = 0; _err = nullptr; _subs = subs; _nSubs = nSubs;
        skipSpaces();
        double v = parseExpr();
        if (_err) return {0, false, _err};
        skipSpaces();
        if (_src[_pos] != 0) return {0, false, "trailing chars"};
        return {v, true, nullptr};
    }

private:
    const char* _src; int _pos;
    const char* _err;
    const Subst* _subs; int _nSubs;

    void skipSpaces() { while (_src[_pos] == ' ' || _src[_pos] == '\t') _pos++; }
    bool match(const char* s) {
        skipSpaces();
        int n = strlen(s);
        if (strncmp(_src + _pos, s, n) == 0) { _pos += n; return true; }
        return false;
    }
    bool matchCI(const char* s) {
        skipSpaces();
        int n = strlen(s);
        if (strncasecmp(_src + _pos, s, n) == 0) { _pos += n; return true; }
        return false;
    }
    void setErr(const char* msg) { if (!_err) _err = msg; }

    double parseExpr() {
        double v = parseTerm();
        while (true) {
            skipSpaces();
            char c = _src[_pos];
            if (c == '+') { _pos++; v += parseTerm(); }
            else if (c == '-') { _pos++; v -= parseTerm(); }
            else break;
        }
        return v;
    }
    double parseTerm() {
        double v = parseFactor();
        while (true) {
            skipSpaces();
            char c = _src[_pos];
            if (c == '*') { _pos++; v *= parseFactor(); }
            else if (c == '/') { _pos++; double w = parseFactor();
                if (w == 0) { setErr("/0"); return 0; } v /= w; }
            else if (c == '%') { _pos++; double w = parseFactor();
                if (w == 0) { setErr("/0"); return 0; } v = fmod(v,w); }
            else if (matchCI("mod")) { double w = parseFactor();
                if (w == 0) { setErr("/0"); return 0; } v = fmod(v,w); }
            else break;
        }
        return v;
    }
    double parseFactor() {
        double v = parseUnary();
        while (true) {
            skipSpaces();
            if (_src[_pos] == '!') { _pos++; v = factD(v); }
            else break;
        }
        skipSpaces();
        if (_src[_pos] == '^') { _pos++; double e = parseFactor(); v = pow(v, e); }
        return v;
    }
    double parseUnary() {
        skipSpaces();
        char c = _src[_pos];
        if (c == '+') { _pos++; return parseUnary(); }
        if (c == '-') { _pos++; return -parseUnary(); }
        return parsePrimary();
    }
    double parsePrimary() {
        skipSpaces();
        char c = _src[_pos];
        if (c == '(') {
            _pos++;
            double v = parseExpr();
            skipSpaces();
            if (_src[_pos] == ')') _pos++;
            else setErr(") expected");
            return v;
        }
        if (isdigit((unsigned char)c) || c == '.') {
            char* end;
            double v = strtod(_src + _pos, &end);
            _pos = end - _src;
            return v;
        }
        if (c == '0' && (_src[_pos+1] == 'x' || _src[_pos+1] == 'X')) {
            _pos += 2;
            unsigned long long v = 0;
            while (isxdigit((unsigned char)_src[_pos])) {
                char d = _src[_pos++];
                int n = (d <= '9') ? d - '0' : ((d|0x20) - 'a' + 10);
                v = v * 16 + n;
            }
            return (double)v;
        }
        if (isalpha((unsigned char)c) || c == '_') {
            return parseIdent();
        }
        setErr("syntax");
        _pos++;
        return 0;
    }

    double parseIdent() {
        char name[24]; int ni = 0;
        while ((isalnum((unsigned char)_src[_pos]) || _src[_pos] == '_') && ni < 23) {
            name[ni++] = _src[_pos++];
        }
        name[ni] = 0;
        skipSpaces();
        if (_src[_pos] == '(') {
            _pos++;
            double a[4]; int na = 0;
            if (_src[_pos] != ')') {
                a[na++] = parseExpr();
                while (_src[_pos] == ',' && na < 4) {
                    _pos++; a[na++] = parseExpr();
                }
            }
            skipSpaces();
            if (_src[_pos] == ')') _pos++;
            else setErr(") expected");
            return callFn(name, a, na);
        }
        if (!strcmp(name, "pi") || !strcmp(name, "PI") || !strcmp(name, "Pi")) return M_PI;
        if (!strcmp(name, "e") || !strcmp(name, "E"))                          return M_E;
        if (!strcasecmp(name, "ans"))                                          return G.ans;
        if (!strcasecmp(name, "mem"))                                          return G.mem;
        if (!strcasecmp(name, "phi"))                                          return 1.6180339887498948482;
        if (!strcasecmp(name, "gamma"))                                        return 0.5772156649015328606;
        if (!strcasecmp(name, "inf"))                                          return INFINITY;
        if (!strcasecmp(name, "nan"))                                          return NAN;
        if (ni == 1) {
            char v = name[0];
            for (int i = 0; i < _nSubs; i++)
                if (_subs[i].var == v) return _subs[i].val;
            if (v >= 'A' && v <= 'J') return G.vars[v - 'A'];
            if (v >= 'a' && v <= 'j') return G.vars[v - 'a'];
            if (v == 'x' || v == 'X') return 0;
            if (v == 't' || v == 'T') return 0;
            if (v == 'n' || v == 'N') return 0;
        }
        setErr("unknown id");
        return 0;
    }

    double callFn(const char* nm, double* a, int na) {
        if (na == 1) {
            if (!strcasecmp(nm,"sin"))   return sin(toRad(a[0]));
            if (!strcasecmp(nm,"cos"))   return cos(toRad(a[0]));
            if (!strcasecmp(nm,"tan"))   return tan(toRad(a[0]));
            if (!strcasecmp(nm,"asin"))  return fromRad(asin(a[0]));
            if (!strcasecmp(nm,"acos"))  return fromRad(acos(a[0]));
            if (!strcasecmp(nm,"atan"))  return fromRad(atan(a[0]));
            if (!strcasecmp(nm,"sinh"))  return sinh(a[0]);
            if (!strcasecmp(nm,"cosh"))  return cosh(a[0]);
            if (!strcasecmp(nm,"tanh"))  return tanh(a[0]);
            if (!strcasecmp(nm,"asinh")) return asinh(a[0]);
            if (!strcasecmp(nm,"acosh")) return acosh(a[0]);
            if (!strcasecmp(nm,"atanh")) return atanh(a[0]);
            if (!strcasecmp(nm,"csc"))   return 1.0/sin(toRad(a[0]));
            if (!strcasecmp(nm,"sec"))   return 1.0/cos(toRad(a[0]));
            if (!strcasecmp(nm,"cot"))   return 1.0/tan(toRad(a[0]));
            if (!strcasecmp(nm,"csch"))  return 1.0/sinh(a[0]);
            if (!strcasecmp(nm,"sech"))  return 1.0/cosh(a[0]);
            if (!strcasecmp(nm,"coth"))  return 1.0/tanh(a[0]);
            if (!strcasecmp(nm,"exp"))   return exp(a[0]);
            if (!strcasecmp(nm,"ln"))    return log(a[0]);
            if (!strcasecmp(nm,"log"))   return log10(a[0]);
            if (!strcasecmp(nm,"log10")) return log10(a[0]);
            if (!strcasecmp(nm,"log2"))  return log(a[0])/log(2.0);
            if (!strcasecmp(nm,"sqrt"))  return sqrt(a[0]);
            if (!strcasecmp(nm,"cbrt"))  return cbrt(a[0]);
            if (!strcasecmp(nm,"abs"))   return fabs(a[0]);
            if (!strcasecmp(nm,"sign"))  return (a[0]>0)-(a[0]<0);
            if (!strcasecmp(nm,"sgn"))   return (a[0]>0)-(a[0]<0);
            if (!strcasecmp(nm,"floor")) return floor(a[0]);
            if (!strcasecmp(nm,"ceil"))  return ceil(a[0]);
            if (!strcasecmp(nm,"round")) return round(a[0]);
            if (!strcasecmp(nm,"int"))   return trunc(a[0]);
            if (!strcasecmp(nm,"frac"))  return a[0] - trunc(a[0]);
            if (!strcasecmp(nm,"fact"))  return factD(a[0]);
            if (!strcasecmp(nm,"gamma")) return tgamma(a[0]);
            if (!strcasecmp(nm,"lgamma"))return lgamma(a[0]);
            if (!strcasecmp(nm,"erf"))   return erf(a[0]);
            if (!strcasecmp(nm,"erfc"))  return erfc(a[0]);
            if (!strcasecmp(nm,"deg"))   return a[0] * 180.0/M_PI;
            if (!strcasecmp(nm,"rad"))   return a[0] * M_PI/180.0;
            if (!strcasecmp(nm,"isPrime") || !strcasecmp(nm,"prime"))
                                          return isPrime((long long)a[0]) ? 1 : 0;
            if (!strcasecmp(nm,"nextPrime")) return nextPrime((long long)a[0]);
            if (!strcasecmp(nm,"prevPrime")) return prevPrime((long long)a[0]);
            if (!strcasecmp(nm,"random"))    return ((double)esp_random() / 4294967296.0);
            if (!strcasecmp(nm,"randInt") || !strcasecmp(nm,"rint"))
                                          return floor(((double)esp_random()/4294967296.0) * a[0]);
            if (!strcasecmp(nm,"randN")) {
                double u1 = (esp_random()+1.0) / 4294967297.0;
                double u2 = (esp_random()+1.0) / 4294967297.0;
                return sqrt(-2.0*log(u1))*cos(2*M_PI*u2) * a[0];
            }
            if (!strcasecmp(nm,"toF"))   return a[0] * 1.8 + 32.0;
            if (!strcasecmp(nm,"toC"))   return (a[0] - 32.0) / 1.8;
            if (!strcasecmp(nm,"toK"))   return a[0] + 273.15;
        }
        if (na == 2) {
            if (!strcasecmp(nm,"pow"))   return pow(a[0], a[1]);
            if (!strcasecmp(nm,"nrt"))   return pow(a[1], 1.0/a[0]);
            if (!strcasecmp(nm,"atan2")) return fromRad(atan2(a[0], a[1]));
            if (!strcasecmp(nm,"hypot")) return hypot(a[0], a[1]);
            if (!strcasecmp(nm,"min"))   return fmin(a[0], a[1]);
            if (!strcasecmp(nm,"max"))   return fmax(a[0], a[1]);
            if (!strcasecmp(nm,"mod"))   return fmod(a[0], a[1]);
            if (!strcasecmp(nm,"gcd"))   return (double)gcdLL((long long)a[0], (long long)a[1]);
            if (!strcasecmp(nm,"lcm"))   {
                long long x = (long long)a[0], y = (long long)a[1];
                if (!x || !y) return 0;
                return (double)((x/gcdLL(x,y)) * y);
            }
            if (!strcasecmp(nm,"nPr"))   return permD((long long)a[0], (long long)a[1]);
            if (!strcasecmp(nm,"nCr"))   return combD((long long)a[0], (long long)a[1]);
            if (!strcasecmp(nm,"logn"))  return log(a[1]) / log(a[0]);
            if (!strcasecmp(nm,"randR")) {
                double u = (double)esp_random() / 4294967296.0;
                return a[0] + u * (a[1] - a[0]);
            }
        }
        if (na == 3) {
            if (!strcasecmp(nm,"if"))    return a[0] ? a[1] : a[2];
            if (!strcasecmp(nm,"clamp")) return fmin(fmax(a[0], a[1]), a[2]);
            if (!strcasecmp(nm,"dms")) {
                double s = (a[0]<0||a[1]<0||a[2]<0) ? -1 : 1;
                return s * (fabs(a[0]) + fabs(a[1])/60.0 + fabs(a[2])/3600.0);
            }
        }
        setErr("unknown fn");
        return 0;
    }

    static double factD(double x) {
        if (x < 0 || x != floor(x) || x > 170) return tgamma(x + 1.0);
        double r = 1;
        for (int i = 2; i <= (int)x; i++) r *= i;
        return r;
    }
    static long long gcdLL(long long a, long long b) {
        if (a < 0) a = -a; if (b < 0) b = -b;
        while (b) { long long t = a % b; a = b; b = t; }
        return a ? a : 1;
    }
    static double permD(long long n, long long r) {
        if (r < 0 || r > n) return 0;
        double v = 1;
        for (long long i = 0; i < r; i++) v *= (n - i);
        return v;
    }
    static double combD(long long n, long long r) {
        if (r < 0 || r > n) return 0;
        if (r > n - r) r = n - r;
        double v = 1;
        for (long long i = 0; i < r; i++) v = v * (n - i) / (i + 1);
        return v;
    }
    static unsigned long long mulmod64(unsigned long long a, unsigned long long b, unsigned long long m) {
        unsigned long long result = 0;
        a %= m;
        while (b > 0) {
            if (b & 1ULL) result = (result + a) % m;
            a = (a << 1) % m;
            b >>= 1;
        }
        return result;
    }
    static unsigned long long modpow64(unsigned long long b, unsigned long long e, unsigned long long m) {
        unsigned long long r = 1 % m; b %= m;
        while (e) {
            if (e & 1) r = mulmod64(r, b, m);
            b = mulmod64(b, b, m);
            e >>= 1;
        }
        return r;
    }
    static bool isPrime(long long n) {
        if (n < 2) return false;
        for (long long p : {2,3,5,7,11,13,17,19,23,29,31,37}) {
            if (n == p) return true;
            if (n % p == 0) return false;
        }
        unsigned long long d = n - 1; int r = 0;
        while ((d & 1) == 0) { d >>= 1; r++; }
        for (unsigned long long a : {2ULL,3ULL,5ULL,7ULL,11ULL,13ULL,17ULL,19ULL,23ULL,29ULL,31ULL,37ULL}) {
            if (a >= (unsigned long long)n) continue;
            unsigned long long x = modpow64(a, d, n);
            if (x == 1 || x == (unsigned long long)(n-1)) continue;
            bool composite = true;
            for (int i = 0; i < r-1; i++) {
                x = mulmod64(x, x, n);
                if (x == (unsigned long long)(n-1)) { composite = false; break; }
            }
            if (composite) return false;
        }
        return true;
    }
    static long long nextPrime(long long n) {
        if (n < 2) return 2;
        long long p = n + 1;
        while (!isPrime(p)) p++;
        return p;
    }
    static long long prevPrime(long long n) {
        if (n <= 2) return 0;
        long long p = n - 1;
        while (p > 1 && !isPrime(p)) p--;
        return (p > 1) ? p : 0;
    }
};
ExprEval gEv;

// ╔═══════════════════════════════════════════════════════════════════════╗
// ║  Part 4 / 20 ─ NumAnalysis / Mat / Dist                               ║
// ╚═══════════════════════════════════════════════════════════════════════╝

namespace NumAnalysis {

inline double evalAt(const char* expr, char var, double x) {
    ExprEval::Subst s = {var, x};
    auto r = gEv.eval(expr, &s, 1);
    return r.ok ? r.val : NAN;
}

inline double diffN(const char* expr, char var, double x, double h = 0.0, double* errOut = nullptr) {
    const int    NTAB = 12;
    const double CON = 1.4, CON2 = CON*CON, BIG = 1e300, SAFE = 2.0;
    double hh = (h > 0) ? h : 0.01 * (fabs(x) > 1.0 ? fabs(x) : 1.0);
    if (hh == 0) hh = 1e-4;
    static double A[NTAB][NTAB];
    A[0][0] = (evalAt(expr, var, x+hh) - evalAt(expr, var, x-hh)) / (2*hh);
    double err = BIG, ans = A[0][0];
    for (int i = 1; i < NTAB; i++) {
        hh /= CON;
        A[0][i] = (evalAt(expr, var, x+hh) - evalAt(expr, var, x-hh)) / (2*hh);
        double fac = CON2;
        for (int j = 1; j <= i; j++) {
            A[j][i] = (A[j-1][i]*fac - A[j-1][i-1]) / (fac - 1.0);
            fac = CON2*fac;
            double errt = fmax(fabs(A[j][i]-A[j-1][i]), fabs(A[j][i]-A[j-1][i-1]));
            if (errt <= err) { err = errt; ans = A[j][i]; }
        }
        if (fabs(A[i][i] - A[i-1][i-1]) >= SAFE*err) break;
    }
    if (errOut) *errOut = (err >= BIG) ? NAN : err;
    return ans;
}

inline double adsimp(const char* expr, char var,
                     double a, double b, double fa, double fb, double fm,
                     double S, double eps, int depth, double* errAcc, long* evals) {
    double m  = 0.5*(a + b);
    double lm = 0.5*(a + m), rm = 0.5*(m + b);
    double flm = evalAt(expr, var, lm), frm = evalAt(expr, var, rm);
    if (evals) *evals += 2;
    double Sl = (m - a)/6.0 * (fa + 4*flm + fm);
    double Sr = (b - m)/6.0 * (fm + 4*frm + fb);
    double S2 = Sl + Sr;
    bool budget = (evals && *evals > 200000L);
    if (depth <= 0 || budget || fabs(S2 - S) <= 15.0*eps) {
        if (errAcc) *errAcc += fabs(S2 - S) / 15.0;
        return S2 + (S2 - S)/15.0;
    }
    return adsimp(expr, var, a, m, fa, fm, flm, Sl, eps*0.5, depth-1, errAcc, evals)
         + adsimp(expr, var, m, b, fm, fb, frm, Sr, eps*0.5, depth-1, errAcc, evals);
}

inline double integN(const char* expr, char var, double a, double b, int n = 0, double* errOut = nullptr) {
    (void)n;
    if (a == b) { if (errOut) *errOut = 0; return 0.0; }
    double sign = 1.0;
    if (a > b) { double t=a; a=b; b=t; sign = -1.0; }
    double fa = evalAt(expr, var, a), fb = evalAt(expr, var, b);
    double m  = 0.5*(a+b),  fm = evalAt(expr, var, m);
    double S  = (b - a)/6.0 * (fa + 4*fm + fb);
    double eps = 1e-11 * (fabs(S) + 1.0);
    double errAcc = 0.0; long evals = 3;
    double r = adsimp(expr, var, a, b, fa, fb, fm, S, eps, 22, &errAcc, &evals);
    if (errOut) *errOut = errAcc;
    return sign * r;
}

inline double sumN(const char* expr, char var, double from, double to) {
    KSum k;
    long long lo = (long long)from, hi = (long long)to;
    for (long long i = lo; i <= hi; i++) k.add(evalAt(expr, var, (double)i));
    return k.get();
}
inline double prodN(const char* expr, char var, double from, double to) {
    double p = 1;
    long long lo = (long long)from, hi = (long long)to;
    for (long long i = lo; i <= hi; i++) p *= evalAt(expr, var, (double)i);
    return p;
}

inline bool solveNR(const char* expr, char var, double x0, double& root,
                    double tol = 1e-9, int maxIter = 200) {
    double x = x0;
    for (int i = 0; i < maxIter; i++) {
        double fx = evalAt(expr, var, x);
        if (!isfinite(fx)) return false;
        if (fabs(fx) < tol) { root = x; return true; }
        double d = diffN(expr, var, x);
        if (fabs(d) < 1e-15) return false;
        double nx = x - fx / d;
        if (fabs(nx - x) < tol) { root = nx; return true; }
        x = nx;
    }
    root = x;
    return false;
}

inline bool bisect(const char* expr, char var, double a, double b,
                   double& root, double tol = 1e-10, int maxIter = 200) {
    double fa = evalAt(expr, var, a);
    double fb = evalAt(expr, var, b);
    if (!isfinite(fa) || !isfinite(fb)) return false;
    if (fa * fb > 0) return false;
    for (int i = 0; i < maxIter; i++) {
        double m = (a + b) / 2;
        double fm = evalAt(expr, var, m);
        if (!isfinite(fm)) return false;
        if (fabs(fm) < tol || (b - a) / 2 < tol) { root = m; return true; }
        if (fa * fm < 0) { b = m; fb = fm; }
        else             { a = m; fa = fm; }
    }
    root = (a + b) / 2;
    return true;
}

inline int quadratic(double a, double b, double c, double r[2], double i[2]) {
    if (fabs(a) < 1e-15) {
        if (fabs(b) < 1e-15) return 0;
        r[0] = -c / b; i[0] = 0;
        return 1;
    }
    double D = b*b - 4*a*c;
    if (D >= 0) {
        double s = sqrt(D);
        r[0] = (-b + s) / (2*a); i[0] = 0;
        r[1] = (-b - s) / (2*a); i[1] = 0;
        return 2;
    }
    double s = sqrt(-D);
    r[0] = -b / (2*a); i[0] =  s / (2*a);
    r[1] = -b / (2*a); i[1] = -s / (2*a);
    return 2;
}

inline int cubic(double a, double b, double c, double d, double r[3], double im[3]) {
    if (fabs(a) < 1e-15) {
        double rr[2], ii[2];
        int k = quadratic(b, c, d, rr, ii);
        for (int i = 0; i < k; i++) { r[i] = rr[i]; im[i] = ii[i]; }
        return k;
    }
    double A = b/a, B = c/a, C = d/a;
    double Q = (3*B - A*A) / 9.0;
    double R = (9*A*B - 27*C - 2*A*A*A) / 54.0;
    double D = Q*Q*Q + R*R;
    if (D >= 0) {
        double sD = sqrt(D);
        double S = cbrt(R + sD);
        double T = cbrt(R - sD);
        r[0]  = -A/3 + (S + T);            im[0] = 0;
        r[1]  = -A/3 - (S + T)/2;          im[1] =  sqrt(3.0)/2 * (S - T);
        r[2]  = r[1];                      im[2] = -im[1];
        return 3;
    }
    double th = acos(R / sqrt(-Q*Q*Q));
    double mq = 2 * sqrt(-Q);
    r[0] = mq * cos( th       /3) - A/3; im[0] = 0;
    r[1] = mq * cos((th+2*M_PI)/3) - A/3; im[1] = 0;
    r[2] = mq * cos((th+4*M_PI)/3) - A/3; im[2] = 0;
    return 3;
}

inline int quartic(double a, double b, double c, double d, double e,
                   double r[4], double im[4]) {
    if (fabs(a) < 1e-15) return cubic(b, c, d, e, r, im);
    b /= a; c /= a; d /= a; e /= a;
    double p = c - 3*b*b/8;
    double q = d - b*c/2 + b*b*b/8;
    double rr2 = e - b*d/4 + b*b*c/16 - 3*b*b*b*b/256;
    double rc[3], ic[3];
    int k = cubic(1, 2*p, p*p - 4*rr2, -q*q, rc, ic);
    double y = NAN;
    for (int i = 0; i < k; i++) if (fabs(ic[i]) < 1e-9 && rc[i] > 0) { y = rc[i]; break; }
    if (!isfinite(y)) {
        for (int i = 0; i < k; i++) if (fabs(ic[i]) < 1e-6) { y = rc[i]; break; }
    }
    if (!isfinite(y)) return 0;
    double sq = sqrt(y);
    double A1 =  sq, B1 = -(2*p + y - q/sq) / 2;
    double A2 = -sq, B2 = -(2*p + y + q/sq) / 2;
    double r1[2], i1[2], r2[2], i2[2];
    quadratic(1, A1, B1, r1, i1);
    quadratic(1, A2, B2, r2, i2);
    r[0]  = r1[0] - b/4; im[0] = i1[0];
    r[1]  = r1[1] - b/4; im[1] = i1[1];
    r[2]  = r2[0] - b/4; im[2] = i2[0];
    r[3]  = r2[1] - b/4; im[3] = i2[1];
    return 4;
}

inline int polyRoots(const double* coef, int deg, Complex* roots) {
    if (deg <= 0) return 0;
    if (deg == 1) { roots[0] = Complex(-coef[0]/coef[1], 0); return 1; }
    Complex z[16];
    for (int i = 0; i < deg; i++) {
        double th = 2*M_PI*i/deg + 0.4;
        z[i] = Complex(0.4*cos(th), 0.9*sin(th));
    }
    auto evalC = [&](Complex x){
        Complex r;
        for (int i = deg; i >= 0; i--) r = r*x + Complex(coef[i],0);
        return r;
    };
    for (int it = 0; it < 200; it++) {
        double maxd = 0;
        for (int i = 0; i < deg; i++) {
            Complex denom(1,0);
            for (int j = 0; j < deg; j++) if (j != i) denom = denom * (z[i] - z[j]);
            Complex fz = evalC(z[i]);
            if (denom.abs() < 1e-300) continue;
            Complex dz = fz / denom;
            z[i] = z[i] - dz;
            if (dz.abs() > maxd) maxd = dz.abs();
        }
        if (maxd < 1e-12) break;
    }
    for (int i = 0; i < deg; i++) roots[i] = z[i];
    return deg;
}

} // namespace NumAnalysis

namespace Mat {

inline void mul(const double* a, int r, int c, const double* b, int k, double* out) {
    for (int i = 0; i < r; i++)
        for (int j = 0; j < k; j++) {
            double s = 0;
            for (int l = 0; l < c; l++) s += a[i*c+l] * b[l*k+j];
            out[i*k+j] = s;
        }
}
inline void transpose(const double* a, int r, int c, double* out) {
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++) out[j*r+i] = a[i*c+j];
}
inline bool gaussSolve(double* A, int n, double* b) {
    for (int i = 0; i < n; i++) {
        int pi = i; double pv = fabs(A[i*n+i]);
        for (int k = i+1; k < n; k++) if (fabs(A[k*n+i]) > pv) { pv = fabs(A[k*n+i]); pi = k; }
        if (pv < 1e-15) return false;
        if (pi != i) {
            for (int j = 0; j < n; j++) { double t = A[i*n+j]; A[i*n+j] = A[pi*n+j]; A[pi*n+j] = t; }
            double t = b[i]; b[i] = b[pi]; b[pi] = t;
        }
        double piv = A[i*n+i];
        for (int j = i; j < n; j++) A[i*n+j] /= piv;
        b[i] /= piv;
        for (int k = 0; k < n; k++) if (k != i) {
            double f = A[k*n+i];
            for (int j = i; j < n; j++) A[k*n+j] -= f * A[i*n+j];
            b[k] -= f * b[i];
        }
    }
    return true;
}
inline double det(const double* in, int n) {
    double* A = (double*)malloc(n*n*sizeof(double));
    if (!A) return 0;
    memcpy(A, in, n*n*sizeof(double));
    double s = 1;
    for (int i = 0; i < n; i++) {
        int pi = i; double pv = fabs(A[i*n+i]);
        for (int k = i+1; k < n; k++) if (fabs(A[k*n+i]) > pv) { pv = fabs(A[k*n+i]); pi = k; }
        if (pv < 1e-15) { free(A); return 0; }
        if (pi != i) {
            for (int j = 0; j < n; j++) { double t = A[i*n+j]; A[i*n+j] = A[pi*n+j]; A[pi*n+j] = t; }
            s = -s;
        }
        for (int k = i+1; k < n; k++) {
            double f = A[k*n+i] / A[i*n+i];
            for (int j = i; j < n; j++) A[k*n+j] -= f * A[i*n+j];
        }
    }
    double d = s;
    for (int i = 0; i < n; i++) d *= A[i*n+i];
    free(A);
    return d;
}
inline bool inv(const double* in, int n, double* out) {
    double* A = (double*)malloc(n*n*sizeof(double));
    if (!A) return false;
    memcpy(A, in, n*n*sizeof(double));
    for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) out[i*n+j] = (i==j)?1.0:0.0;
    for (int i = 0; i < n; i++) {
        int pi = i; double pv = fabs(A[i*n+i]);
        for (int k = i+1; k < n; k++) if (fabs(A[k*n+i]) > pv) { pv = fabs(A[k*n+i]); pi = k; }
        if (pv < 1e-15) { free(A); return false; }
        if (pi != i) {
            for (int j = 0; j < n; j++) {
                double t1 = A[i*n+j];   A[i*n+j]   = A[pi*n+j];   A[pi*n+j]   = t1;
                double t2 = out[i*n+j]; out[i*n+j] = out[pi*n+j]; out[pi*n+j] = t2;
            }
        }
        double p = A[i*n+i];
        for (int j = 0; j < n; j++) { A[i*n+j] /= p; out[i*n+j] /= p; }
        for (int k = 0; k < n; k++) if (k != i) {
            double f = A[k*n+i];
            for (int j = 0; j < n; j++) {
                A[k*n+j]   -= f * A[i*n+j];
                out[k*n+j] -= f * out[i*n+j];
            }
        }
    }
    free(A);
    return true;
}
inline int rank(const double* in, int r, int c) {
    double* A = (double*)malloc(r*c*sizeof(double));
    if (!A) return 0;
    memcpy(A, in, r*c*sizeof(double));
    int rk = 0, col = 0;
    for (int row = 0; row < r && col < c; col++) {
        int pi = row; double pv = fabs(A[row*c + col]);
        for (int k = row+1; k < r; k++) if (fabs(A[k*c+col]) > pv) { pv = fabs(A[k*c+col]); pi = k; }
        if (pv < 1e-12) continue;
        if (pi != row) for (int j = 0; j < c; j++) { double t = A[row*c+j]; A[row*c+j] = A[pi*c+j]; A[pi*c+j] = t; }
        for (int k = row+1; k < r; k++) {
            double f = A[k*c + col] / A[row*c + col];
            for (int j = col; j < c; j++) A[k*c+j] -= f * A[row*c+j];
        }
        row++; rk++;
    }
    free(A);
    return rk;
}
inline bool jacobiEig(double* A, int n, double* eig, double* V = nullptr) {
    if (V) for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) V[i*n+j] = (i==j)?1.0:0.0;
    for (int it = 0; it < 100; it++) {
        int p = 0, q = 1;
        double maxv = 0;
        for (int i = 0; i < n; i++) for (int j = i+1; j < n; j++)
            if (fabs(A[i*n+j]) > maxv) { maxv = fabs(A[i*n+j]); p = i; q = j; }
        if (maxv < 1e-12) break;
        double app = A[p*n+p], aqq = A[q*n+q], apq = A[p*n+q];
        double th = 0.5 * atan2(2*apq, app - aqq);
        double c = cos(th), s = sin(th);
        for (int i = 0; i < n; i++) {
            double aip = A[i*n+p], aiq = A[i*n+q];
            A[i*n+p] = c*aip + s*aiq;
            A[i*n+q] = -s*aip + c*aiq;
        }
        for (int j = 0; j < n; j++) {
            double apj = A[p*n+j], aqj = A[q*n+j];
            A[p*n+j] = c*apj + s*aqj;
            A[q*n+j] = -s*apj + c*aqj;
        }
        if (V) for (int i = 0; i < n; i++) {
            double vip = V[i*n+p], viq = V[i*n+q];
            V[i*n+p] = c*vip + s*viq;
            V[i*n+q] = -s*vip + c*viq;
        }
    }
    for (int i = 0; i < n; i++) eig[i] = A[i*n+i];
    for (int i = 0; i < n; i++)
        for (int j = i+1; j < n; j++)
            if (eig[i] < eig[j]) {
                double t = eig[i]; eig[i] = eig[j]; eig[j] = t;
                if (V) for (int k = 0; k < n; k++) {
                    double tt = V[k*n+i]; V[k*n+i] = V[k*n+j]; V[k*n+j] = tt;
                }
            }
    return true;
}
inline bool qrEig(const double* in, int n, double* eigRe, double* eigIm) {
    double* A = (double*)malloc(n*n*sizeof(double));
    if (!A) return false;
    memcpy(A, in, n*n*sizeof(double));
    for (int it = 0; it < 200; it++) {
        double* Q = (double*)malloc(n*n*sizeof(double));
        if (!Q) { free(A); return false; }
        for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) Q[i*n+j] = (i==j)?1.0:0.0;
        for (int i = 0; i < n-1; i++)
            for (int k = i+1; k < n; k++) {
                double a = A[i*n+i], b = A[k*n+i];
                double r = hypot(a, b);
                if (r < 1e-15) continue;
                double c = a/r, s = b/r;
                for (int j = 0; j < n; j++) {
                    double t1 = c*A[i*n+j] + s*A[k*n+j];
                    double t2 = -s*A[i*n+j] + c*A[k*n+j];
                    A[i*n+j] = t1; A[k*n+j] = t2;
                    double q1 = c*Q[j*n+i] + s*Q[j*n+k];
                    double q2 = -s*Q[j*n+i] + c*Q[j*n+k];
                    Q[j*n+i] = q1; Q[j*n+k] = q2;
                }
            }
        double* T = (double*)malloc(n*n*sizeof(double));
        if (!T) { free(Q); free(A); return false; }
        Mat::mul(A, n, n, Q, n, T);
        memcpy(A, T, n*n*sizeof(double));
        free(T); free(Q);
    }
    int i = 0;
    while (i < n) {
        if (i < n-1 && fabs(A[(i+1)*n+i]) > 1e-9) {
            double a = A[i*n+i], b = A[i*n+i+1];
            double c = A[(i+1)*n+i], d = A[(i+1)*n+i+1];
            double tr = a + d, dt = a*d - b*c;
            double D = tr*tr - 4*dt;
            if (D >= 0) {
                eigRe[i]   = (tr + sqrt(D))/2; eigIm[i]   = 0;
                eigRe[i+1] = (tr - sqrt(D))/2; eigIm[i+1] = 0;
            } else {
                eigRe[i]   = tr/2; eigIm[i]   =  sqrt(-D)/2;
                eigRe[i+1] = tr/2; eigIm[i+1] = -sqrt(-D)/2;
            }
            i += 2;
        } else {
            eigRe[i] = A[i*n+i]; eigIm[i] = 0; i++;
        }
    }
    free(A);
    return true;
}
inline bool luSolve(double* A, int n, double* b) { return gaussSolve(A, n, b); }

} // namespace Mat

namespace Dist {

inline double normalCDF(double x) {
    return 0.5 * (1.0 + erf(x / sqrt(2.0)));
}
inline double normalPDF(double x, double mu = 0, double sd = 1) {
    double z = (x - mu) / sd;
    return exp(-0.5*z*z) / (sd * sqrt(2*M_PI));
}
inline double normalInv(double p) {
    if (p <= 0 || p >= 1) return p <= 0 ? -INFINITY : INFINITY;
    static const double a[] = {-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02,
                                1.383577518672690e+02,-3.066479806614716e+01, 2.506628277459239e+00};
    static const double b[] = {-5.447609879822406e+01, 1.615858368580409e+02,-1.556989798598866e+02,
                                6.680131188771972e+01,-1.328068155288572e+01};
    static const double c[] = {-7.784894002430293e-03,-3.223964580411365e-01,-2.400758277161838e+00,
                               -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00};
    static const double dd[]= { 7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,
                                3.754408661907416e+00};
    double pl = 0.02425, ph = 1 - pl;
    if (p < pl) {
        double q = sqrt(-2*log(p));
        return (((((c[0]*q+c[1])*q+c[2])*q+c[3])*q+c[4])*q+c[5]) /
               ((((dd[0]*q+dd[1])*q+dd[2])*q+dd[3])*q+1);
    }
    if (p <= ph) {
        double q = p - 0.5, r = q*q;
        return (((((a[0]*r+a[1])*r+a[2])*r+a[3])*r+a[4])*r+a[5])*q /
               (((((b[0]*r+b[1])*r+b[2])*r+b[3])*r+b[4])*r+1);
    }
    double q = sqrt(-2*log(1-p));
    return -(((((c[0]*q+c[1])*q+c[2])*q+c[3])*q+c[4])*q+c[5]) /
            ((((dd[0]*q+dd[1])*q+dd[2])*q+dd[3])*q+1);
}

inline double gammaP(double a, double x) {
    if (x <= 0 || a <= 0) return 0;
    if (x < a + 1) {
        double s = 1.0/a, t = s;
        for (int i = 1; i < 200; i++) {
            t *= x / (a + i);
            s += t;
            if (fabs(t) < fabs(s)*1e-12) break;
        }
        return s * exp(-x + a*log(x) - lgamma(a));
    }
    double b = x + 1 - a, c = 1e30, d = 1.0/b, h = d;
    for (int i = 1; i < 200; i++) {
        double an = -i*(i - a);
        b += 2;
        d = an * d + b;       if (fabs(d) < 1e-300) d = 1e-300;
        c = b + an / c;       if (fabs(c) < 1e-300) c = 1e-300;
        d = 1.0 / d;
        double del = d * c;
        h *= del;
        if (fabs(del - 1) < 1e-12) break;
    }
    return 1.0 - h * exp(-x + a*log(x) - lgamma(a));
}
inline double chiSquareCDF(double x, double df) {
    if (x <= 0) return 0;
    return gammaP(df/2.0, x/2.0);
}
inline double chiSquarePDF(double x, double df) {
    if (x <= 0) return 0;
    return pow(x, df/2-1) * exp(-x/2) / (pow(2, df/2) * tgamma(df/2));
}
inline double chiSquareInv(double p, double df) {
    if (p <= 0) return 0;
    if (p >= 1) return INFINITY;
    double lo = 0, hi = df * 10 + 100;
    while (chiSquareCDF(hi, df) < p) hi *= 2;
    for (int i = 0; i < 100; i++) {
        double m = (lo + hi)/2;
        if (chiSquareCDF(m, df) < p) lo = m; else hi = m;
        if (hi - lo < 1e-9) break;
    }
    return (lo + hi)/2;
}

inline double betaCF(double a, double b, double x) {
    double qab = a+b, qap = a+1, qam = a-1;
    double c = 1, d = 1 - qab*x/qap;
    if (fabs(d) < 1e-300) d = 1e-300;
    d = 1/d; double h = d;
    for (int m = 1; m <= 200; m++) {
        int m2 = 2*m;
        double aa = m*(b-m)*x/((qam+m2)*(a+m2));
        d = 1 + aa*d; if (fabs(d) < 1e-300) d = 1e-300;
        c = 1 + aa/c; if (fabs(c) < 1e-300) c = 1e-300;
        d = 1/d; h *= d*c;
        aa = -(a+m)*(qab+m)*x/((a+m2)*(qap+m2));
        d = 1 + aa*d; if (fabs(d) < 1e-300) d = 1e-300;
        c = 1 + aa/c; if (fabs(c) < 1e-300) c = 1e-300;
        d = 1/d; double del = d*c; h *= del;
        if (fabs(del - 1) < 1e-12) break;
    }
    return h;
}
inline double betaInc(double a, double b, double x) {
    if (x <= 0) return 0;
    if (x >= 1) return 1;
    double bt = exp(lgamma(a+b)-lgamma(a)-lgamma(b)+a*log(x)+b*log(1-x));
    if (x < (a+1)/(a+b+2)) return bt * betaCF(a, b, x) / a;
    return 1 - bt * betaCF(b, a, 1-x) / b;
}
inline double tCDF(double t, double df) {
    double x = df / (df + t*t);
    double p = 0.5 * betaInc(df/2.0, 0.5, x);
    return (t > 0) ? 1 - p : p;
}
inline double tPDF(double t, double df) {
    return exp(lgamma((df+1)/2) - lgamma(df/2)) / sqrt(df*M_PI) *
           pow(1 + t*t/df, -(df+1)/2);
}
inline double tInv(double p, double df) {
    if (p <= 0) return -INFINITY;
    if (p >= 1) return  INFINITY;
    double lo = -100, hi = 100;
    for (int i = 0; i < 100; i++) {
        double m = (lo + hi)/2;
        if (tCDF(m, df) < p) lo = m; else hi = m;
        if (hi - lo < 1e-9) break;
    }
    return (lo + hi)/2;
}
inline double FCDF(double x, double d1, double d2) {
    if (x <= 0) return 0;
    return 1 - betaInc(d2/2, d1/2, d2/(d2 + d1*x));
}
inline double FPDF(double x, double d1, double d2) {
    if (x <= 0) return 0;
    return exp(lgamma((d1+d2)/2) - lgamma(d1/2) - lgamma(d2/2) +
               (d1/2)*log((double)d1/d2) + (d1/2-1)*log(x) -
               ((d1+d2)/2)*log(1 + (double)d1*x/d2));
}
inline double FInv(double p, double d1, double d2) {
    if (p <= 0) return 0;
    if (p >= 1) return INFINITY;
    double lo = 0, hi = 100;
    while (FCDF(hi, d1, d2) < p) hi *= 2;
    for (int i = 0; i < 100; i++) {
        double m = (lo + hi)/2;
        if (FCDF(m, d1, d2) < p) lo = m; else hi = m;
        if (hi - lo < 1e-9) break;
    }
    return (lo + hi)/2;
}
inline double binomPMF(int k, int n, double p) {
    if (k < 0 || k > n) return 0;
    return exp(lgamma(n+1) - lgamma(k+1) - lgamma(n-k+1) + k*log(p) + (n-k)*log(1-p));
}
inline double binomCDF(int k, int n, double p) {
    double s = 0;
    for (int i = 0; i <= k; i++) s += binomPMF(i, n, p);
    return s;
}
inline int binomInv(double q, int n, double p) {
    double s = 0;
    for (int k = 0; k <= n; k++) {
        s += binomPMF(k, n, p);
        if (s >= q) return k;
    }
    return n;
}
inline double poisPMF(int k, double lam) {
    if (k < 0) return 0;
    return exp(-lam + k*log(lam) - lgamma(k+1));
}
inline double poisCDF(int k, double lam) {
    double s = 0;
    for (int i = 0; i <= k; i++) s += poisPMF(i, lam);
    return s;
}
inline int poisInv(double q, double lam) {
    double s = 0;
    for (int k = 0; k < 1000; k++) {
        s += poisPMF(k, lam);
        if (s >= q) return k;
    }
    return 1000;
}
inline double geomPMF(int k, double p) {
    if (k < 1) return 0;
    return p * pow(1 - p, k - 1);
}
inline double geomCDF(int k, double p) {
    if (k < 1) return 0;
    return 1 - pow(1 - p, k);
}
inline int geomInv(double q, double p) {
    if (q <= 0) return 1;
    if (q >= 1) return 1000;
    return (int)ceil(log(1 - q) / log(1 - p));
}

} // namespace Dist