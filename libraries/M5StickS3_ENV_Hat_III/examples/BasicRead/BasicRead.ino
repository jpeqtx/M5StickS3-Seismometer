/**
 * BasicRead.ino
 * M5StickS3_ENV_Hat_III の基本的な使用例
 *
 * ENV III HATから気温・湿度・気圧を読み取り、シリアルと画面に表示します。
 * 自動熱補正が有効なため、起動後10〜15分で値が安定します。
 */

#include <M5Unified.h>
#include <M5StickS3_ENV_Hat_III.h>

M5StickS3_ENV_Hat_III env;

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);

    M5.Lcd.setRotation(0);
    M5.Lcd.setBrightness(20);   // 発熱抑制
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setFont(&fonts::Font2);
    M5.Lcd.setCursor(4, 4);
    M5.Lcd.printf("ENV Hat III\nInit...");

    Serial.begin(115200);
    delay(300);

    if (!env.begin()) {
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setTextColor(TFT_RED);
        M5.Lcd.setCursor(4, 4);
        M5.Lcd.printf("Sensor init failed");
        while (true) delay(1000);
    }

    // 自動補正を有効化（デフォルトで有効）
    env.enableAutoComp(true);

    // 必要に応じて実気温を入力してキャリブレーション
    // 例：正確な温度計で25.3℃を確認した場合
    // env.setActualTemp(25.3f);
}

void loop() {
    M5.update();

    env.update();

    float temp = env.getTemperatureSHT();
    float hum  = env.getHumidity();
    float press = env.getPressure();

    // シリアル出力
    Serial.printf("T=%.2f C  H=%.1f %%  P=%.1f hPa  Eq=%s\n",
                  temp, hum, press,
                  env.isThermalEquilibrium() ? "YES" : "NO");

    // 画面表示
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(4, 4);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.printf("ENV Hat III\n\n");

    M5.Lcd.setTextColor(TFT_CYAN);
    M5.Lcd.printf("Temp : %.2f C\n", temp);

    M5.Lcd.setTextColor(TFT_YELLOW);
    M5.Lcd.printf("Hum  : %.1f %%\n", hum);

    M5.Lcd.setTextColor(TFT_GREEN);
    M5.Lcd.printf("Press: %.1f hPa\n\n", press);

    M5.Lcd.setTextColor(env.isThermalEquilibrium() ? TFT_GREEN : TFT_ORANGE);
    M5.Lcd.printf("%s", env.isThermalEquilibrium() ?
                  "Equilibrium: YES" : "Warming up...");

    delay(1000);
}