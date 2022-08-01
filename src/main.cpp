#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <pcf8563.h>
#include "esp_adc_cal.h"

#define TP_PIN_PIN          33
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define IMU_INT_PIN         38
#define RTC_INT_PIN         34
#define BATT_ADC_PIN        35
#define VBUS_PIN            36
#define TP_PWR_PIN          25
#define LED_PIN             4
#define CHARGE_PIN          32

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
PCF8563_Class rtc;

char buff[256];
bool rtcIrq = false;
bool initial = 1;
bool otaStart = false;
uint8_t func_select = 0;
uint8_t omm = 99;
uint8_t xcolon = 0;
uint32_t targetTime = 0;       // for next 1 second timeout
uint32_t colour = 0;
int vref = 1100;
bool pressed = false;
uint32_t pressedTime = 0;
bool charge_indication = false;
uint8_t hh, mm, ss ;

//----------------------------------------------------------------------------------------------------------------

String getVoltage()
{
    uint16_t v = analogRead(BATT_ADC_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    return String(battery_voltage) + "V";
}
    
void Voltage_Show(){
    tft.drawString(getVoltage(),0 , 20);
}

void setupADC()
{
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    } else {
        Serial.println("Default Vref: 1100mV");
    }
}

void setupRTC()
{
    rtc.begin(Wire);
    //Check if the RTC clock matches, if not, use compile time
    rtc.check();
    RTC_Date datetime = rtc.getDateTime();
    hh = datetime.hour;
    mm = datetime.minute;
    ss = datetime.second;
}

void RTC_Show()
{
    if (targetTime < millis()) {
        RTC_Date datetime = rtc.getDateTime();
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setCursor (0, 0);
        tft.print(__TIME__); // This uses the standard ADAFruit small font
        tft.setCursor (0, 10);
        tft.print(__DATE__);
    }
}

//----------------------------------------------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    tft.init();
    tft.setRotation(0);
    tft.setSwapBytes(true);
    tft.fillScreen(TFT_BLACK);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);
    
    pinMode(TP_PIN_PIN, INPUT);
    pinMode(TP_PWR_PIN, PULLUP);//! Must be set to pull-up output mode in order to wake up in deep sleep mode
    digitalWrite(TP_PWR_PIN, HIGH);
    pinMode(LED_PIN, OUTPUT);
    pinMode(CHARGE_PIN, INPUT_PULLUP);
    
    attachInterrupt(CHARGE_PIN, [] {
        charge_indication = true;
    }, CHANGE);
    if (digitalRead(CHARGE_PIN) == LOW) {
        charge_indication = true;
    }
    
    setupRTC();
}

void loop() {
       if (digitalRead(TP_PIN_PIN) == HIGH) {
        if (!pressed) {
            initial = 1;
            targetTime = millis() + 1000;
            tft.fillScreen(TFT_BLACK);
            omm = 99;
            func_select = func_select + 1 > 2 ? 0 : func_select + 1;
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            pressed = true;
            pressedTime = millis();
        } else {
            if (millis() - pressedTime > 3000) {
                tft.fillScreen(TFT_BLACK);
                tft.drawString("pressed 3 segs",0, 0);
                delay(3000);
                esp_restart();
            }
        }
    } else {
        pressed = false;
    }

    switch (func_select) {
    case 0:
        RTC_Show();
        Voltage_Show();
        delay(3000);
        tft.writecommand(ST7735_SLPIN);
        tft.writecommand(ST7735_DISPOFF);
        esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH);
        esp_deep_sleep_start();
        break;
    default:
        break;
    }
}