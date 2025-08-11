#pragma once

#include <Arduino.h>
#include <helpers/RefCountedDigitalPin.h>

// LoRa radio module pins for Heltec V3
// Also for Heltec Wireless Tracker/Paper
#define  P_LORA_DIO_1   14
#define  P_LORA_NSS      8
#define  P_LORA_RESET   RADIOLIB_NC
#define  P_LORA_BUSY    13
#define  P_LORA_SCLK     9
#define  P_LORA_MISO    11
#define  P_LORA_MOSI    10

// built-ins
#ifndef PIN_VBAT_READ              // set in platformio.ini for boards like Heltec Wireless Paper (20)
  #define  PIN_VBAT_READ    1
#endif
#ifndef PIN_ADC_CTRL              // set in platformio.ini for Heltec Wireless Tracker (2)
  #define  PIN_ADC_CTRL    37
#endif
#define  PIN_ADC_CTRL_ACTIVE    LOW
#define  PIN_ADC_CTRL_INACTIVE  HIGH
//#define  PIN_LED_BUILTIN 35

#include "ESP32Board.h"

#include <driver/rtc_io.h>

class HeltecV3Board : public ESP32Board {
private:
  bool adc_active_state;

public:
  RefCountedDigitalPin periph_power;

  HeltecV3Board() : periph_power(PIN_VEXT_EN) { }

  void begin() {
    ESP32Board::begin();

    // Auto-detect correct ADC_CTRL pin polarity (different for boards >3.2)
    pinMode(PIN_ADC_CTRL, INPUT);
    adc_active_state = !digitalRead(PIN_ADC_CTRL);
    
    pinMode(PIN_ADC_CTRL, OUTPUT);
    digitalWrite(PIN_ADC_CTRL, !adc_active_state); // Initially inactive

    periph_power.begin();

    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_DEEPSLEEP) {
      // Check if this was a power-off wake-up that needs verification
      checkWakeupReason();
      
      long wakeup_source = esp_sleep_get_ext1_wakeup_status();
      if (wakeup_source & (1 << P_LORA_DIO_1)) {  // received a LoRa packet (while in deep sleep)
        startup_reason = BD_STARTUP_RX_PACKET;
      }

      rtc_gpio_hold_dis((gpio_num_t)P_LORA_NSS);
      rtc_gpio_deinit((gpio_num_t)P_LORA_DIO_1);
      
      // Clean up GPIO 0 after wake-up check
      rtc_gpio_hold_dis(GPIO_NUM_0);
      rtc_gpio_deinit(GPIO_NUM_0);
    }
  }

  void enterDeepSleep(uint32_t secs, int pin_wake_btn = -1) {
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    // Make sure the DIO1 and NSS GPIOs are hold on required levels during deep sleep 
    rtc_gpio_set_direction((gpio_num_t)P_LORA_DIO_1, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_en((gpio_num_t)P_LORA_DIO_1);

    rtc_gpio_hold_en((gpio_num_t)P_LORA_NSS);

    if (pin_wake_btn < 0) {
      esp_sleep_enable_ext1_wakeup( (1L << P_LORA_DIO_1), ESP_EXT1_WAKEUP_ANY_HIGH);  // wake up on: recv LoRa packet
    } else {
      esp_sleep_enable_ext1_wakeup( (1L << P_LORA_DIO_1) | (1L << pin_wake_btn), ESP_EXT1_WAKEUP_ANY_HIGH);  // wake up on: recv LoRa packet OR wake btn
    }

    if (secs > 0) {
      esp_sleep_enable_timer_wakeup(secs * 1000000);
    }

    // Finally set ESP32 into sleep
    esp_deep_sleep_start();   // CPU halts here and never returns!
  }

  void enterPowerOffSleep(uint32_t timeout_seconds = 3600, bool enable_button_wake = true) {
    // Power-off sleep with optional button wake-up
    if (enable_button_wake) {
      // Configure GPIO 0 for RTC wake-up with proper pull-up to avoid boot conflicts
      rtc_gpio_init(GPIO_NUM_0);
      rtc_gpio_set_direction(GPIO_NUM_0, RTC_GPIO_MODE_INPUT_ONLY);
      rtc_gpio_pullup_en(GPIO_NUM_0);    // Ensure HIGH when button released
      rtc_gpio_hold_en(GPIO_NUM_0);      // Hold state during deep sleep
      
      // Enable EXT0 wake-up (single pin, LOW level trigger for button press)
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);  // Wake on LOW (button pressed)
    }
    
    // Timer failsafe wake-up
    esp_sleep_enable_timer_wakeup(timeout_seconds * 1000000ULL);
    esp_deep_sleep_start();  // Never returns
  }

  void checkWakeupReason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    if (wakeup_reason != ESP_SLEEP_WAKEUP_EXT0) {
      // Timer wake-up or hardware reset - always stay awake
      return;
    }
    
    // Button wake-up - check if button is still being held for long press
    int hold_time = 0;
    while (digitalRead(0) == LOW && hold_time < 5000) {  // Button pressed (LOW)
      delay(100);
      hold_time += 100;
    }
    
    if (hold_time < 3000) {  // Less than 3 seconds - ignore wake-up
      Serial.println("Short press ignored, going back to sleep...");
      Serial.flush();
      enterPowerOffSleep(3600, true);  // Go back to deep sleep
      return;
    }
    
    Serial.println("Long press detected, staying awake");
  }

  void powerOff() override {
    // True power-off with long-press wake-up capability
    // Users can wake via: 1) Long button press (3s+) 2) Hardware reset 3) 1-hour emergency timeout
    Serial.println("Entering deep sleep...");
    Serial.flush();  // Ensure message is sent before sleep
    
    enterPowerOffSleep(3600, true);  // 1-hour timeout, enable button wake
  }

  uint16_t getBattMilliVolts() override {
    analogReadResolution(10);
    digitalWrite(PIN_ADC_CTRL, adc_active_state);

    uint32_t raw = 0;
    for (int i = 0; i < 8; i++) {
      raw += analogRead(PIN_VBAT_READ);
    }
    raw = raw / 8;

    digitalWrite(PIN_ADC_CTRL, !adc_active_state);

    return (5.42 * (3.3 / 1024.0) * raw) * 1000;
  }

  const char* getManufacturerName() const override {
    return "Heltec V3";
  }
};
