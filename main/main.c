#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

/*
 * Hardware target:
 * - ESP32 DevKit V1 WROVER (3.3V logic)
 * - ACS712 10A (powered at 5V, output centered at VCC/2)
 * - Battery voltage input 0-15V via resistor divider to ADC
 *
 * IMPORTANT WIRING NOTES:
 * 1) ESP32 ADC pins are NOT 5V tolerant.
 * 2) ACS712 output can swing around 2.5V, which is safe for ADC if attenuation is configured.
 * 3) Battery 0-15V must be reduced by a resistor divider so ADC input never exceeds ~3.1V.
 * 4) Grounds must be common: battery -, ACS712 GND, ESP32 GND.
 */

static const char *TAG = "BATTERY_MONITOR";

/* ADC1 is used because ADC2 conflicts with Wi-Fi on ESP32. */
#define CURR_ADC_UNIT          ADC_UNIT_1
#define CURR_ADC_CHANNEL       ADC_CHANNEL_6   // GPIO34 (input only)
#define CURR_ADC_ATTEN         ADC_ATTEN_DB_11
#define CURR_ADC_WIDTH         ADC_WIDTH_BIT_12

#define VBAT_ADC_UNIT          ADC_UNIT_1
#define VBAT_ADC_CHANNEL       ADC_CHANNEL_7   // GPIO35 (input only)
#define VBAT_ADC_ATTEN         ADC_ATTEN_DB_11
#define VBAT_ADC_WIDTH         ADC_WIDTH_BIT_12

/* ACS712-10A sensitivity is typically 185 mV/A. */
#define ACS712_SENSITIVITY_V_PER_A      (0.185f)

/*
 * ACS712 zero-current output is ideally VCC/2.
 * If sensor is powered from 5.0V, zero is ~2.5V.
 * We auto-calibrate offset at startup (recommended).
 */
#define ACS712_NOMINAL_ZERO_VOLT        (2.500f)

/* Battery divider ratio:
 * VBAT --- R_TOP ---+--- R_BOTTOM --- GND
 *                    |
 *                  ADC pin
 *
 * Example values for 15V max:
 * R_TOP = 100k, R_BOTTOM = 27k
 * Divider ratio at ADC node: Vadc = Vbat * (R_BOTTOM / (R_TOP + R_BOTTOM))
 * So Vbat = Vadc * ((R_TOP + R_BOTTOM)/R_BOTTOM)
 */
#define VBAT_R_TOP_OHM                  (100000.0f)
#define VBAT_R_BOTTOM_OHM               (27000.0f)

#define ADC_SAMPLES_PER_READING         (128)
#define OFFSET_CALIBRATION_SAMPLES      (400)
#define MEASURE_PERIOD_MS               (1000)

static esp_adc_cal_characteristics_t adc_chars_curr;
static esp_adc_cal_characteristics_t adc_chars_vbat;

static uint32_t adc1_read_avg(adc1_channel_t channel, uint32_t samples)
{
    uint64_t acc = 0;
    for (uint32_t i = 0; i < samples; ++i) {
        acc += adc1_get_raw(channel);
    }
    return (uint32_t)(acc / samples);
}

static float adc_raw_to_volts(adc1_channel_t channel,
                              esp_adc_cal_characteristics_t *chars,
                              uint32_t samples)
{
    uint32_t raw = adc1_read_avg(channel, samples);
    uint32_t mv = esp_adc_cal_raw_to_voltage(raw, chars);
    return ((float)mv) / 1000.0f;
}

static void configure_adc(void)
{
    /* Configure width once for ADC1. */
    adc1_config_width(ADC_WIDTH_BIT_12);

    /* Configure channels with 11 dB attenuation for up to ~3.1V input range. */
    adc1_config_channel_atten(CURR_ADC_CHANNEL, CURR_ADC_ATTEN);
    adc1_config_channel_atten(VBAT_ADC_CHANNEL, VBAT_ADC_ATTEN);

    /* Characterize both channels with the same ADC unit/atten/width assumptions. */
    esp_adc_cal_characterize(CURR_ADC_UNIT, CURR_ADC_ATTEN, CURR_ADC_WIDTH, 1100, &adc_chars_curr);
    esp_adc_cal_characterize(VBAT_ADC_UNIT, VBAT_ADC_ATTEN, VBAT_ADC_WIDTH, 1100, &adc_chars_vbat);
}

static float calibrate_current_zero_offset(void)
{
    ESP_LOGI(TAG, "Calibrating ACS712 zero-current offset... keep load current at 0A");

    float sum_v = 0.0f;
    for (uint32_t i = 0; i < OFFSET_CALIBRATION_SAMPLES; ++i) {
        sum_v += adc_raw_to_volts(CURR_ADC_CHANNEL, &adc_chars_curr, 16);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    float offset_v = sum_v / OFFSET_CALIBRATION_SAMPLES;
    ESP_LOGI(TAG, "Measured zero-current offset: %.4f V (nominal %.3f V)",
             offset_v, ACS712_NOMINAL_ZERO_VOLT);
    return offset_v;
}

void app_main(void)
{
    configure_adc();

    float current_zero_offset_v = calibrate_current_zero_offset();

    while (1) {
        /* Measure ACS712 output voltage. */
        float v_acs = adc_raw_to_volts(CURR_ADC_CHANNEL, &adc_chars_curr, ADC_SAMPLES_PER_READING);

        /* Convert to current in amperes. */
        float current_a = (v_acs - current_zero_offset_v) / ACS712_SENSITIVITY_V_PER_A;

        /* Small values near zero are mostly sensor/ADC noise. */
        if (fabsf(current_a) < 0.03f) {
            current_a = 0.0f;
        }

        /* Measure divided battery voltage at ADC pin. */
        float v_div = adc_raw_to_volts(VBAT_ADC_CHANNEL, &adc_chars_vbat, ADC_SAMPLES_PER_READING);

        /* Reconstruct battery voltage from divider equation. */
        float divider_gain = (VBAT_R_TOP_OHM + VBAT_R_BOTTOM_OHM) / VBAT_R_BOTTOM_OHM;
        float v_batt = v_div * divider_gain;

        /* Optional computed power. */
        float power_w = v_batt * current_a;

        ESP_LOGI(TAG,
                 "Vacs=%.3f V | I=%.3f A | Vdiv=%.3f V | Vbat=%.3f V | P=%.3f W",
                 v_acs, current_a, v_div, v_batt, power_w);

        vTaskDelay(pdMS_TO_TICKS(MEASURE_PERIOD_MS));
    }
}
