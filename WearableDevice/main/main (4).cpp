// main_heartrate.cpp - 실제 심박수 측정 코드
// PPG_RDY 인터럽트 + 피크 검출 + BPM 계산

#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// I2C 설정
#define I2C_NUM I2C_NUM_0
#define SDA_PIN 18
#define SCL_PIN 20
#define I2C_FREQ 400000
#define I2C_TIMEOUT_MS 1000

#define TAG "MAX30102_HR"
#define ADDR 0x57

// 레지스터 정의
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONF 0x08
#define REG_MODE 0x09
#define REG_SPO2 0x0A
#define REG_LED1 0x0C
#define REG_LED2 0x0D

// 인터럽트 핀
#define MAX30102_INT_GPIO GPIO_NUM_8

// 심박수 측정 파라미터
#define SAMPLING_RATE 100           // Hz
#define BUFFER_SIZE 200             // 2초 버퍼
#define MIN_PEAK_DISTANCE 40        // 0.4초 = 150 BPM 상한
#define MAX_PEAK_DISTANCE 200       // 2초 = 30 BPM 하한
#define BPM_CALC_INTERVAL 10        // 10초마다 BPM 계산

// 적응적 임계값 (실제 데이터에 기반)
#define ADAPTIVE_THRESHOLD_FACTOR 0.7f
#define SIGNAL_PRESENT_THRESHOLD 100000UL  // 센서에 손가락 있는지 판단

// 전역 변수
static QueueHandle_t s_int_queue = NULL;
static volatile uint32_t s_sample_count = 0;
static volatile uint32_t s_interrupt_count = 0;

// 심박수 측정용 버퍼
static uint32_t ir_buffer[BUFFER_SIZE];
static uint32_t buffer_head = 0;
static bool buffer_ready = false;

// 피크 검출 변수
static uint32_t last_peak_time = 0;
static uint32_t peak_intervals[10] = { 0 };
static uint8_t peak_count = 0;
static float current_bpm = 0.0f;

// 적응적 임계값 변수
static uint32_t signal_min = UINT32_MAX;
static uint32_t signal_max = 0;
static uint32_t adaptive_threshold = 110000;

// I2C 함수들
static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return r;
}

static esp_err_t read_regs(uint8_t reg, uint8_t* buf, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADDR << 1) | I2C_MASTER_READ, true);

    for (size_t i = 0; i < len; i++) {
        i2c_master_read_byte(cmd, &buf[i], (i < len - 1) ? I2C_MASTER_ACK : I2C_MASTER_NACK);
    }

    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return r;
}

// 단일 IR 샘플 읽기
static esp_err_t read_single_ir_sample(uint32_t* ir_value) {
    uint8_t buf[3] = { 0 };
    esp_err_t r = read_regs(REG_FIFO_DATA, buf, 3);

    if (r == ESP_OK) {
        *ir_value = (((uint32_t)buf[0] & 0x03) << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    }

    return r;
}

// 적응적 임계값 업데이트
static void update_adaptive_threshold(uint32_t ir_value) {
    // Min/Max 업데이트 (최근 100 샘플 기준)
    static uint32_t sample_counter = 0;
    sample_counter++;

    if (ir_value < signal_min) signal_min = ir_value;
    if (ir_value > signal_max) signal_max = ir_value;

    // 100 샘플마다 임계값 재계산
    if (sample_counter % 100 == 0) {
        if (signal_max > signal_min) {
            uint32_t signal_range = signal_max - signal_min;
            adaptive_threshold = signal_min + (uint32_t)(signal_range * ADAPTIVE_THRESHOLD_FACTOR);
            ESP_LOGD(TAG, "Adaptive threshold updated: %lu (range: %lu-%lu)",
                (unsigned long)adaptive_threshold,
                (unsigned long)signal_min,
                (unsigned long)signal_max);
        }

        // Min/Max 값 부분 리셋 (적응성 위해)
        signal_min = (signal_min + adaptive_threshold) / 2;
        signal_max = (signal_max + adaptive_threshold) / 2;
    }
}

// 피크 검출 및 BPM 계산
static void detect_peak_and_calculate_bpm(uint32_t ir_value, uint32_t timestamp_ms) {
    // 버퍼에 데이터 저장
    ir_buffer[buffer_head] = ir_value;
    buffer_head = (buffer_head + 1) % BUFFER_SIZE;

    if (!buffer_ready && buffer_head == 0) {
        buffer_ready = true;
        ESP_LOGI(TAG, "Buffer ready for peak detection");
    }

    if (!buffer_ready) return;

    // 현재 샘플이 피크인지 검사 (3점 비교)
    uint32_t prev_idx = (buffer_head + BUFFER_SIZE - 2) % BUFFER_SIZE;
    uint32_t curr_idx = (buffer_head + BUFFER_SIZE - 1) % BUFFER_SIZE;
    uint32_t next_idx = buffer_head;

    uint32_t prev_val = ir_buffer[prev_idx];
    uint32_t curr_val = ir_buffer[curr_idx];
    uint32_t next_val = ir_buffer[next_idx];

    // 피크 조건: 현재값이 임계값 이상 && 이전/다음보다 크거나 같음
    bool is_peak = (curr_val >= adaptive_threshold) &&
        (curr_val >= prev_val) &&
        (curr_val >= next_val);

    if (is_peak) {
        uint32_t interval_ms = timestamp_ms - last_peak_time;

        // 유효한 피크 간격인지 확인 (30-150 BPM 범위)
        if (interval_ms >= (60000 / 150) && interval_ms <= (60000 / 30) && last_peak_time > 0) {
            // 피크 간격 저장
            for (int i = 9; i > 0; i--) {
                peak_intervals[i] = peak_intervals[i - 1];
            }
            peak_intervals[0] = interval_ms;

            if (peak_count < 10) peak_count++;

            // 평균 간격으로 BPM 계산
            if (peak_count >= 3) {
                uint32_t total_interval = 0;
                for (int i = 0; i < peak_count; i++) {
                    total_interval += peak_intervals[i];
                }

                float avg_interval_ms = (float)total_interval / peak_count;
                current_bpm = 60000.0f / avg_interval_ms;

                ESP_LOGI(TAG, "💓 PEAK detected! BPM: %.1f (interval: %.0fms, peaks: %d)",
                    current_bpm, avg_interval_ms, peak_count);
            }
        }

        last_peak_time = timestamp_ms;
    }
}

// 인터럽트 서비스 루틴
static portMUX_TYPE s_spinlock = portMUX_INITIALIZER_UNLOCKED;
static void IRAM_ATTR int_isr(void* arg) {
    portENTER_CRITICAL_ISR(&s_spinlock);
    s_interrupt_count++;
    portEXIT_CRITICAL_ISR(&s_spinlock);

    uint32_t gpio_num = (uint32_t)arg;
    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(s_int_queue, &gpio_num, &hpw);

    if (hpw) {
        portYIELD_FROM_ISR();
    }
}

// 인터럽트 처리 태스크
static void int_task(void* arg) {
    uint32_t io_num;
    uint32_t ir_value;

    while (1) {
        if (!xQueueReceive(s_int_queue, &io_num, portMAX_DELAY)) continue;

        if (io_num != MAX30102_INT_GPIO) continue;

        // 인터럽트 상태 클리어
        uint8_t int1 = 0, int2 = 0;
        if (read_regs(REG_INTR_STATUS_1, &int1, 1) != ESP_OK ||
            read_regs(REG_INTR_STATUS_2, &int2, 1) != ESP_OK) {
            continue;
        }

        // PPG_RDY 인터럽트 처리
        if (int1 & 0x40) {
            if (read_single_ir_sample(&ir_value) == ESP_OK) {
                s_sample_count++;
                uint32_t timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                // 적응적 임계값 업데이트
                update_adaptive_threshold(ir_value);

                // 손가락 접촉 확인
                bool finger_detected = (ir_value > SIGNAL_PRESENT_THRESHOLD);

                if (finger_detected) {
                    // 피크 검출 및 BPM 계산
                    detect_peak_and_calculate_bpm(ir_value, timestamp_ms);

                    // RAW 데이터 출력 (간헐적으로)
                    if (s_sample_count % 20 == 0) {  // 5Hz 출력
                        const char* status = (ir_value >= adaptive_threshold) ? "[PEAK]" : "[NORM]";
                        printf("HR_IR: %6lu %s (BPM: %.1f, Thr: %lu)\n",
                            (unsigned long)ir_value, status, current_bpm,
                            (unsigned long)adaptive_threshold);
                    }
                }
                else {
                    // 손가락 없음
                    if (s_sample_count % 50 == 0) {  // 2Hz 출력
                        printf("NO_FINGER: %lu (threshold: %lu)\n",
                            (unsigned long)ir_value, (unsigned long)SIGNAL_PRESENT_THRESHOLD);
                    }

                    // BPM 리셋
                    current_bpm = 0.0f;
                    peak_count = 0;
                }
            }
        }
    }
}

// I2C 초기화
static esp_err_t i2c_init(void) {
    i2c_config_t cfg = {};
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = SDA_PIN;
    cfg.scl_io_num = SCL_PIN;
    cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = I2C_FREQ;

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, cfg.mode, 0, 0, 0));
    return ESP_OK;
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "💓 MAX30102 Heart Rate Monitor");
    ESP_LOGI(TAG, "═════════════════════════════════");

    // I2C 초기화
    ESP_ERROR_CHECK(i2c_init());
    ESP_LOGI(TAG, "✅ I2C initialized");
    vTaskDelay(pdMS_TO_TICKS(100));

    // MAX30102 초기화
    ESP_LOGI(TAG, "🔧 Initializing MAX30102...");

    // 소프트 리셋
    write_reg(REG_MODE, 0x40);
    vTaskDelay(pdMS_TO_TICKS(100));

    // FIFO 리셋
    write_reg(REG_FIFO_WR_PTR, 0x00);
    write_reg(REG_OVF_COUNTER, 0x00);
    write_reg(REG_FIFO_RD_PTR, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 레지스터 설정
    write_reg(REG_FIFO_CONF, 0x00);     // FIFO 비활성화
    write_reg(REG_MODE, 0x02);          // Heart Rate mode
    write_reg(REG_SPO2, 0x27);          // 16-bit ADC, 100Hz
    write_reg(REG_LED1, 0x24);          // Red LED current
    write_reg(REG_LED2, 0x24);          // IR LED current
    vTaskDelay(pdMS_TO_TICKS(50));

    // GPIO 인터럽트 설정
    ESP_LOGI(TAG, "📌 Setting up interrupt...");
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << MAX30102_INT_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // 큐 및 태스크 생성
    s_int_queue = xQueueCreate(32, sizeof(uint32_t));
    xTaskCreate(int_task, "hr_int", 8192, NULL, 10, NULL);

    // ISR 서비스 설치
    gpio_install_isr_service(0);
    gpio_isr_handler_add(MAX30102_INT_GPIO, int_isr, (void*)MAX30102_INT_GPIO);

    // PPG_RDY 인터럽트 활성화
    write_reg(REG_INTR_ENABLE_1, 0x40);  // PPG_RDY
    write_reg(REG_INTR_ENABLE_2, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "✅ Heart Rate Monitor Ready!");
    ESP_LOGI(TAG, "📍 Features:");
    ESP_LOGI(TAG, "   - Real-time BPM calculation");
    ESP_LOGI(TAG, "   - Adaptive peak detection");
    ESP_LOGI(TAG, "   - Finger presence detection");
    ESP_LOGI(TAG, "   - 100Hz sampling rate");

    ESP_LOGI(TAG, "💓 Place finger on sensor for heart rate measurement!");

    // 상태 모니터링 루프
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // 10초마다

        if (current_bpm > 0) {
            ESP_LOGI(TAG, "❤️  CURRENT BPM: %.1f (Samples: %lu, Peaks: %d)",
                current_bpm, (unsigned long)s_sample_count, peak_count);
        }
        else {
            ESP_LOGI(TAG, "⏳ Waiting for valid heart rate... (Place finger firmly)");
        }
    }
}
