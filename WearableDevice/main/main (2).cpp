// main.cpp — HR + Battery voltage JSON together

#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "solicare_device.hpp"


#define I2C_NUM I2C_NUM_0
#define SDA_PIN 18
#define SCL_PIN 20
#define I2C_FREQ 400000
#define I2C_TIMEOUT_MS 1000

// ========= MAX30102 defines (existing) =========
#define TAG "MAX30102_HR"
#define ADDR 0x57
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
#undef FIFO_CONF_INIT
#define FIFO_CONF_INIT 0x77 // SMP_AVE=8, ROLLOVER=1, A_FULL=7
#define MODE_HEART 0x02
#define SPO2_STD 0x27
#define LED_STD 0x22
#define MAX30102_INT_GPIO GPIO_NUM_8
#define INT_PIN_SEL (1ULL << MAX30102_INT_GPIO)
#define BURST_SZ 8
#define BYTES_PER_SAMPLE 3
#define SR_HZ 100.0f
#define MIN_BPM 40.0f
#define MAX_BPM 200.0f
#define IR_ON_THRESHOLD 50000U
#define IR_OFF_THRESHOLD 35000U
#define STABLE_COUNT 12
#define MIN_RR_MS 340
#define HR_SMOOTH_N 5
#define MA_N 100

// BMI160 =================
#define BMI160_INT1_PIN             GPIO_NUM_4

// BMI160 센서 설정
#define BMI160_I2C_ADDR             0x68      
#define BMI160_REG_CHIP_ID          0x00
#define BMI160_REG_ACC_CONF_ADDR    0x40
#define BMI160_REG_ACC_CONF         0x2C
#define BMI160_REG_ACC_RANGE_ADDR   0x41
#define BMI160_REG_ACC_RANGE        0x08
#define BMI160_REG_GYR_CONF_ADDR    0x42
#define BMI160_REG_GYR_CONF         0x2D
#define BMI160_REG_GYR_RANGE_ADDR   0x43
#define BMI160_REG_GYR_RANGE        0x00
#define BMI160_REG_CMD              0x7E
#define BMI160_REG_PMU_STATUS       0x03
#define BMI160_REG_GYR_X_L          0x0C      
#define BMI160_REG_ACC_X_L          0x12      

// BMI INT 레지스터

#define BMI160_REG_INT_en1          0x51 // LOW_G
#define BMI160_REG_INT_LOWHIGH_dur  0x5A //20ms
#define BMI160_REG_INT_LOWHIGH_th   0x5B //0.4g
#define BMI160_REG_INT_OUT_CTRL     0x53 //active low , push-pull
#define BMI160_REG_INT_MAP          0x55 //INT1 enalbe 
#define BMI160_REG_INT_STATUS_0     0x1E

// BMI160 명령어
#define BMI160_CMD_SOFT_RESET       0xB6
#define BMI160_CMD_ACC_MODE_NORMAL  0x11
#define BMI160_CMD_GYR_MODE_NORMAL  0x15

// 변환 계수 (기본 설정: Accel ±8g, Gyro ±2000 dps)
#define ACCEL_SENSITIVITY           4096.0f
#define GYRO_SENSITIVITY            16.384f   // 32768 / 2000dps



// AHT21B Sensor Address and Commands
#define AHT21B_ADDR 0x38             
#define AHT21B_CMD_GET_STATUS 0x71   
#define AHT21B_CMD_TRIGGER_MEAS 0xAC 
#define AHT21B_CMD_PARAM_BYTE1 0x33
#define AHT21B_CMD_PARAM_BYTE2 0x00


// ======================= databuffer ===============================
struct SensorDataBuffer {
    float bpm = 0.0f;
    float temperature = -99.0f;
    float humidity = -99.0f;
    float voltage_percent = 0.0f;
    bool  fall_detected = false;
    char status[8] = "OFF";
    uint32_t timestamp_ms = 0;
};


static SensorDataBuffer g_sensor_data;
static SemaphoreHandle_t g_data_mutex;

static esp_err_t bmi160_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = { reg_addr, data }; // [0]는 주소, [1]은 데이터
    esp_err_t ret = i2c_master_write_to_device(I2C_NUM, BMI160_I2C_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
    return ret;
}
static esp_err_t bmi160_read_reg(uint8_t reg_addr, uint8_t* data, size_t len) {
    return i2c_master_write_read_device(I2C_NUM, BMI160_I2C_ADDR, &reg_addr, 1, data, len, pdMS_TO_TICKS(1000));
}

static esp_err_t bmi160_sleep_mode() {
    esp_err_t ret;
    ret = bmi160_write_reg(BMI160_REG_CMD, 0x12);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "ACC suspend failed"); return ret; }
    vTaskDelay(pdMS_TO_TICKS(10));
    ret = bmi160_write_reg(BMI160_REG_CMD, 0x17);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "GYR suspend failed"); return ret; }
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;

}
static esp_err_t bmi160_wakeup_mode() {
    esp_err_t ret;  
    ESP_LOGI(TAG, "Setting accelerometer to Normal mode...");
    ret = bmi160_write_reg(BMI160_REG_CMD, BMI160_CMD_ACC_MODE_NORMAL);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "ACC normal failed"); return ret; }
    vTaskDelay(pdMS_TO_TICKS(10));
    ret = bmi160_write_reg(BMI160_REG_CMD, BMI160_CMD_GYR_MODE_NORMAL);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "GYR normal failed"); return ret; }
    vTaskDelay(pdMS_TO_TICKS(80));  
    return ESP_OK;
}

static esp_err_t bmi160_init() {
    uint8_t chip_id = 0;
    esp_err_t ret = bmi160_read_reg(BMI160_REG_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK || chip_id != 0xD1) {
        ESP_LOGE(TAG, "BMI160 not found or wrong chip ID: 0x%02X", chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMI160 found with chip ID: 0x%02X", chip_id);

    ESP_LOGI(TAG, "Performing soft reset...");
    ret = bmi160_write_reg(BMI160_REG_CMD, BMI160_CMD_SOFT_RESET);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Soft reset command failed!"); return ESP_FAIL; }
    vTaskDelay(pdMS_TO_TICKS(250));

    ESP_LOGI(TAG, "Setting accelerometer to configuration...");
    ret = bmi160_write_reg(BMI160_REG_ACC_CONF_ADDR, BMI160_REG_ACC_CONF);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Set Accel Normal mode command failed!"); return ESP_FAIL; }
    vTaskDelay(pdMS_TO_TICKS(250));

    ESP_LOGI(TAG, "Setting accelerometer RANGE...");
    ret = bmi160_write_reg(BMI160_REG_ACC_RANGE_ADDR, BMI160_REG_ACC_RANGE);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Set Accel Normal mode command failed!"); return ESP_FAIL; }
    vTaskDelay(pdMS_TO_TICKS(250));

    ESP_LOGI(TAG, "Setting accelerometer treshold 0.4g");
    ret = bmi160_write_reg(BMI160_REG_INT_LOWHIGH_th, 0x33);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Set Accel Normal mode command failed!"); return ESP_FAIL; }
    vTaskDelay(pdMS_TO_TICKS(250));

    ESP_LOGI(TAG, "Setting accelerometer duration 20ms");
    ret = bmi160_write_reg(BMI160_REG_INT_LOWHIGH_dur, 0x07);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Set Accel Normal mode command failed!"); return ESP_FAIL; }
    vTaskDelay(pdMS_TO_TICKS(250));

    ESP_LOGI(TAG, "Setting accelerometer INT enable");
    ret = bmi160_write_reg(BMI160_REG_INT_en1, 0x08);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Set Accel Normal mode command failed!"); return ESP_FAIL; }
    vTaskDelay(pdMS_TO_TICKS(250));

    ESP_LOGI(TAG, "Setting accelerometer MAPPING INT1 pin");
    ret = bmi160_write_reg(BMI160_REG_INT_MAP, 0x01);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Set Accel Normal mode command failed!"); return ESP_FAIL; }
    vTaskDelay(pdMS_TO_TICKS(250));

    ESP_LOGI(TAG, "Setting INT1 pin Active low , push pull");
    ret = bmi160_write_reg(BMI160_REG_INT_OUT_CTRL, 0x08);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Set Accel Normal mode command failed!"); return ESP_FAIL; }
    vTaskDelay(pdMS_TO_TICKS(250));

   

    // --- 상태 검증 로그 추가 ---
    uint8_t pmu_status = 0;
    bmi160_read_reg(BMI160_REG_PMU_STATUS, &pmu_status, 1);
    ESP_LOGI(TAG, "PMU_STATUS Register (0x03): 0x%02X", pmu_status);
    if (pmu_status < 0x14) {
        ESP_LOGW(TAG, "Warning: PMU status indicates sensors are not in normal mode.");
    }
    else {
        ESP_LOGI(TAG, "PMU status indicates sensors are powered up correctly.");
    }
    // -------------------------

    ESP_LOGI(TAG, "BMI160 initialized successfully");
    return ESP_OK;
}


//====================
static const float a_bp[9] = {1.0f, -1.5610180758007182f, 0.6413515380575631f};
static const float b_bp[9] = {0.020083365564211235f, 0.0f,
                              -0.020083365564211235f};

static QueueHandle_t s_int_queue = NULL;
static volatile uint32_t s_isr_cnt = 0;

static float z1 = 0.0f, z2 = 0.0f;
static float ma_buf[MA_N] = {0};
static int ma_i = 0, ma_cnt = 0;
static float ma_sum = 0.0f;
static float hrBuf[HR_SMOOTH_N] = {0};
static int hrCount = 0, hrIdx = 0;
static bool worn = false;
static int on_cnt = 0, off_cnt = 0;
static float prev_y = 0.0f, last_y = 0.0f;
static float last_peak_ms = -1.0f;
static float noise_ma = 0.0f;
static uint32_t sample_idx = 0;
static std::function<void(float, const char *, uint32_t)> g_emit_bpm;
// ========= AHT21B Sensor Functions (added) =========
static bool aht21b_check_and_init() {
  vTaskDelay(pdMS_TO_TICKS(100)); 
  uint8_t status;
  const uint8_t cmd_buf[] = {0x71};
 
  esp_err_t err =
      i2c_master_write_read_device(I2C_NUM, AHT21B_ADDR, cmd_buf, 1, &status, 1,
                                   pdMS_TO_TICKS(I2C_TIMEOUT_MS));

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "AHT21B not found or comm error: %s", esp_err_to_name(err));
    return false;
  }
  if ((status & 0x18) != 0x18) {
    ESP_LOGW(TAG, "AHT21B not calibrated! Status: 0x%02X", status);
  } else {
    ESP_LOGI(TAG, "AHT21B calibration OK. Status: 0x%02X", status);
  }
  return true;
}

static bool read_aht21b_data(float &temperature, float &humidity) {
  const uint8_t trigger_cmd[] = {0xAC, 0x33, 0x00};
  esp_err_t err = i2c_master_write_to_device(I2C_NUM, AHT21B_ADDR, trigger_cmd,
                                             sizeof(trigger_cmd),
                                             pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  if (err != ESP_OK)
    return false;

  vTaskDelay(pdMS_TO_TICKS(100)); 

  uint8_t read_buf[7] = {0};
  err = i2c_master_read_from_device(I2C_NUM, AHT21B_ADDR, read_buf,
                                    sizeof(read_buf),
                                    pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  if (err != ESP_OK || (read_buf[0] & 0x80)) { 
    return false;
  }

  uint32_t raw_humidity = ((uint32_t)read_buf[1] << 12) |
                          ((uint32_t)read_buf[2] << 4) | (read_buf[3] >> 4);
  uint32_t raw_temperature = (((uint32_t)read_buf[3] & 0x0F) << 16) |
                             ((uint32_t)read_buf[4] << 8) | read_buf[5];

  humidity = (float)raw_humidity * 100.0f / (1 << 20);
  temperature = (float)raw_temperature * 200.0f / (1 << 20) - 50.0f;
  return true;
}

// ========= Battery ADC (integrated from voltagesen.cpp) =========
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

static constexpr int BAT_ADC_GPIO = 0; 
static constexpr auto BAT_USE_ATTEN = ADC_ATTEN_DB_2_5;
static constexpr int BAT_ADC_SAMPLES = 32;
static constexpr float R1_OHMS = 30000.0f;
static constexpr float R2_OHMS = 7500.0f;
static constexpr float DIV_GAIN_INV = (R1_OHMS + R2_OHMS) / R2_OHMS;
static adc_oneshot_unit_handle_t s_bat_adc = nullptr;
static adc_cali_handle_t s_bat_cali = nullptr;
static adc_unit_t s_bat_unit;
static adc_channel_t s_bat_channel;

static bool battery_adc_init_once() {
  if (s_bat_adc)
    return true;
  if (adc_oneshot_io_to_channel(BAT_ADC_GPIO, &s_bat_unit, &s_bat_channel) !=
      ESP_OK) {
    ESP_LOGE(TAG, "Battery adc_oneshot_io_to_channel failed");
    return false;
  }
  adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = s_bat_unit,
                                          .ulp_mode = ADC_ULP_MODE_DISABLE};
  if (adc_oneshot_new_unit(&unit_cfg, &s_bat_adc) != ESP_OK) {
    ESP_LOGE(TAG, "Battery adc_oneshot_new_unit failed");
    return false;
  }
  adc_oneshot_chan_cfg_t chan_cfg = {.atten = BAT_USE_ATTEN,
                                     .bitwidth = ADC_BITWIDTH_DEFAULT};
  if (adc_oneshot_config_channel(s_bat_adc, s_bat_channel, &chan_cfg) !=
      ESP_OK) {
    ESP_LOGE(TAG, "Battery adc_oneshot_config_channel failed");
    return false;
  }
  adc_cali_curve_fitting_config_t cali_cfg = {.unit_id = s_bat_unit,
                                              .chan = s_bat_channel,
                                              .atten = BAT_USE_ATTEN,
                                              .bitwidth = ADC_BITWIDTH_DEFAULT};
  if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_bat_cali) != ESP_OK) {
    ESP_LOGE(TAG, "Battery adc_cali_create_scheme_curve_fitting failed");
    return false;
  }
  ESP_LOGI(TAG, "Battery calibration ready");
  return true;
}

static bool read_battery_voltage(float &v_batt_out) {
  if (!battery_adc_init_once())
    return false;
  int64_t sum_mv = 0;
  int valid = 0;
  for (int i = 0; i < BAT_ADC_SAMPLES; ++i) {
    int mv = 0;
    esp_err_t r = adc_oneshot_get_calibrated_result(s_bat_adc, s_bat_cali,
                                                    s_bat_channel, &mv);
    if (r == ESP_OK) {
      sum_mv += mv;
      valid++;
    } else {
      ESP_LOGW(TAG, "Battery read failed: %d", r);
    }
  }
  if (valid == 0)
    return false;
  float v_out = (float)sum_mv / valid / 1000.0f;
  v_batt_out = v_out * DIV_GAIN_INV;
  return true;
}

// ========= I2C utils (existing) =========
static esp_err_t write_reg(uint8_t reg, uint8_t val) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, val, true);
  i2c_master_stop(cmd);
  esp_err_t r =
      i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  if (r != ESP_OK)
    ESP_LOGE(TAG, "WRITE reg 0x%02X val 0x%02X fail=%s", reg, val,
             esp_err_to_name(r));
  return r;
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (ADDR << 1) | I2C_MASTER_READ, true);
  for (size_t i = 0; i < len; i++) {
    i2c_master_read_byte(cmd, &buf[i],
                         (i < len - 1) ? I2C_MASTER_ACK : I2C_MASTER_NACK);
  }
  i2c_master_stop(cmd);
  esp_err_t r =
      i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  if (r != ESP_OK)
    ESP_LOGE(TAG, "READ reg 0x%02X len %u fail=%s", reg, (unsigned)len,
             esp_err_to_name(r));
  return r;
}

esp_err_t i2c_init() {
  i2c_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.mode = I2C_MODE_MASTER;
  cfg.sda_io_num = SDA_PIN;
  cfg.scl_io_num = SCL_PIN;
  cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.master.clk_speed = I2C_FREQ;
  esp_err_t err = i2c_param_config(I2C_NUM, &cfg);
  if (err != ESP_OK)
    return err;
  err = i2c_driver_install(I2C_NUM, cfg.mode, 0, 0, 0);
  if (err != ESP_OK)
    return err;
  return ESP_OK;
}

static void clear_interrupt_status(void) {
  uint8_t s1 = 0, s2 = 0;
  if (read_regs(REG_INTR_STATUS_1, &s1, 1) == ESP_OK &&
      read_regs(REG_INTR_STATUS_2, &s2, 1) == ESP_OK) {
    ESP_LOGI(TAG, "Cleared INT status: s1=0x%02X s2=0x%02X", s1, s2);
  } else {
    ESP_LOGE(TAG, "Failed to read INT status for clear");
  }
}

static void fifo_reset(void) {
  write_reg(REG_FIFO_WR_PTR, 0x00);
  write_reg(REG_OVF_COUNTER, 0x00);
  write_reg(REG_FIFO_RD_PTR, 0x00);
}

static int fifo_count_debug(void) {
  uint8_t wr = 0, rd = 0, ovf = 0;
  if (read_regs(REG_FIFO_WR_PTR, &wr, 1) != ESP_OK) {
    ESP_LOGE(TAG, "read WR_PTR fail");
    return -1;
  }
  if (read_regs(REG_OVF_COUNTER, &ovf, 1) != ESP_OK) {
    ESP_LOGW(TAG, "read OVF fail");
  }
  if (read_regs(REG_FIFO_RD_PTR, &rd, 1) != ESP_OK) {
    ESP_LOGE(TAG, "read RD_PTR fail");
    return -1;
  }
  int cnt = (int)((wr - rd) & 0x1F);
  ESP_LOGD(TAG, "FIFO ptr wr=%u rd=%u ovf=%u cnt=%d", wr, rd, ovf, cnt);
  return cnt;
}

static esp_err_t fifo_read_n_ir(int n, uint32_t *out_ir) {
  if (n <= 0 || n > BURST_SZ)
    return ESP_ERR_INVALID_ARG;
  uint8_t buf[BURST_SZ * BYTES_PER_SAMPLE] = {0};
  int bytes = n * BYTES_PER_SAMPLE;
  esp_err_t r = read_regs(REG_FIFO_DATA, buf, bytes);
  if (r != ESP_OK)
    return r;
  for (int i = 0; i < n; i++) {
    const uint8_t *p = &buf[i * 3];
    out_ir[i] = (((uint32_t)p[0] & 0x03) << 16) | ((uint32_t)p[1] << 8) | p[2];
  }
  return ESP_OK;
}

// ========= Signal processing (existing) =========
static float dc_remove(float x) {
  ma_sum += x - ma_buf[ma_i];
  ma_buf[ma_i] = x;
  ma_i = (ma_i + 1) % MA_N;
  if (ma_cnt < MA_N)
    ma_cnt++;
  float dc = ma_sum / (float)ma_cnt;
  return x - dc;
}

static float biquad_bp(float x) {
  float y = b_bp[0] * x + z1;
  z1 = b_bp[1] * x - a_bp[1] * y + z2;
  z2 = b_bp[2] * x - a_bp[2] * y;
  return y;
}

static float smooth_hr(float hr) {
  hrBuf[hrIdx] = hr;
  hrIdx = (hrIdx + 1) % HR_SMOOTH_N;
  if (hrCount < HR_SMOOTH_N)
    hrCount++;
  float s = 0.0f;
  for (int i = 0; i < hrCount; i++)
    s += hrBuf[i];
  return s / (float)hrCount;
}

static void reset_filter_states(void) {
  z1 = z2 = 0.0f;
  prev_y = last_y = 0.0f;
  noise_ma = 0.0f;
  for (int i = 0; i < MA_N; i++)
    ma_buf[i] = 0.0f;
  ma_i = 0;
  ma_cnt = 0;
  ma_sum = 0.0f;
  hrCount = 0;
  hrIdx = 0;
  for (int i = 0; i < HR_SMOOTH_N; i++)
    hrBuf[i] = 0.0f;
  last_peak_ms = -1.0f;
}


static void process_sample(uint32_t ir_raw, float t_ms) {
    static bool prev_worn = false;
  if (!worn) {
    if (ir_raw > IR_ON_THRESHOLD) {
      if (++on_cnt > STABLE_COUNT) {
        worn = true;
        on_cnt = 0;
        off_cnt = 0;
        reset_filter_states();
        ESP_LOGI(TAG, "Finger ON");
     
      }
    } else {
      on_cnt = 0;
    }
    return;
  } else {
    if (ir_raw < IR_OFF_THRESHOLD) {
      if (++off_cnt > STABLE_COUNT) {
        worn = false;
        off_cnt = 0;
        on_cnt = 0;
        ESP_LOGW(TAG, "Finger OFF");
        if (g_emit_bpm)
          g_emit_bpm(0.0f, "OFF", (uint32_t)t_ms);
        return;
      }
    } else {
      off_cnt = 0;
    }
    if (worn != prev_worn) {
        if (worn) {
            bmi160_wakeup_mode();  // 착용 시 센서 활성화
        }
        else {
            bmi160_sleep_mode();   // 미착용 시 절전
        }
        prev_worn = worn;
    }
  }

  float x = (float)ir_raw;
  x = dc_remove(x);
  float y = biquad_bp(x);
  float abs_y = fabsf(y);

  noise_ma = 0.97f * noise_ma + 0.03f * abs_y;
  float thr = fmaxf(0.05f, 1.0f * noise_ma);

  if (prev_y < last_y && last_y >= y && last_y > thr) {
    if (last_peak_ms < 0.0f) {
      last_peak_ms = t_ms;
    } else {
      float rr = t_ms - last_peak_ms;
      if (rr > MIN_RR_MS) {
        last_peak_ms = t_ms;
        float hr = 60000.0f / rr;
        float hr_s = smooth_hr(hr);
        if (hr_s >= MIN_BPM && hr_s <= MAX_BPM) {
          ESP_LOGI(TAG, "Heart Rate: %.1f BPM", hr_s);
          if (g_emit_bpm)
            g_emit_bpm(hr_s, "ON", (uint32_t)t_ms);
        } else {
          ESP_LOGW(TAG, "HR out of range: %.1f BPM (raw=%.1f)", hr_s, hr);
        }
      }
    }
  }
  prev_y = last_y;
  last_y = y;
}

// ========= ISR / INT task (existing) =========
static portMUX_TYPE s_spinlock = portMUX_INITIALIZER_UNLOCKED;
static void IRAM_ATTR int_isr(void *arg) {
  portENTER_CRITICAL_ISR(&s_spinlock);
  s_isr_cnt++;
  portEXIT_CRITICAL_ISR(&s_spinlock);
  uint32_t gpio_num = (uint32_t)arg;
  BaseType_t hpw = pdFALSE;
  xQueueSendFromISR(s_int_queue, &gpio_num, &hpw);
  if (hpw)
    portYIELD_FROM_ISR();
}

static void int_task(void *arg) {
  uint32_t io_num;
  uint32_t ir_buf[BURST_SZ];
  const float Ts_ms = 1000.0f / SR_HZ;

  uint8_t int_status_buffer[5];


  while (1) {
      if (!xQueueReceive(s_int_queue, &io_num, portMAX_DELAY)) continue;

      switch (io_num) {
      case MAX30102_INT_GPIO: {
          
          uint8_t int1 = 0, int2 = 0;
          if (read_regs(REG_INTR_STATUS_1, &int1, 1) != ESP_OK ||
              read_regs(REG_INTR_STATUS_2, &int2, 1) != ESP_OK) {
              ESP_LOGE(TAG, "INT_STATUS read fail");
              continue;
          }
          int cnt = fifo_count_debug();
          if (cnt <= 0) continue;
          int left = cnt;
          while (left > 0) {
              int n = (left >= BURST_SZ) ? BURST_SZ : left;
              if (fifo_read_n_ir(n, ir_buf) != ESP_OK) {
                  ESP_LOGE(TAG, "fifo_read_n_ir fail");
                  break;
              }
              for (int i = 0; i < n; i++) {
                  float t_ms = (sample_idx * Ts_ms);
                  process_sample(ir_buf[i], t_ms);
                  sample_idx++;
              }
              left -= n;
          }
          break;
      }

      case BMI160_INT1_PIN: {
          
          gpio_intr_disable(BMI160_INT1_PIN);

          
          if (bmi160_read_reg(BMI160_REG_INT_STATUS_0, int_status_buffer, 4) != ESP_OK) {
              ESP_LOGW(TAG, "BMI160 INT status read failed");
              gpio_intr_enable(BMI160_INT1_PIN);
              break;
          }
          ESP_LOGW(TAG, "INTERRUPT: Possible freefall detected! Checking...");

          // 3) 임시 윈도우 폴링으로 충격/회전 확인 (최대 1초)
          bool impact_detected = false;
          bool rotation_detected = false;
          TickType_t start_time = xTaskGetTickCount();
          while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(1000)) {
              uint8_t data[2];
              if (bmi160_read_reg(BMI160_REG_GYR_X_L, data, 12) != ESP_OK) {
                  ESP_LOGW(TAG, "BMI160 data read failed");
                  vTaskDelay(pdMS_TO_TICKS(20));
                  continue;
              }
              int16_t raw_ax = (int16_t)((data[7] << 8) | data[6]);
              int16_t raw_ay = (int16_t)((data[9] << 8) | data[8]);
              int16_t raw_az = (int16_t)((data[11] << 8) | data[10]);
              int16_t raw_gx = (int16_t)((data[1] << 8) | data[0]);
              int16_t raw_gy = (int16_t)((data[3] << 8) | data[2]);
              int16_t raw_gz = (int16_t)((data[5] << 8) | data[4]);

              float ax = raw_ax / ACCEL_SENSITIVITY;
              float ay = raw_ay / ACCEL_SENSITIVITY;
              float az = raw_az / ACCEL_SENSITIVITY;
              float gx = raw_gx / GYRO_SENSITIVITY;
              float gy = raw_gy / GYRO_SENSITIVITY;
              float gz = raw_gz / GYRO_SENSITIVITY;

              float SVM_A = sqrtf(ax * ax + ay * ay + az * az);
              float SVM_G = sqrtf(gx * gx + gy * gy + gz * gz);

              if (SVM_A > 1.0f)  impact_detected = true;   
              if (SVM_G > 500.0f) rotation_detected = true;  

              ESP_LOGW(TAG, "Checking... Accel: %.2f g, Gyro: %.2f dps", SVM_A, SVM_G);

              if (impact_detected && rotation_detected) {
                  ESP_LOGW(TAG, "Impact and rotation confirmed!");
                  break;
              }
              vTaskDelay(pdMS_TO_TICKS(20));
          }

          
          if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
              if (impact_detected && rotation_detected) {
                  g_sensor_data.fall_detected = true;
              }
              //else {
                  
              //    g_sensor_data.fall_detected = false;
             // }
              xSemaphoreGive(g_data_mutex);
          }

          
          xQueueReset(s_int_queue); 
          gpio_intr_enable(BMI160_INT1_PIN);
          ESP_LOGI(TAG, "System armed. Waiting for next interrupt...");
          break;
      }

      default:
          
          // ESP_LOGW(TAG, "Unknown GPIO INT: %u", (unsigned)io_num);
          break;
      } // switch
  } // while
}




void periodic_sender_task(void* pvParameters) {
    auto* device = static_cast<SolicareDevice*>(pvParameters);
    const TickType_t send_interval_ms = 2000; // 2초마다 데이터 전송

    ESP_LOGI("SENDER_TASK", "Periodic data sender task started.");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(send_interval_ms)); 

        SensorDataBuffer data_to_send;

        
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            data_to_send = g_sensor_data; 
            if (g_sensor_data.fall_detected) {
                g_sensor_data.fall_detected = false;            
            }
            xSemaphoreGive(g_data_mutex); 
        }
        else {
            ESP_LOGW("SENDER_TASK", "Could not get mutex, skipping send cycle.");
            continue;
        }

        ESP_LOGI("SENDER_TASK", "Sending data: BPM %.1f, V %.1f%%, Status: %s",
            data_to_send.bpm, data_to_send.voltage_percent, data_to_send.status);

        device->send_bpm_json(
            data_to_send.bpm,
            data_to_send.temperature,
            data_to_send.humidity,
            data_to_send.voltage_percent,
            data_to_send.fall_detected,
            data_to_send.status,
            data_to_send.timestamp_ms
        );
    }
}

// ========= SolicareDevice changes: send JSON with voltage =========
void SolicareDevice::send_bpm_json(float bpm, float temp, float hum,
                                   float voltage, bool fall_detected ,const char *status,
                                   uint32_t t_ms) {
  if (!socket_client_) {
    ESP_LOGW(TAG, "WebSocket client not ready");
    return;
  }
  if (!socket_client_->is_available()) {
    ESP_LOGW(TAG, "WebSocket not connected, skip send");
    return;
  }

  char json[256];

  int n = snprintf(json, sizeof(json),
      "{\"device\":\"%s\",\"bpm\":%.1f,\"temperature\":%.2f,\"humidity\":%.2f,"
      "\"voltage\":%.3f,\"timestamp_ms\":%u,\"fall_detected\":%s,\"status\":\"%s\"}",
      config_.device_name.c_str(),
      bpm,                
      temp,               
      hum,                 
      voltage,            
      (unsigned)t_ms,     
      fall_detected ? "true" : "false", 
      status              
  );
  


  if (n < 0 || n >= (int)sizeof(json)) {
    ESP_LOGW(TAG, "JSON truncated or format error");
    return;
  }
  std::string json_str(json, n);
  socket_client_->send_text_now(json_str);
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "🚀 Starting Solicare Application");
  ESP_LOGI(TAG, "═══════════════════════════════════════");

  // 1. I2C 버스 초기화 (중앙 관리)
  i2c_config_t i2c_conf = {};
  i2c_conf.mode = I2C_MODE_MASTER;
  i2c_conf.sda_io_num = SDA_PIN;
  i2c_conf.scl_io_num = SCL_PIN;
  i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_conf.master.clk_speed = I2C_FREQ;
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &i2c_conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, i2c_conf.mode, 0, 0, 0));
  ESP_LOGI(TAG, "I2C Master Bus initialized successfully for all sensors");

  // 2. 객체 생성 및 초기화
  SolicareDevice::Config config;
  SolicareDevice device(config);
  if (!device.initialize()) {
    ESP_LOGE(TAG, "❌ Failed to initialize application");
    return;
  }
  // 뮤텍스, 주기적 전송
  g_data_mutex = xSemaphoreCreateMutex();
  if (g_data_mutex == NULL) {
      ESP_LOGE(TAG, "❌ Failed to create data mutex!");
      return;

  }
  device.run();

  xTaskCreate(
      periodic_sender_task,     
      "periodic_sender_task",   
      4096,                     
      &device,                 
      5,                       
      NULL
  );

  // 3. 센서 초기화
  aht21b_check_and_init(); // 온습도 센서 초기화 추가
  battery_adc_init_once(); // 배터리 ADC 초기화
  bmi160_init();

  

  // 4. MAX30102 센서 설정 및 인터럽트 시작 (기존 i2c_init() 호출은 제거됨)
  vTaskDelay(pdMS_TO_TICKS(10));
  clear_interrupt_status();
  fifo_reset();
  vTaskDelay(pdMS_TO_TICKS(5));

  write_reg(REG_FIFO_CONF, FIFO_CONF_INIT);
  write_reg(REG_SPO2, SPO2_STD);
  write_reg(REG_LED1, LED_STD);
  write_reg(REG_LED2, LED_STD);
  write_reg(REG_MODE, MODE_HEART);
  write_reg(REG_INTR_ENABLE_1, 0x80);
  write_reg(REG_INTR_ENABLE_2, 0x00);
  vTaskDelay(pdMS_TO_TICKS(50));

  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = INT_PIN_SEL;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  gpio_config_t io_conf1 = {};
  io_conf1.intr_type = GPIO_INTR_NEGEDGE;
  io_conf1.pin_bit_mask = (1ULL << BMI160_INT1_PIN);
  io_conf1.mode = GPIO_MODE_INPUT;
  io_conf1.pull_up_en = GPIO_PULLUP_ENABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf1));

  s_int_queue = xQueueCreate(16, sizeof(uint32_t));
  xTaskCreate(int_task, "max30102_int_task", 4096, NULL, 10, NULL);

  gpio_install_isr_service(0);
  gpio_isr_handler_add(MAX30102_INT_GPIO, int_isr, (void *)MAX30102_INT_GPIO);
  gpio_isr_handler_add(BMI160_INT1_PIN, int_isr, (void*)BMI160_INT1_PIN);
  ESP_LOGI(TAG, "MAX30102 sampler started. Waiting for interrupts...");

  while (1)
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ========= SolicareDevice::initialize / run definitions =========
bool SolicareDevice::initialize() {
  ESP_LOGI(TAG, "=== Initializing Solicare Device ===");
  ESP_LOGI(TAG, "📶 Initializing WiFi Client...");
  WiFiClient::Config wifi_config(config_.wifi_ssid, config_.wifi_password);
  wifi_client_ = std::make_unique<WiFiClient>(wifi_config);
  if (!wifi_client_->initialize()) {
    ESP_LOGE(TAG, "❌ Failed to initialize WiFi Client");
    return false;
  }
  ESP_LOGI(TAG, "✅ WiFi Client initialized");

  ESP_LOGI(TAG, "🌐 Initializing WebSocket client...");
  WebSocketClient::Config ws_config(config_.device_name,
                                    config_.socket_server_ip,
                                    config_.socket_server_port);
  socket_client_ = std::make_unique<WebSocketClient>(
      ws_config, [this]() { return wifi_client_->is_available(); });
  if (!socket_client_->initialize()) {
    ESP_LOGE(TAG, "❌ Failed to initialize WebSocket client");
    return false;
  }
  ESP_LOGI(TAG, "✅ WebSocket client initialized and ready to connect as '%s'",
           config_.device_name.c_str());
  ESP_LOGI(TAG, "=== Solicare Device initialized successfully ===");
  return true;
}

void SolicareDevice::run() {
  ESP_LOGI(TAG, "=== Starting Solicare Device Runtime ===");
  wifi_client_->try_connect();
  socket_client_->async_connect();
  socket_client_->start_sender_task();

  
  g_emit_bpm = [this](float bpm, const char *status, uint32_t t_ms) {
      if (xSemaphoreTake(g_data_mutex, portMAX_DELAY) == pdTRUE) {

         
          read_aht21b_data(g_sensor_data.temperature, g_sensor_data.humidity);


          float vbatt = 0.0f;
          if (read_battery_voltage(vbatt)) {
              float percentage = (vbatt - 2.5f) / (4.2f - 2.5f) * 100.0f;
              g_sensor_data.voltage_percent = fmax(0.0f, fmin(100.0f, percentage)); // 0~100% 범위로 제한
          }
          else {
              g_sensor_data.voltage_percent = 0.0f; // 배터리 읽기 실패
              ESP_LOGW(TAG, "Battery read failed, storing voltage=0.0%%");
          }

          //float corrected_bpm = bpm;
          //if (strcmp(status, "ON") == 0) { // "ON" 상태일 때만 보정 적용
          //    corrected_bpm -= 0.0f;
          //    if (corrected_bpm < 0) corrected_bpm = 0;
          //}

         
          g_sensor_data.bpm = bpm;
          strncpy(g_sensor_data.status, status, sizeof(g_sensor_data.status) - 1);
          g_sensor_data.timestamp_ms = t_ms;

         
          xSemaphoreGive(g_data_mutex);
      }
      };

}
//#include <cstring>
//#include "esp_log.h"
//#include "driver/i2c.h"
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//
//// 프로젝트에서 쓰던 정의 재사용
//#define I2C_NUM        I2C_NUM_0
//#define SDA_PIN        18
//#define SCL_PIN        20
//#define I2C_FREQ       400000
//#define I2C_TIMEOUT_MS 1000 
//
//static const char* TAG = "APP";
//static const char* TAG_SCAN = "I2C_SCAN";
//
//static void i2c_master_init(void)
//{
//    i2c_config_t conf = {};
//    conf.mode = I2C_MODE_MASTER;
//    conf.sda_io_num = SDA_PIN;
//    conf.scl_io_num = SCL_PIN;
//    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//#if ESP_IDF_VERSION_MAJOR >= 5
//    conf.clk_flags = 0;
//#endif
//    conf.master.clk_speed = I2C_FREQ;
//
//    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &conf));
//    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, conf.mode, 0, 0, 0));
//    ESP_LOGI(TAG, "I2C init OK (SDA=%d, SCL=%d, F=%lu)", SDA_PIN, SCL_PIN, (unsigned long)I2C_FREQ);
//}
//
//static void i2c_scan_bus(void)
//{
//    ESP_LOGI(TAG_SCAN, "Scanning I2C bus on port %d ...", I2C_NUM);
//    int found = 0;
//    for (uint8_t addr = 1; addr < 0x7F; ++addr) {
//        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//        i2c_master_start(cmd);
//        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
//        i2c_master_stop(cmd);
//        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(50));
//        i2c_cmd_link_delete(cmd);
//
//        if (ret == ESP_OK) {
//            ESP_LOGI(TAG_SCAN, "Device @ 0x%02X", addr);
//            found++;
//        }
//        else if (ret != ESP_FAIL) {
//            ESP_LOGW(TAG_SCAN, "Error @ 0x%02X: %s", addr, esp_err_to_name(ret));
//        }
//        vTaskDelay(pdMS_TO_TICKS(2));
//    }
//    if (found == 0) ESP_LOGI(TAG_SCAN, "No I2C devices found");
//    else ESP_LOGI(TAG_SCAN, "Scan complete: %d device(s)", found);
//}
//
//// C++ 파일에선 반드시 C 링키지로 내보내기
//extern "C" void app_main(void)
//{
//    ESP_LOGI(TAG, "==== Boot ====");
//    i2c_master_init();
//    i2c_scan_bus();
//
//    // 여기에 기존 센서 초기화/태스크 생성 코드 이어서 배치
//    // ...
//    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
//}
