/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Bike Computer: BMI160 + BME280 + SSD1306 + Hall + Button + Power
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "bmi160.h"
#include "bmi160_port.h"

#include "bme280.h"
#include "bme280_defs.h"

#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "hall.h"
#include "stm32l4xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct { float alt_m; float trip_m; uint32_t t_ms; } sample_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACC_LSB_PER_G_2G        16384.0f
#define GYR_LSB_PER_DPS_250     131.2f

#define IMU_PERIOD_MS           200u    /* IMU tick */
#define BME_PERIOD_MS           1000u   /* BME tick */

#define WHEEL_CIRCUMFERENCE_M   2.096f  /* calibrate to your wheel */
#define HALL_DEBOUNCE_MS        12u
#define HALL_STOP_TIMEOUT_MS    2000u   /* no pulses for this time => standstill */
#define HALL_EMA_TAU_S          0.6f    /* EMA time constant [s] */
#define SPEED_MIN_MOVING_KMH    1.0f    /* autopause threshold */

#define MIN_DT_MS               1u
#define MAX_SPEED_KMH           120.0f  /* sanity limit */

#define ALT_CALIBRATION_M       0.0f    /* 0 => ALT=0 on start; or put known AMSL altitude */

#define GRAD_WINDOW_S           5u      /* gradient window 5 s */
#define VAM_WINDOW_S            30u     /* VAM window 30 s */
#define PRESS_TREND_MIN         15u     /* pressure trend (minutes) */

#define HIST5_LEN               (GRAD_WINDOW_S*5u)      /* 5 Hz */
#define HIST30_LEN              (VAM_WINDOW_S*5u)       /* 5 Hz */
#define PTREND_LEN              (PRESS_TREND_MIN*60u)   /* 1 Hz */

/* Button – uses Switch_Pin / Switch_GPIO_Port from main.h */
#define BTN_PORT        Switch_GPIO_Port
#define BTN_PIN         Switch_Pin
#define BTN_DEBOUNCE_MS 150u

/* Vibration RMS window (2 s at 5 Hz) */
#define VIB_LEN         10u

#define ICON_PLAY               ">"
#define ICON_PAUSE              "||"

/* --- Power model constants --- */
#define MASS_TOTAL_KG   90.0f   /* rider + bike mass, change for yourself */
#define C_RR            0.004f  /* rolling resistance coefficient */
#define RHO_AIR         1.20f   /* air density [kg/m^3] */
#define CD_AREA         0.35f   /* CdA [m^2] */
#define HUMAN_EFF       0.25f   /* mechanical efficiency (25%) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG(x) ((x) * (180.0f/(float)M_PI))
#define CLAMP(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))

#ifndef BME280_REG_CHIP_ID
#define BME280_REG_CHIP_ID   0xD0
#define BME280_REG_CTRL_HUM  0xF2
#define BME280_REG_STATUS    0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG    0xF5
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static struct bmi160_dev dev_bmi;
static struct bme280_dev dev_bme;

static uint16_t bme_addr = (0x76u << 1);

static uint32_t t_last_imu = 0;
static uint32_t t_last_bme = 0;

/* --- Ride / trip --- */
volatile uint32_t g_hall_count    = 0;
volatile uint32_t g_last_pulse_ms = 0;

static hall_state_t g_hall;

static float    g_speed_kmh       = 0.0f;   /* displayed speed */
static float    g_speed_filt_kmh  = 0.0f;   /* filtered speed (EMA) */
static float    g_speed_inst_kmh  = 0.0f;   /* instantaneous speed */
static float    g_max_speed_kmh   = 0.0f;
static float    g_trip_m          = 0.0f;   /* distance [m] */
static uint32_t g_moving_ms       = 0;      /* moving time (autopause) */
static uint8_t  g_autopause       = 1;      /* 1 = pause */

/* --- Altimeter / climb --- */
static float    g_p0_hpa          = 1013.25f;  /* reference pressure */
static uint8_t  g_p0_set          = 0;
static float    g_alt_m           = 0.0f;
static float    g_prev_alt_m      = 0.0f;
static float    g_grade_pct       = 0.0f;
static float    g_vam_mph         = 0.0f;      /* m/h */
static float    g_total_ascent_m  = 0.0f;

static sample_t hist5[HIST5_LEN];   static uint16_t h5_idx=0,  h5_cnt=0;
static sample_t hist30[HIST30_LEN]; static uint16_t h30_idx=0, h30_cnt=0;

/* --- Environment --- */
static float g_temp_c = 0.0f, g_press_hpa = 0.0f, g_hum_rh = 0.0f;
static float g_dewpoint_c = 0.0f;
static float g_press_trend_dhpa = 0.0f;  /* change in 15 min */
static char  g_press_trend_char  = '~';  /* '^', 'v', '-', '~' */
static float p_hist[PTREND_LEN]; static uint16_t p_idx=0, p_cnt=0;

/* --- IMU --- */
static float g_ax=0, g_ay=0, g_az=0;     /* [g] */
static float g_gx=0, g_gy=0, g_gz=0;     /* [dps] */
static float g_t_imu_c=0.0f;
static float g_roll_deg=0.0f, g_pitch_deg=0.0f;

static float vib_sq[VIB_LEN]; static uint8_t vib_i=0, vib_n=0;
static float g_vibr_rms_g = 0.0f;

/* --- Power / energy --- */
static float g_power_w        = 0.0f;  /* instantaneous total power [W] */
static float g_power_avg_w    = 0.0f;  /* average power over moving time [W] */
static float g_kcal_total     = 0.0f;  /* total energy [kcal] */
static float g_mech_energy_J  = 0.0f;  /* cumulative mechanical energy [J] */

/* --- UI --- */
enum { PAGE_RIDE=0, PAGE_CLIMB, PAGE_ENV, PAGE_IMU, PAGE_STATS, PAGE_COUNT };
static   uint8_t  g_ui_page = PAGE_RIDE;

/* Button logic (polling) */
static uint8_t  g_btn_pressed = 0;         // logical pressed state
static uint8_t  g_btn_event   = 0;         // new press event
static uint32_t g_btn_last_change_ms = 0;  // for debounce
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void uart_puts(const char *s);
static void uart_printf(const char *fmt, ...);
static void i2c_scan_print(void);

/* BMI160 */
static void  bmi160_init_polling(void);
static int   bmi160_read_acc_gyr(float *ax_g, float *ay_g, float *az_g,
                                 float *gx_dps, float *gy_dps, float *gz_dps);
static float bmi160_read_temperature_c(void);

/* BME280 – API + fallback */
static int  bme280_probe_and_init(void);
static int  bme280_read(float *temp_c, float *press_hpa, float *hum_rh);

/* Altitude & meteo helpers */
static void  update_alt_grad_vam_from_samples(void);
static float dewpoint_c(float T_c, float RH);
static void  update_pressure_trend(float p_hpa);

/* OLED / UI */
static void oled_init_and_banner(void);
static void UI_DrawRide(void);
static void UI_DrawClimb(void);
static void UI_DrawEnv(void);
static void UI_DrawIMU(void);
static void UI_DrawStats(void);

/* Button polling */
static uint8_t BTN_IsPressedRaw(void);
static void    BTN_PollTask(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart_puts(const char *s) {
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

static void uart_printf(const char *fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n > 0) {
    if (n > (int)sizeof(buf)) n = (int)sizeof(buf);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
  }
}

static void i2c_scan_print(void) {
  uart_puts("I2C:");
  for (uint8_t a = 1; a < 0x7F; ++a) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (a<<1), 1, 5) == HAL_OK)
      uart_printf(" %02X", a);
  }
  uart_puts("\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();

  HAL_TIM_Base_Start_IT(&htim2);  /* Timer for speed / power / alt updates */

  /* Hall processing init (speed/distance from period between pulses) */
  Hall_Init(&g_hall, WHEEL_CIRCUMFERENCE_M);
  Hall_SetParams(&g_hall, HALL_DEBOUNCE_MS, HALL_STOP_TIMEOUT_MS, MAX_SPEED_KMH, HALL_EMA_TAU_S);

  uart_puts("\r\n--- Bike Computer start ---\r\n");
  i2c_scan_print();

  /* OLED */
  oled_init_and_banner();

  /* BMI160 */
  bmi160_port_init_i2c(&dev_bmi);
  bmi160_init_polling();

  /* BME280 */
  if (bme280_probe_and_init() == 0) uart_puts("BME280 OK\r\n");
  else                              uart_puts("BME280 NOT FOUND\r\n");

  /* CSV header (log) */
  uart_puts("time_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_IMU_C,"
            "Temp_C,Press_hPa,Hum_%RH,Alt_m,Grad_pct,VAM_mph,Ascent_m,"
            "Speed_kmh,Trip_km,Moving_s,AvgMov_kmh,Max_kmh,Power_W,AvgPow_W,Kcal\r\n");

  /* Main loop */
  t_last_imu = t_last_bme = HAL_GetTick();
  while (1) {
    uint32_t now = HAL_GetTick();

    /* Hall maintenance: apply stop timeout only (avoid decaying speed between valid pulses) */
    if (g_hall.last_pulse_ms != 0u && (uint32_t)(now - g_hall.last_pulse_ms) > g_hall.stop_timeout_ms) {
      Hall_TaskMs(&g_hall, now);
    }

    /* --- BUTTON: priority, handle first --- */
    BTN_PollTask();
    if (g_btn_event) {
      g_btn_event = 0;
      g_ui_page = (uint8_t)((g_ui_page + 1u) % PAGE_COUNT);
      uart_printf("BTN EVENT -> page=%u\r\n", (unsigned)g_ui_page);
    }

    /* IMU every 200 ms */
    if ((now - t_last_imu) >= IMU_PERIOD_MS) {
      (void)bmi160_read_acc_gyr(&g_ax,&g_ay,&g_az,&g_gx,&g_gy,&g_gz);
      g_t_imu_c = bmi160_read_temperature_c();

      /* Roll/Pitch from accelerometer, simple LPF */
      float roll  = DEG(atan2f(g_ay, g_az));
      float pitch = DEG(atan2f(-g_ax, sqrtf(g_ay*g_ay + g_az*g_az)));
      const float a_ang = 0.2f;
      g_roll_deg  = a_ang*roll  + (1.0f-a_ang)*g_roll_deg;
      g_pitch_deg = a_ang*pitch + (1.0f-a_ang)*g_pitch_deg;

      /* Vibration RMS: HP(|a|-1g), 2 s window */
      float amag = sqrtf(g_ax*g_ax + g_ay*g_ay + g_az*g_az);
      float hp   = fabsf(amag - 1.0f);
      vib_sq[vib_i] = hp*hp;
      vib_i = (uint8_t)((vib_i+1u) % VIB_LEN);
      if (vib_n < VIB_LEN) vib_n++;
      float sum=0.0f;
      for (uint8_t i=0;i<vib_n;i++) sum += vib_sq[i];
      g_vibr_rms_g = sqrtf(sum / (float)vib_n);

      t_last_imu = now;
    }

    /* BME every 1 s */
    if ((now - t_last_bme) >= BME_PERIOD_MS) {
      if (bme280_read(&g_temp_c, &g_press_hpa, &g_hum_rh) == 0) {
        /* Calibrate P0 at start */
        if (!g_p0_set) {
          if (ALT_CALIBRATION_M != 0.0f) {
            g_p0_hpa = g_press_hpa / powf(1.0f - (ALT_CALIBRATION_M/44330.0f), 5.255f);
          } else {
            g_p0_hpa = g_press_hpa; /* ALT=0 at start */
          }
          g_p0_set = 1;
        }

        /* Altitude (ISA-like) */
        float ratio = CLAMP(g_press_hpa / g_p0_hpa, 0.001f, 2.0f);
        g_prev_alt_m = g_alt_m;
        g_alt_m = 44330.0f * (1.0f - powf(ratio, 0.19029495f));

        /* Total ascent: only positive dh and while moving */
        float dh = g_alt_m - g_prev_alt_m;
        if (dh > 0.25f && g_speed_kmh >= SPEED_MIN_MOVING_KMH) {
          g_total_ascent_m += dh;
        }

        /* Dew point & pressure trend */
        g_dewpoint_c = dewpoint_c(g_temp_c, g_hum_rh);
        update_pressure_trend(g_press_hpa);
      }
      t_last_bme = now;
    }

    /* UI – draw active page */
    switch (g_ui_page) {
      default:
      case PAGE_RIDE:  UI_DrawRide();  break;
      case PAGE_CLIMB: UI_DrawClimb(); break;
      case PAGE_ENV:   UI_DrawEnv();   break;
      case PAGE_IMU:   UI_DrawIMU();   break;
      case PAGE_STATS: UI_DrawStats(); break;
    }

    /* CSV log every ~50 ms */
    float avg_mov = (g_moving_ms>0)
        ? ( (g_trip_m/1000.0f) / ((float)g_moving_ms/3600000.0f) )
        : 0.0f;

    uart_printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,"
                "%.2f,%.2f,%.2f,%.2f,%.1f,%.0f,%.1f,"
                "%.2f,%.3f,%lu,%.2f,%.2f,%.1f,%.1f,%.1f\r\n",
      (unsigned long)now,
      g_ax,g_ay,g_az,g_gx,g_gy,g_gz,g_t_imu_c,
      g_temp_c,g_press_hpa,g_hum_rh,g_alt_m,g_grade_pct,g_vam_mph,
      g_total_ascent_m,g_speed_kmh,g_trip_m/1000.0f,(unsigned long)(g_moving_ms/1000U),
      avg_mov,g_max_speed_kmh,g_power_w,g_power_avg_w,g_kcal_total
    );
    LowPower_EnterStop2();
    //LowPower_EnterSleep();
    //HAL_Delay(50);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* ==================== BMI160 ==================== */
static void bmi160_init_polling(void)
{
  if (bmi160_init(&dev_bmi) != BMI160_OK) {
    uart_puts("BMI160 init FAIL\r\n");
    Error_Handler();
  }
  dev_bmi.accel_cfg.odr   = BMI160_ACCEL_ODR_25HZ;
  dev_bmi.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
  dev_bmi.accel_cfg.bw    = BMI160_ACCEL_BW_NORMAL_AVG4;
  dev_bmi.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

  dev_bmi.gyro_cfg.odr    = BMI160_GYRO_ODR_25HZ;
  dev_bmi.gyro_cfg.range  = BMI160_GYRO_RANGE_250_DPS;
  dev_bmi.gyro_cfg.bw     = BMI160_GYRO_BW_NORMAL_MODE;
  dev_bmi.gyro_cfg.power  = BMI160_GYRO_NORMAL_MODE;

  if (bmi160_set_sens_conf(&dev_bmi) != BMI160_OK) {
    uart_puts("BMI160 cfg FAIL\r\n");
    Error_Handler();
  }
}

static int bmi160_read_acc_gyr(float *ax_g, float *ay_g, float *az_g,
                               float *gx_dps, float *gy_dps, float *gz_dps)
{
  struct bmi160_sensor_data a, g;
  if (bmi160_get_sensor_data(BMI160_ACCEL_SEL|BMI160_GYRO_SEL, &a, &g, &dev_bmi) != BMI160_OK)
    return -1;

  *ax_g = a.x / ACC_LSB_PER_G_2G;
  *ay_g = a.y / ACC_LSB_PER_G_2G;
  *az_g = a.z / ACC_LSB_PER_G_2G;
  *gx_dps = g.x / GYR_LSB_PER_DPS_250;
  *gy_dps = g.y / GYR_LSB_PER_DPS_250;
  *gz_dps = g.z / GYR_LSB_PER_DPS_250;
  return 0;
}

/* TEMP_LSB(0x20)/MSB(0x21) -> T[°C] ≈ 23 + raw/512 */
static float bmi160_read_temperature_c(void)
{
  int16_t raw = 0;
  (void)bmi160_get_regs(0x20, (uint8_t*)&raw, 2, &dev_bmi);
  return 23.0f + ((float)raw / 512.0f);
}

/* ==================== BME280: I2C glue & fallback ==================== */
static int8_t bme_i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr)
{
  uint16_t addr = *(uint16_t*)intf_ptr;
  return (HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT,
                           data, (uint16_t)len, 100) == HAL_OK) ? 0 : -1;
}

static int8_t bme_i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr)
{
  uint16_t addr = *(uint16_t*)intf_ptr;
  return (HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT,
                            (uint8_t*)data, (uint16_t)len, 100) == HAL_OK) ? 0 : -1;
}

static void bme_delay_us(uint32_t us, void *intf_ptr)
{
  (void)intf_ptr;
  HAL_Delay((us + 999U)/1000U);
}

static int bme_try_addr(uint16_t addr7)
{
  uint8_t id = 0;
  uint16_t addr = (uint16_t)(addr7 << 1);
  dev_bme.intf     = BME280_I2C_INTF;
  dev_bme.read     = bme_i2c_read;
  dev_bme.write    = bme_i2c_write;
  dev_bme.delay_us = bme_delay_us;
  dev_bme.intf_ptr = &bme_addr;
  bme_addr = addr;

  if (bme_i2c_read(BME280_REG_CHIP_ID, &id, 1, &bme_addr) != 0 || id != BME280_CHIP_ID)
    return -1;

  if (bme280_init(&dev_bme) != BME280_OK) return -2;

#if defined(BME280_OSR_TEMP_SEL) && defined(BME280_FILTER_SEL)
  struct bme280_settings s;
  memset(&s, 0, sizeof(s));
  s.osr_t        = BME280_OVERSAMPLING_2X;
  s.osr_p        = BME280_OVERSAMPLING_2X;
  s.osr_h        = BME280_OVERSAMPLING_2X;
  s.filter       = BME280_FILTER_COEFF_4;
  s.standby_time = BME280_STANDBY_TIME_1000_MS;

  uint8_t sel = (uint8_t)(BME280_OSR_TEMP_SEL | BME280_OSR_PRESS_SEL |
                          BME280_OSR_HUM_SEL  | BME280_FILTER_SEL    |
                          BME280_STANDBY_SEL);

  if (bme280_set_sensor_settings(sel, &s, &dev_bme) != BME280_OK) return -3;

# ifdef BME280_NORMAL_MODE
  if (bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev_bme) != BME280_OK) return -4;
# else
  uint8_t ctrl_meas = (uint8_t)((2u<<5) | (2u<<2) | 3u);
  (void)bme_i2c_write(BME280_REG_CTRL_MEAS, &ctrl_meas, 1, &bme_addr);
# endif

#else
  uint8_t v = 0x02; /* osrs_h=2x */
  (void)bme_i2c_write(BME280_REG_CTRL_HUM, &v, 1, &bme_addr);
  v = 0xA8;         /* t_sb=1000ms, filter=4 */
  (void)bme_i2c_write(BME280_REG_CONFIG, &v, 1, &bme_addr);
  v = 0x4B;         /* osrs_t=2x, osrs_p=2x, mode=normal */
  (void)bme_i2c_write(BME280_REG_CTRL_MEAS, &v, 1, &bme_addr);
#endif

  return 0;
}

static int bme280_probe_and_init(void)
{
  if (bme_try_addr(0x76) == 0) return 0;
  return bme_try_addr(0x77);
}

static int bme280_read(float *temp_c, float *press_hpa, float *hum_rh)
{
  struct bme280_data d = {0};
#ifdef BME280_ALL
  if (bme280_get_sensor_data(BME280_ALL, &d, &dev_bme) != BME280_OK) return -1;
#else
  if (bme280_get_sensor_data(0x07, &d, &dev_bme) != BME280_OK) return -1;
#endif
  *temp_c    = d.temperature;
  *press_hpa = d.pressure / 100.0f;
  *hum_rh    = d.humidity;
  return 0;
}

/* ==================== Helpers ==================== */
static float dewpoint_c(float T_c, float RH)
{
  /* Magnus-Tetens */
  const float a = 17.62f, b = 243.12f;
  float gamma = (a*T_c)/(b+T_c) + logf(CLAMP(RH,1.0f,100.0f)/100.0f);
  return (b*gamma)/(a - gamma);
}

static void update_pressure_trend(float p_hpa)
{
  p_hist[p_idx] = p_hpa;
  p_idx = (uint16_t)((p_idx + 1u) % PTREND_LEN);
  if (p_cnt < PTREND_LEN) p_cnt++;

  if (p_cnt >= PTREND_LEN) {
    uint16_t old_idx = (uint16_t)(p_idx % PTREND_LEN);
    float dp = p_hpa - p_hist[old_idx];
    g_press_trend_dhpa = dp;
    if      (dp >  0.5f) g_press_trend_char = '^';
    else if (dp < -0.5f) g_press_trend_char = 'v';
    else                 g_press_trend_char = '-';
  } else {
    g_press_trend_char = '~'; /* not enough data */
  }
}

/* Store samples and compute GRAD / VAM (called from TIM2 every ~200 ms) */
static void update_alt_grad_vam_from_samples(void)
{
  uint32_t now = HAL_GetTick();

  /* push sample into 5s / 30s buffers */
  hist5[h5_idx]   = (sample_t){ .alt_m=g_alt_m, .trip_m=g_trip_m, .t_ms=now };
  h5_idx = (uint16_t)((h5_idx+1u) % HIST5_LEN);
  if (h5_cnt < HIST5_LEN) h5_cnt++;

  hist30[h30_idx] = (sample_t){ .alt_m=g_alt_m, .trip_m=g_trip_m, .t_ms=now };
  h30_idx = (uint16_t)((h30_idx+1u) % HIST30_LEN);
  if (h30_cnt < HIST30_LEN) h30_cnt++;

  /* Gradient: 5 s window */
  if (h5_cnt >= HIST5_LEN) {
    uint16_t old = (uint16_t)(h5_idx % HIST5_LEN);
    float ds = g_trip_m - hist5[old].trip_m; /* [m] */
    float dh = g_alt_m  - hist5[old].alt_m;  /* [m] */
    if (ds > 1.0f) g_grade_pct = CLAMP(100.0f * (dh/ds), -30.0f, 30.0f);
    else           g_grade_pct = 0.0f;
  }

  /* VAM: 30 s window (m/h) */
  if (h30_cnt >= HIST30_LEN) {
    uint16_t old = (uint16_t)(h30_idx % HIST30_LEN);
    float dt = (float)(now - hist30[old].t_ms) / 1000.0f; /* s */
    float dh = g_alt_m - hist30[old].alt_m;               /* m */
    if (dt > 1.0f) g_vam_mph = (dh / dt) * 3600.0f;
    else           g_vam_mph = 0.0f;
  }
}

/* ==================== OLED / UI ==================== */
static void oled_init_and_banner(void)
{
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);  ssd1306_WriteString("STM32 BikeComp", Font_7x10, White);
  ssd1306_SetCursor(0,12);  ssd1306_WriteString("Ride/Climb/Env", Font_7x10, White);
  ssd1306_SetCursor(0,24);  ssd1306_WriteString("IMU/Stats: BTN", Font_7x10, White);
  ssd1306_UpdateScreen();
}

static void UI_DrawRide(void)
{
  char ln[32];
  float avg_mov = (g_moving_ms>0)
      ? ( (g_trip_m/1000.0f) / ((float)g_moving_ms/3600000.0f) )
      : 0.0f;

  ssd1306_Fill(Black);

  /* Speed – big digits */
  ssd1306_SetCursor(0, 0);
  snprintf(ln, sizeof(ln), "%5.1f", g_speed_kmh);
  ssd1306_WriteString(ln, Font_11x18, White);
  ssd1306_SetCursor(90, 0);
  ssd1306_WriteString("km/h", Font_6x8, White);

  /* Distance */
  ssd1306_SetCursor(0, 22);
  snprintf(ln, sizeof(ln), "Distance %5.2f km", g_trip_m/1000.0f);
  ssd1306_WriteString(ln, Font_7x10, White);

  /* Moving time */
  ssd1306_SetCursor(0, 34);
  uint32_t s  = g_moving_ms/1000U;
  uint32_t hh = s/3600U, mm=(s/60U)%60U, ss=s%60U;
  snprintf(ln, sizeof(ln), "Ride %2lu:%02lu:%02lu",
           (unsigned long)hh,(unsigned long)mm,(unsigned long)ss);
  ssd1306_WriteString(ln, Font_7x10, White);

  /* Power & kcal + pause icon */
  ssd1306_SetCursor(0, 46);
  snprintf(ln, sizeof(ln), "Power %3.0f W  %4.0f kcal", g_power_w, g_kcal_total);
  ssd1306_WriteString(ln, Font_7x10, White);

  ssd1306_SetCursor(110, 34);
  ssd1306_WriteString(g_autopause ? ICON_PAUSE : ICON_PLAY, Font_7x10, White);

  ssd1306_UpdateScreen();
}

static void UI_DrawClimb(void)
{
  char ln[32];
  ssd1306_Fill(Black);

  ssd1306_SetCursor(0, 0);
  snprintf(ln,sizeof(ln),"Altitude %6.1f m", g_alt_m);
  ssd1306_WriteString(ln, Font_7x10, White);

  ssd1306_SetCursor(0, 12);
  snprintf(ln,sizeof(ln),"Gradient %5.1f %%", g_grade_pct);
  ssd1306_WriteString(ln, Font_7x10, White);

  ssd1306_SetCursor(0, 24);
  snprintf(ln,sizeof(ln),"Total Ascent %6.0f m", g_total_ascent_m);
  ssd1306_WriteString(ln, Font_7x10, White);

  ssd1306_SetCursor(0, 36);
  snprintf(ln,sizeof(ln),"VAM %5.0f m/h", g_vam_mph);
  ssd1306_WriteString(ln, Font_7x10, White);

  ssd1306_UpdateScreen();
}

static void UI_DrawEnv(void)
{
  char ln[32];
  ssd1306_Fill(Black);

  ssd1306_SetCursor(0, 0);
  snprintf(ln,sizeof(ln),"Air Temp: %.1f C", g_temp_c);
  ssd1306_WriteString(ln, Font_7x10, White);

  ssd1306_SetCursor(0, 12);
  snprintf(ln,sizeof(ln),"Humidity: %.0f %%", g_hum_rh);
  ssd1306_WriteString(ln, Font_7x10, White);

  ssd1306_SetCursor(0, 24);
  snprintf(ln,sizeof(ln),"Pressure: %.1f hPa %c", g_press_hpa, g_press_trend_char);
  ssd1306_WriteString(ln, Font_7x10, White);

  ssd1306_SetCursor(0, 36);
  snprintf(ln,sizeof(ln),"Dew point: %.1f C", g_dewpoint_c);
  ssd1306_WriteString(ln, Font_7x10, White);

  ssd1306_UpdateScreen();
}

static void UI_DrawIMU(void)
{
  char ln[32];
  ssd1306_Fill(Black);

  /* 1st line – Roll */
  ssd1306_SetCursor(0, 0);
  snprintf(ln, sizeof(ln), "Roll   %6.1f", g_roll_deg);
  ssd1306_WriteString(ln, Font_7x10, White);
  ssd1306_WriteString(" deg", Font_7x10, White);

  /* 2nd line – Pitch */
  ssd1306_SetCursor(0, 12);
  snprintf(ln, sizeof(ln), "Pitch  %6.1f", g_pitch_deg);
  ssd1306_WriteString(ln, Font_7x10, White);
  ssd1306_WriteString(" deg", Font_7x10, White);

  /* 3rd line – RMS vibrations */
  ssd1306_SetCursor(0, 24);
  snprintf(ln, sizeof(ln), "Vib RMS %.2f g", g_vibr_rms_g);
  ssd1306_WriteString(ln, Font_7x10, White);

  ssd1306_UpdateScreen();
}

static void UI_DrawStats(void)
{
  char ln[32];
  ssd1306_Fill(Black);

  float avg_mov = (g_moving_ms>0)
      ? ( (g_trip_m/1000.0f) / ((float)g_moving_ms/3600000.0f) )
      : 0.0f;

  uint32_t sm = g_moving_ms/1000U;
  uint32_t hh = sm/3600U, mm=(sm/60U)%60U, ss=sm%60U;

  ssd1306_SetCursor(0, 0);
  snprintf(ln,sizeof(ln),"Max Speed %5.1f", g_max_speed_kmh);
  ssd1306_WriteString(ln, Font_7x10, White);
  ssd1306_WriteString(" km/h", Font_7x10, White);

  ssd1306_SetCursor(0, 12);
  snprintf(ln,sizeof(ln),"Avg Speed %5.1f", avg_mov);
  ssd1306_WriteString(ln, Font_7x10, White);
  ssd1306_WriteString(" km/h", Font_7x10, White);

  ssd1306_SetCursor(0, 24);
  snprintf(ln,sizeof(ln),"Avg Power %3.0f W", g_power_avg_w);
  ssd1306_WriteString(ln, Font_7x10, White);

  ssd1306_SetCursor(0, 36);
  snprintf(ln,sizeof(ln),"Energy %5.0f kcal", g_kcal_total);
  ssd1306_WriteString(ln, Font_7x10, White);

  ssd1306_SetCursor(0, 48);
  snprintf(ln,sizeof(ln),"Ride %2lu:%02lu:%02lu",
           (unsigned long)hh,(unsigned long)mm,(unsigned long)ss);
  ssd1306_WriteString(ln, Font_7x10, White);

  ssd1306_UpdateScreen();
}

/* ==================== BUTTON – POLLING ==================== */
/* Raw read: 1 = pressed, 0 = released
 * Switch_Pin has PULLUP, so:
 *   GPIO_PIN_RESET (0) => button pressed
 *   GPIO_PIN_SET   (1) => button released
 */
static uint8_t BTN_IsPressedRaw(void)
{
  GPIO_PinState s = HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN);
  return (s == GPIO_PIN_RESET) ? 1u : 0u;
}

/* Simple state machine with debounce:
 *  - detects only 0 -> 1 transitions (new press)
 *  - sets g_btn_event = 1
 */
static void BTN_PollTask(void)
{
  static uint8_t last_raw = 0;
  uint32_t now = HAL_GetTick();
  uint8_t raw = BTN_IsPressedRaw();

  if (raw != last_raw) {
    g_btn_last_change_ms = now;
    last_raw = raw;
  }

  if ((now - g_btn_last_change_ms) >= BTN_DEBOUNCE_MS) {
    if (raw && !g_btn_pressed) {
      g_btn_pressed = 1;
      g_btn_event   = 1;
    } else if (!raw && g_btn_pressed) {
      g_btn_pressed = 0;
    }
  }
}

/* ==================== HAL CALLBACKS ==================== */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Hall – using hall_Pin from main.h */
  if (GPIO_Pin == hall_Pin) {
    Hall_OnEdgeMs(&g_hall, HAL_GetTick());
    return;
  }

  /* Button EXTI not used – button handled by polling */
}

/* TIM2: every tick -> speed, autopause, power, history windows */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    static uint32_t last_tim2_ms = 0;
    uint32_t now   = HAL_GetTick();
    uint32_t dt_ms = (last_tim2_ms == 0U) ? 0U : (now - last_tim2_ms);
    last_tim2_ms   = now;

    /* Speed & distance from Hall module (period between pulses) */
    g_speed_inst_kmh  = Hall_SpeedInstKmh(&g_hall);
    g_speed_filt_kmh  = Hall_SpeedFiltKmh(&g_hall);
    g_trip_m          = Hall_DistanceM(&g_hall);
    g_hall_count      = Hall_Pulses(&g_hall);

    /* Autopause + moving time + displayed speed */
    if (g_speed_filt_kmh >= SPEED_MIN_MOVING_KMH) {
      g_moving_ms += (dt_ms > 0U) ? dt_ms : 0U;
      g_autopause = 0;
      g_speed_kmh = g_speed_filt_kmh;
    } else {
      g_autopause = 1;
      g_speed_kmh = 0.0f;
    }

    /* Max speed */
    if (g_speed_filt_kmh > g_max_speed_kmh) g_max_speed_kmh = g_speed_filt_kmh;

    /* Power model & energy integration */
    if (dt_ms > 0U) {
      float dt_s   = (float)dt_ms / 1000.0f;
      float v_mps  = g_speed_filt_kmh / 3.6f;
      float grade  = g_grade_pct / 100.0f;

      if (v_mps <= 0.1f) {
        g_power_w = 0.0f;
      } else {
        float P_climb = MASS_TOTAL_KG * 9.81f * v_mps * grade;
        if (P_climb < 0.0f) P_climb = 0.0f;

        float P_roll  = MASS_TOTAL_KG * 9.81f * C_RR * v_mps;
        float P_aero  = 0.5f * RHO_AIR * CD_AREA * v_mps * v_mps * v_mps;
        float P_total = P_climb + P_roll + P_aero;
        if (P_total < 0.0f) P_total = 0.0f;

        g_power_w = P_total;

        if (g_speed_filt_kmh >= SPEED_MIN_MOVING_KMH) {
          float E_mech = P_total * dt_s;           /* J */
          g_mech_energy_J += E_mech;
          float kcal_step = E_mech / (4184.0f * HUMAN_EFF);
          g_kcal_total += kcal_step;
        }
      }

      if (g_moving_ms > 0U) {
        g_power_avg_w = g_mech_energy_J / ((float)g_moving_ms/1000.0f);
      } else {
        g_power_avg_w = 0.0f;
      }
    }

    /* Update GRAD / VAM windows */
    update_alt_grad_vam_from_samples();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
void LowPower_EnterSleep(void)
{
    /* Zatrzymaj SysTick, żeby nie wybudzał co 1 ms */
    HAL_SuspendTick();

    /* Opcjonalnie: skasuj flagi wakeup */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    /* Wejście w SLEEP, regulator ON */
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

    /* Po wybudzeniu */
    HAL_ResumeTick();
}
void LowPower_EnterStop2(void)
{
    /* Wyłącz debug w STOP (jeśli debuger podpięty, potrafi podnosić pobór) */
#ifdef DBGMCU
    HAL_DBGMCU_DisableDBGStopMode();
#endif

    /* Zatrzymaj SysTick */
    HAL_SuspendTick();

    /* Opcjonalne flagi low-power (często dają dodatkowy zysk) */
    //HAL_PWREx_EnableUltraLowPower();
    //HAL_PWREx_EnableFastWakeUp();

    /* Skasuj flagę wybudzenia */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    /* Wejście w STOP2 */
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

    /* ====== po wybudzeniu ======
       W STOP2 zegary systemowe zwykle wracają do MSI/bez PLL.
       Trzeba przywrócić konfigurację zegarów. */
    SystemClock_Config();

    /* Wznów SysTick */
    HAL_ResumeTick();
}
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
