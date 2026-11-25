/* USER CODE BEGIN Header */
/**
 * STM32 NUCLEO-F446RE
 * ICM-20948 9-axis IMU + Stepper + Vibration
 * PC communication via USART2 / ST-LINK VCP
 *
 * Existing wiring preserved:
 * PA6  = Vibration (D12)
 * PA10 = Stepper STEP
 * PB3  = Stepper DIR
 * PB8  = I2C1 SCL
 * PB9  = I2C1 SDA
 * PA2  = USART2 TX
 * PA3  = USART2 RX
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "attitude_filters.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t icm_ok = 0;
static uint8_t mag_ok = 0;
static uint8_t mag_initialized = 0;
static uint8_t mag_last_st1 = 0, mag_last_st2 = 0;
static uint8_t mag_last_i2c_ok = 0;
static uint32_t gyro_zero_samples = 0;
static uint32_t mag_valid_samples = 0;
static uint32_t sample_count_window = 0;
static uint32_t diag_last_tick = 0;

static int16_t accel_x = 0;
static int16_t accel_y = 0;
static int16_t accel_z = 0;

static int16_t gyro_x = 0;
static int16_t gyro_y = 0;
static int16_t gyro_z = 0;

static int16_t mag_x = 0;
static int16_t mag_y = 0;
static int16_t mag_z = 0;

static uint32_t sample_seq = 0;
/* Stage 1: real STM32 Complementary and Mahony filter states. */
static AttitudeFilterState complementary_state;
static AttitudeFilterState mahony_state;
static AttitudeFilterState aqua_state;
static AttitudeFilterState ekf_state;
static AttitudeOutput complementary_out;
static AttitudeOutput mahony_out;
static AttitudeOutput aqua_out;
static AttitudeOutput ekf_out;
static uint32_t last_filter_tick = 0;
#define IMU_SAMPLE_HZ 60UL
static uint32_t filter_cycles[4] = {0};
static uint32_t filter_cycles_avg[4] = {0};
static uint32_t cpu_clock_hz = 84000000UL;
static uint8_t perf_divider = 0;
/* Protractor tracking uses quaternion increments around the physical rotation
 * axis, NOT Euler yaw. The first deliberate CCW motion after ZERO defines +.
 * Auto axis selection uses gyro magnitude, then remains locked until ZERO.
 * AXIS,X,+ / AXIS,Y,- / AXIS,Z,+ can override auto if needed. */
static float angle_home_q[4][4];
static float angle_last_twist_deg[4] = {0};
static float angle_accum_deg[4] = {0};
static uint8_t angle_prev_valid[4] = {0};
static uint8_t angle_zero_requested = 0;
static uint32_t angle_boot_start_ms = 0;
static uint8_t angle_home_ready = 0;
#define ANGLE_HOME_SETTLE_MS 3000UL
static int8_t angle_axis = -1;       /* -1 = auto/unlocked, 0=X,1=Y,2=Z */
static int8_t angle_axis_sign = 0;   /* +1 or -1; 0 = first CCW movement */
static int8_t angle_forced_axis = -1;
static int8_t angle_forced_sign = 0;
static uint8_t angle_auto_votes[3] = {0};
static uint8_t angle_axis_announced = 0;


static char rx_buffer[64];
static uint8_t rx_index = 0;
static uint8_t rx_discard = 0;
#define UART_RX_RING_SIZE 256U
static uint8_t uart_rx_byte;
static volatile uint8_t uart_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t uart_rx_head = 0, uart_rx_tail = 0;
static volatile uint8_t uart_rx_fault = 0;
static volatile uint8_t uart_rx_restart = 0;

static uint32_t motor_steps_left = 0;
static uint32_t motor_last_tick = 0;
static uint8_t motor_step_high = 0;
static uint8_t motor_dir_pending = 0;
static uint32_t motor_dir_tick = 0;

/*
 * 200 = full-step 1.8-degree motor.
 * Change if driver microstepping is enabled.
 */
#define MOTOR_STEPS_PER_REV 200UL
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef ICM_Bank(uint8_t bank);
static HAL_StatusTypeDef ICM_Write(uint8_t reg, uint8_t val);
static HAL_StatusTypeDef ICM_ReadReg(uint8_t reg, uint8_t *val);

static uint8_t ICM_Init(void);
static uint8_t MAG_Init(void);
static uint8_t ICM_ReadAll(void);

static void UART_Send(const char *str);
static void UART_ReceivePoll(void);
static void UART_ProcessCommand(char *cmd);
static void UART_SendSensor(void);
static void UART_SendDiagnostics(void);
static void Filters_UpdateAndSend(void);

static void Motor_Start(int32_t degrees);
static void Motor_Stop(void);
static void Motor_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define ICM_ADDR       (0x68 << 1)
#define MAG_ADDR       0x0C

#define ICM_BANK_SEL   0x7F
#define ICM_WHO_AM_I   0x00
#define ICM_PWR_MGMT_1 0x06
#define ICM_ACCEL_XOUT 0x2D
#define ICM_GYRO_XOUT  0x33
#define ICM_PWR_MGMT_2 0x07
#define ICM_GYRO_CONFIG_1 0x01

#define ICM_USER_CTRL  0x03
#define ICM_EXT_SENS   0x3B

#define MAG_ST1        0x10
#define MAG_CNTL2      0x31

#define VIB_PORT       GPIOA
#define VIB_PIN        GPIO_PIN_6

#define STEP_PORT      GPIOA
#define STEP_PIN       GPIO_PIN_10

#define DIR_PORT       GPIOB
#define DIR_PIN        GPIO_PIN_3

/* ---------------- UART ---------------- */

static void UART_Send(const char *str)
{
    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)str,
        (uint16_t)strlen(str),
        100
    );
}

/* ---------------- ICM register access ---------------- */

static HAL_StatusTypeDef ICM_Bank(uint8_t bank)
{
    uint8_t val = (uint8_t)(bank << 4);

    return HAL_I2C_Mem_Write(
        &hi2c1,
        ICM_ADDR,
        ICM_BANK_SEL,
        I2C_MEMADD_SIZE_8BIT,
        &val,
        1,
        100
    );
}

static HAL_StatusTypeDef ICM_Write(
    uint8_t reg,
    uint8_t val
)
{
    return HAL_I2C_Mem_Write(
        &hi2c1,
        ICM_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &val,
        1,
        100
    );
}

static HAL_StatusTypeDef ICM_ReadReg(
    uint8_t reg,
    uint8_t *val
)
{
    return HAL_I2C_Mem_Read(
        &hi2c1,
        ICM_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        val,
        1,
        100
    );
}

/* ---------------- ICM initialization ---------------- */

static uint8_t ICM_Init(void)
{
    uint8_t id = 0, check = 0;
    if (ICM_Bank(0) != HAL_OK || ICM_ReadReg(ICM_WHO_AM_I, &id) != HAL_OK || id != 0xEA)
        return 0;
    if (ICM_Write(ICM_PWR_MGMT_1, 0x80) != HAL_OK) return 0;
    HAL_Delay(120);
    if (ICM_Bank(0) != HAL_OK) return 0;
    if (ICM_Write(ICM_PWR_MGMT_1, 0x01) != HAL_OK) return 0;
    HAL_Delay(20);
    /* Explicitly enable all three accelerometer and gyroscope axes. */
    if (ICM_Write(ICM_PWR_MGMT_2, 0x00) != HAL_OK) return 0;
    HAL_Delay(100);
    if (ICM_ReadReg(ICM_PWR_MGMT_2, &check) != HAL_OK || check != 0x00) return 0;
    if (ICM_Bank(2) != HAL_OK) return 0;
    /* FS_SEL=0: accel +/-2g (16384 LSB/g), gyro +/-250 dps (131 LSB/dps). */
    if (ICM_Write(0x14, 0x00) != HAL_OK) return 0;
    if (ICM_Write(ICM_GYRO_CONFIG_1, 0x00) != HAL_OK) return 0;
    if (ICM_ReadReg(ICM_GYRO_CONFIG_1, &check) != HAL_OK || check != 0x00) return 0;
    if (ICM_Bank(0) != HAL_OK) return 0;
    HAL_Delay(100);
    return 1;
}

/* ---------------- AK09916 magnetometer ---------------- */

/*
 * ICM-20948 accesses AK09916 through
 * its internal I2C master.
 *
 * SLV0 is first configured for writing CNTL2,
 * then reconfigured for continuous reading.
 */

static uint8_t MAG_Init(void)
{
    uint8_t data[9] = {0};

    /* Bank 0: enable internal I2C master */
    if (ICM_Bank(0) != HAL_OK)
        return 0;

    if (ICM_Write(ICM_USER_CTRL, 0x20) != HAL_OK)
        return 0;

    HAL_Delay(10);

    /* Bank 3: internal I2C master configuration */
    if (ICM_Bank(3) != HAL_OK)
        return 0;

    /*
     * Internal I2C master ODR:
     * 1125 / 2^4 = approximately 70.3 Hz
     */
    if (ICM_Write(0x00, 0x04) != HAL_OK)
        return 0;

    /* Internal I2C master clock */
    if (ICM_Write(0x01, 0x07) != HAL_OK)
        return 0;

    /* Disable SLV0 before changing configuration */
    if (ICM_Write(0x05, 0x00) != HAL_OK)
        return 0;

    /* Write AK09916 CNTL2 = 0x08 (100 Hz) */
    if (ICM_Write(0x03, MAG_ADDR) != HAL_OK)
        return 0;

    if (ICM_Write(0x04, MAG_CNTL2) != HAL_OK)
        return 0;

    if (ICM_Write(0x06, 0x08) != HAL_OK)
        return 0;

    if (ICM_Write(0x05, 0x81) != HAL_OK)
        return 0;

    HAL_Delay(50);

    /* Disable SLV0 before switching to read */
    if (ICM_Write(0x05, 0x00) != HAL_OK)
        return 0;

    /* Read AK09916 from ST1 through ST2 */
    if (ICM_Write(0x03, 0x80 | MAG_ADDR) != HAL_OK)
        return 0;

    if (ICM_Write(0x04, MAG_ST1) != HAL_OK)
        return 0;

    if (ICM_Write(0x05, 0x89) != HAL_OK)
        return 0;

    /* Apply ODR control to SLV0 */
    if (ICM_Write(0x02, 0x01) != HAL_OK)
        return 0;

    HAL_Delay(100);

    if (ICM_Bank(0) != HAL_OK)
        return 0;

    /* Confirm that the internal I2C master is not reporting NACK */
    uint8_t status = 0;

    if (ICM_ReadReg(0x17, &status) != HAL_OK)
        return 0;

    if (status & 0x01)
        return 0;

    if (HAL_I2C_Mem_Read(
        &hi2c1,
        ICM_ADDR,
        ICM_EXT_SENS,
        I2C_MEMADD_SIZE_8BIT,
        data,
        9,
        100
    ) != HAL_OK)
    {
        return 0;
    }

    return 1;
}


/* ---------------- Read all sensors ---------------- */

static uint8_t ICM_ReadAll(void)
{
    uint8_t data[12];
    uint8_t m[9];

    if (ICM_Bank(0) != HAL_OK)
        return 0;

    if (HAL_I2C_Mem_Read(
        &hi2c1,
        ICM_ADDR,
        ICM_ACCEL_XOUT,
        I2C_MEMADD_SIZE_8BIT,
        data,
        12,
        100
    ) != HAL_OK)
    {
        return 0;
    }

    /* V1.4: preliminary accelerometer zero-g offset correction.
     * Derived from opposing stationary orientations in the user's logs:
     * corrected X at 0/180 deg ~= -16000/+16600 (midpoint +300);
     * corrected Z at 90/270 deg ~= -19000/+14000 (midpoint -2500).
     * The prior software corrections were +308, +1596, -6596 counts.
     * Subtract the observed residual midpoints from X and Z only.
     * These are preliminary sensor calibration values, NOT angle fitting.
     * Recalibrate with a controlled six-face test before final experiments.
     */
    accel_x = (int16_t)((data[0] << 8) | data[1]);
    accel_y = (int16_t)((data[2] << 8) | data[3]);
    accel_z = (int16_t)((data[4] << 8) | data[5]);

    accel_x = (int16_t)(accel_x + 8);
    accel_y = (int16_t)(accel_y + 1596);
    accel_z = (int16_t)(accel_z - 2846); /* Hybrid: provisional Z offset correction */

    /* Gyroscope */
    gyro_x = (int16_t)((data[6] << 8) | data[7]);
    gyro_y = (int16_t)((data[8] << 8) | data[9]);
    gyro_z = (int16_t)((data[10] << 8) | data[11]);
    /* Independent gyro register read: avoids silently trusting the burst tail. */
    uint8_t g[6];
    if (HAL_I2C_Mem_Read(&hi2c1, ICM_ADDR, ICM_GYRO_XOUT,
                         I2C_MEMADD_SIZE_8BIT, g, sizeof(g), 20) != HAL_OK)
        return 0;
    gyro_x = (int16_t)((g[0] << 8) | g[1]);
    gyro_y = (int16_t)((g[2] << 8) | g[3]);
    gyro_z = (int16_t)((g[4] << 8) | g[5]);

    /* Magnetometer: report fresh samples only; never pass stale data as valid. */
    mag_ok = 0;
    mag_last_i2c_ok = 0;

    if (HAL_I2C_Mem_Read(
        &hi2c1,
        ICM_ADDR,
        ICM_EXT_SENS,
        I2C_MEMADD_SIZE_8BIT,
        m,
        9,
        100
    ) == HAL_OK)
    {
        mag_last_i2c_ok = 1;
        uint8_t status1 = m[0];
        uint8_t status2 = m[8];
        mag_last_st1 = status1;
        mag_last_st2 = status2;

        /* Check data ready and magnetic overflow */
        if ((status1 & 0x01) &&
            !(status2 & 0x08))
        {
            mag_x = (int16_t)((m[2] << 8) | m[1]);
            mag_y = (int16_t)((m[4] << 8) | m[3]);
            mag_z = (int16_t)((m[6] << 8) | m[5]);

            mag_ok = 1;
        }
    }
    if (!mag_ok) mag_x = mag_y = mag_z = 0;
    if (gyro_x == 0 && gyro_y == 0 && gyro_z == 0) gyro_zero_samples++;
    if (mag_ok) mag_valid_samples++;
    sample_count_window++;
    return 1;
}

/* ---------------- Send sensor packet ---------------- */

/*
 * Packet:
 * DATA,seq,time_ms,ax,ay,az,gx,gy,gz,mx,my,mz,mag_valid
 *
 * All 9 sensor values are raw integer readings,
 * except the existing accelerometer offsets.
 *
 * Accelerometer scale: 16384 LSB/g
 * Gyroscope scale:     131 LSB/(deg/s)
 * Magnetometer scale:  0.15 uT/LSB
 *
 * Convert to physical units in Python.
 */

static void UART_SendSensor(void)
{
    char line[180];

    int n = snprintf(
        line,
        sizeof(line),
        "DATA,%lu,%lu,"
        "%d,%d,%d,"
        "%d,%d,%d,"
        "%d,%d,%d,%u\r\n",
        (unsigned long)sample_seq++,
        (unsigned long)HAL_GetTick(),
        (int)accel_x,
        (int)accel_y,
        (int)accel_z,
        (int)gyro_x,
        (int)gyro_y,
        (int)gyro_z,
        (int)mag_x,
        (int)mag_y,
        (int)mag_z,
        (unsigned int)mag_ok
    );

    if (n > 0 && n < (int)sizeof(line))
    {
        HAL_UART_Transmit(
            &huart2,
            (uint8_t *)line,
            (uint16_t)n,
            30
        );
    }
}

/* Low-rate diagnostic packet, independent of existing DATA/FILT/PERF format.
   DIAG,imu_pwr2,gyro_cfg,gyro_x,gyro_y,gyro_z,mag_init,mag_i2c,st1,st2,gyro_zero_count,mag_valid_count,total_count */
static void UART_SendDiagnostics(void)
{
    uint8_t pwr2 = 255, gyro_cfg = 255;
    uint8_t ok = (ICM_Bank(0) == HAL_OK &&
                  ICM_ReadReg(ICM_PWR_MGMT_2, &pwr2) == HAL_OK);
    if (ok && ICM_Bank(2) == HAL_OK)
        (void)ICM_ReadReg(ICM_GYRO_CONFIG_1, &gyro_cfg);
    (void)ICM_Bank(0);
    char line[160];
    int n = snprintf(line, sizeof(line),
        "DIAG,%u,%u,%d,%d,%d,%u,%u,%u,%u,%lu,%lu,%lu\r\n",
        (unsigned)pwr2, (unsigned)gyro_cfg,
        (int)gyro_x, (int)gyro_y, (int)gyro_z,
        (unsigned)mag_initialized, (unsigned)mag_last_i2c_ok,
        (unsigned)mag_last_st1, (unsigned)mag_last_st2,
        (unsigned long)gyro_zero_samples,
        (unsigned long)mag_valid_samples,
        (unsigned long)sample_count_window);
    if (n > 0 && n < (int)sizeof(line))
        HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, 30);
    gyro_zero_samples = mag_valid_samples = sample_count_window = 0;
}

/* ---------------- Stage 1 STM32 attitude filters ---------------- */
/* Original DATA packet is preserved for the existing Python receiver. */
/* FILT packet is an additional, separate telemetry message. */
static float Angle_Wrap360(float x)
{
    x = fmodf(x, 360.0f);
    if (x < 0.0f) x += 360.0f;
    return x;
}

static void Angle_CopyQ(const AttitudeOutput *out, float q[4])
{
    q[0] = out->q0; q[1] = out->q1;
    q[2] = out->q2; q[3] = out->q3;
}

static void Angle_Reset(void)
{
    for (int i=0; i<4; i++) {
        angle_prev_valid[i] = 0;
        angle_accum_deg[i] = 0.0f;
        angle_last_twist_deg[i] = 0.0f;
    }
    for (int i=0; i<3; i++) angle_auto_votes[i] = 0;
    angle_axis = angle_forced_axis;
    angle_axis_sign = angle_forced_sign;
    angle_axis_announced = 0;
}

/* Determine the physical rotation axis from the gyro, not Euler angles.
 * Require a dominant axis on 3 samples, > 12 deg/s; ignore rest noise. */
static void Angle_DetectAxis(float gx,float gy,float gz)
{
    if (angle_axis >= 0) return;
    float g[3] = {fabsf(gx),fabsf(gy),fabsf(gz)};
    int k = 0;
    if (g[1] > g[k]) k = 1;
    if (g[2] > g[k]) k = 2;
    float other = g[(k+1)%3] > g[(k+2)%3] ? g[(k+1)%3] : g[(k+2)%3];
    if (g[k] < 0.20944f || g[k] < 1.6f*other) return;
    for (int j=0;j<3;j++) if (j!=k) angle_auto_votes[j]=0;
    if (++angle_auto_votes[k] >= 3) {
        angle_axis = (int8_t)k;
        angle_axis_sign = 0;
    }
}

/* V1.1 ANGLE-only experiment: measure orientation relative to HOME, rather
 * than summing projected frame-to-frame quaternion increments. A signed
 * quaternion twist about the physical sensor axis gives the protractor angle.
 * Unwrap successive HOME-relative twists to preserve multiple revolutions.
 * FILT quaternion values and all four filter algorithms remain untouched. */
static void Angle_Update(int i, const AttitudeOutput *out)
{
    float q[4]; Angle_CopyQ(out,q);
    float norm = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (!isfinite(norm) || norm < 1e-6f) return;
    for (int k=0;k<4;k++) q[k] /= norm;
    if (!angle_prev_valid[i]) {
        memcpy(angle_home_q[i], q, sizeof(q));
        angle_prev_valid[i] = 1;
        return;
    }
    if (angle_axis < 0 || angle_axis_sign == 0) return;
    const float *h = angle_home_q[i];
    /* conjugate(home) * current: relative rotation in HOME body axes. */
    float w = h[0]*q[0]+h[1]*q[1]+h[2]*q[2]+h[3]*q[3];
    float v[3] = {
        h[0]*q[1]-h[1]*q[0]-h[2]*q[3]+h[3]*q[2],
        h[0]*q[2]+h[1]*q[3]-h[2]*q[0]-h[3]*q[1],
        h[0]*q[3]-h[1]*q[2]+h[2]*q[1]-h[3]*q[0]
    };
    /* Twist is invariant to quaternion sign (q and -q). */
    float twist = 2.0f*atan2f(v[angle_axis], w)*57.29577951308232f;
    twist *= (float)angle_axis_sign;
    /* Difference is the shortest continuous step, even across +/-180/360. */
    float delta = twist - angle_last_twist_deg[i];
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    angle_accum_deg[i] += delta;
    angle_last_twist_deg[i] = twist;
}

static void Filters_UpdateAndSend(void)
{
    const float gyro_scale = 0.017453292519943295f / 131.0f;
    const float accel_scale = 1.0f / 16384.0f;
    uint32_t tick = HAL_GetTick();
    float dt = (last_filter_tick == 0) ? (1.0f / (float)IMU_SAMPLE_HZ) :
               (float)(tick - last_filter_tick) * 0.001f;
    last_filter_tick = tick;
    if (dt <= 0.0f || dt > 0.2f) dt = 1.0f / (float)IMU_SAMPLE_HZ;

    float ax = (float)accel_x * accel_scale;
    float ay = (float)accel_y * accel_scale;
    float az = (float)accel_z * accel_scale;
    float gx = (float)gyro_x * gyro_scale;
    float gy = (float)gyro_y * gyro_scale;
    float gz = (float)gyro_z * gyro_scale;
    /* Full 9-axis fusion. Preserve last valid magnetic reading only when mag_ok. */
    float mx = mag_ok ? (float)mag_x * 0.15f : 0.0f;
    float my = mag_ok ? (float)mag_y * 0.15f : 0.0f;
    float mz = mag_ok ? (float)mag_z * 0.15f : 0.0f;
    uint32_t cy0,cy1;
    cy0 = DWT->CYCCNT;
    Complementary_Update(&complementary_state, ax, ay, az,
                         gx, gy, gz, mx, my, mz, dt, &complementary_out);
    cy1 = DWT->CYCCNT; filter_cycles[0] = cy1-cy0;
    cy0 = DWT->CYCCNT;
    Mahony_Update(&mahony_state, ax, ay, az,
                  gx, gy, gz, mx, my, mz, dt, &mahony_out);
    cy1 = DWT->CYCCNT; filter_cycles[1] = cy1-cy0;
    cy0 = DWT->CYCCNT;
    AQUA_Update(&aqua_state, ax, ay, az, gx, gy, gz, mx, my, mz, dt, &aqua_out);
    cy1 = DWT->CYCCNT; filter_cycles[2] = cy1-cy0;
    cy0 = DWT->CYCCNT;
    EKF_Update(&ekf_state, ax, ay, az, gx, gy, gz, mx, my, mz, dt, &ekf_out);
    cy1 = DWT->CYCCNT; filter_cycles[3] = cy1-cy0;
    for (int k=0;k<4;k++) {
        filter_cycles_avg[k] = filter_cycles_avg[k] ?
            (filter_cycles_avg[k]*7UL + filter_cycles[k])/8UL : filter_cycles[k];
    }

    /* No PuTTY command required. Power on with protractor at 0 degrees.
     * Let the four attitude filters settle for 3 seconds, then capture HOME.
     * Do not track rotation before the HOME reference is established. */
    if (angle_boot_start_ms == 0) angle_boot_start_ms = tick ? tick : 1;
    if (!angle_home_ready && (uint32_t)(tick - angle_boot_start_ms) >= ANGLE_HOME_SETTLE_MS) {
        Angle_Reset();
        angle_home_ready = 1;
        UART_Send("STATUS,ANGLE_HOME_AUTO_ZERO\r\n");
        UART_Send("STATUS,ACCEL_CAL_V14,X8,Y1596,Z-4096\r\n");
    }
    if (angle_zero_requested) {
        Angle_Reset();
        angle_zero_requested = 0;
        angle_home_ready = 1;
    }
    if (angle_home_ready) Angle_DetectAxis(gx,gy,gz);
    if (angle_home_ready && angle_axis >= 0 && angle_axis_sign == 0) {
        float g[3] = {gx,gy,gz};
        if (fabsf(g[angle_axis]) > 0.20944f) {
            /* User starts with CCW motion after ZERO: make that positive. */
            angle_axis_sign = g[angle_axis] > 0.0f ? 1 : -1;
        }
    }
    if (angle_home_ready) {
        Angle_Update(0, &complementary_out);
        Angle_Update(1, &mahony_out);
        Angle_Update(2, &aqua_out);
        Angle_Update(3, &ekf_out);
    }
    if (angle_axis >= 0 && angle_axis_sign != 0 && !angle_axis_announced) {
        char msg[48];
        snprintf(msg,sizeof(msg),"STATUS,ANGLE_AXIS,%c,%c\r\n",
                 "XYZ"[angle_axis],angle_axis_sign>0?'+':'-');
        UART_Send(msg);
        angle_axis_announced = 1;
    }

    /* 10 Hz telemetry to limit UART load. Float formatting may require
       enabling -u _printf_float in STM32CubeIDE linker options. */
    static uint8_t telemetry_divider = 0;
    if (++telemetry_divider >= 6)
    {
        telemetry_divider = 0;
        char line[192];
        int n = snprintf(line, sizeof(line),
                         "FILT,C,%.2f,%.2f,%.2f,%.5f,%.5f,%.5f,%.5f\r\n",
                         (double)complementary_out.roll_deg,
                         (double)complementary_out.pitch_deg,
                         (double)complementary_out.yaw_deg,
                         (double)complementary_out.q0,
                         (double)complementary_out.q1,
                         (double)complementary_out.q2,
                         (double)complementary_out.q3);
        if (n > 0 && n < (int)sizeof(line))
            HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, 30);
        n = snprintf(line, sizeof(line),
                     "FILT,M,%.2f,%.2f,%.2f,%.5f,%.5f,%.5f,%.5f\r\n",
                     (double)mahony_out.roll_deg,
                     (double)mahony_out.pitch_deg,
                     (double)mahony_out.yaw_deg,
                     (double)mahony_out.q0,
                     (double)mahony_out.q1,
                     (double)mahony_out.q2,
                     (double)mahony_out.q3);
        if (n > 0 && n < (int)sizeof(line))
            HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, 30);
        n = snprintf(line, sizeof(line),
                     "FILT,A,%.2f,%.2f,%.2f,%.5f,%.5f,%.5f,%.5f\r\n",
                     (double)aqua_out.roll_deg, (double)aqua_out.pitch_deg,
                     (double)aqua_out.yaw_deg, (double)aqua_out.q0,
                     (double)aqua_out.q1, (double)aqua_out.q2,
                     (double)aqua_out.q3);
        if (n > 0 && n < (int)sizeof(line))
            HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, 30);
        n = snprintf(line, sizeof(line),
                     "FILT,E,%.2f,%.2f,%.2f,%.5f,%.5f,%.5f,%.5f\r\n",
                     (double)ekf_out.roll_deg, (double)ekf_out.pitch_deg,
                     (double)ekf_out.yaw_deg, (double)ekf_out.q0,
                     (double)ekf_out.q1, (double)ekf_out.q2,
                     (double)ekf_out.q3);
        if (n > 0 && n < (int)sizeof(line))
            HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, 30);
    }
    /* 5 Hz independent protractor telemetry:
       ANGLE,id,modulo_0_to_360,unwrapped_signed_degrees
       Existing FILT packets and quaternions are unchanged. */
    static uint8_t angle_divider = 0;
    if (++angle_divider >= 12)
    {
        angle_divider = 0;
        const char ids[4] = {'C','M','A','E'};
        for (int i = 0; i < 4; i++)
        {
            if (!angle_home_ready || !angle_prev_valid[i]) continue;
            char line[96];
            int n = snprintf(line, sizeof(line), "ANGLE,%c,%.2f,%.2f\r\n",
                             ids[i],
                             (double)Angle_Wrap360(angle_accum_deg[i]),
                             (double)angle_accum_deg[i]);
            if (n > 0 && n < (int)sizeof(line))
                HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, 30);
        }
    }
    if (++perf_divider >= 30) {
        perf_divider = 0;
        const char ids[4] = {'C','M','A','E'};
        for (int i=0;i<4;i++) {
            float us = (float)filter_cycles_avg[i] * 1000000.0f / (float)cpu_clock_hz;
            float cpu = us * (float)IMU_SAMPLE_HZ / 10000.0f; /* 60Hz, percent of one MCU core */
            char line[100];
            int n = snprintf(line,sizeof(line),"PERF,%c,%.3f,%.3f,%lu\r\n",
                ids[i],(double)us,(double)cpu,(unsigned long)sizeof(AttitudeFilterState));
            if(n>0 && n<(int)sizeof(line))
                HAL_UART_Transmit(&huart2,(uint8_t*)line,(uint16_t)n,30);
        }
    }
}

/* ---------------- Stepper motor ---------------- */

static void Motor_Stop(void)
{
    motor_steps_left = 0;
    motor_step_high = 0;
    motor_dir_pending = 0;

    HAL_GPIO_WritePin(
        STEP_PORT,
        STEP_PIN,
        GPIO_PIN_RESET
    );
}

static void Motor_Start(int32_t degrees)
{
    uint32_t abs_degrees;

    if (degrees < -360 || degrees > 360)
        return;

    Motor_Stop();

    HAL_GPIO_WritePin(
        DIR_PORT,
        DIR_PIN,
        degrees >= 0 ?
            GPIO_PIN_SET : GPIO_PIN_RESET
    );

    abs_degrees = (uint32_t)(
        degrees < 0 ? -degrees : degrees
    );

    motor_steps_left =
        (abs_degrees * MOTOR_STEPS_PER_REV + 180UL)
        / 360UL;

    motor_dir_pending = 1;
    motor_dir_tick = HAL_GetTick();
}

static void Motor_Update(void)
{
    uint32_t now = HAL_GetTick();

    if (motor_steps_left == 0)
        return;

    /* Wait 5ms after changing direction */
    if (motor_dir_pending)
    {
        if ((uint32_t)(now - motor_dir_tick) < 5)
            return;

        motor_dir_pending = 0;
        motor_last_tick = now;
    }

    if ((uint32_t)(now - motor_last_tick) < 1)
        return;

    motor_last_tick = now;

    if (!motor_step_high)
    {
        HAL_GPIO_WritePin(
            STEP_PORT,
            STEP_PIN,
            GPIO_PIN_SET
        );

        motor_step_high = 1;
    }
    else
    {
        HAL_GPIO_WritePin(
            STEP_PORT,
            STEP_PIN,
            GPIO_PIN_RESET
        );

        motor_step_high = 0;
        motor_steps_left--;
    }
}

/* ---------------- Python command processing ---------------- */

/*
 * Commands:
 *
 * VIBRATE,ON
 * VIBRATE,OFF
 * ROTATE,90
 * ROTATE,-90
 * STOP
 * PING
 * ZERO  (set current physical orientation as protractor 0 degrees)
 * AXIS,AUTO or AXIS,Y,+ (optional fixed physical rotation axis)
 */

static void UART_ProcessCommand(char *cmd)
{
    if (strncmp(cmd, "AXIS,", 5) == 0)
    {
        /* AXIS,AUTO or AXIS,X,+ / AXIS,Y,- / AXIS,Z,+ */
        if (strcmp(cmd,"AXIS,AUTO")==0) {
            angle_forced_axis=-1; angle_forced_sign=0;
        } else if (strlen(cmd)==8 && cmd[6]==',' &&
                   (cmd[7]=='+' || cmd[7]=='-') &&
                   (cmd[5]=='X' || cmd[5]=='Y' || cmd[5]=='Z')) {
            angle_forced_axis = (cmd[5]=='X')?0:(cmd[5]=='Y'?1:2);
            angle_forced_sign = (cmd[7]=='+')?1:-1;
        } else {
            UART_Send("ERR,AXIS\r\n"); return;
        }
        angle_zero_requested=1;
        UART_Send("ACK,AXIS\r\n");
    }
    else if (strcmp(cmd, "ZERO") == 0)
    {
        /* Reset at next completed sensor/filter sample, not on stale output. */
        angle_zero_requested = 1;
        UART_Send("ACK,ZERO\r\n");
    }
    else if (strcmp(cmd, "VIBRATE,ON") == 0)
    {
        HAL_GPIO_WritePin(
            VIB_PORT,
            VIB_PIN,
            GPIO_PIN_SET
        );

        UART_Send("ACK,VIBRATE,ON\r\n");
    }
    else if (strcmp(cmd, "VIBRATE,OFF") == 0)
    {
        HAL_GPIO_WritePin(
            VIB_PORT,
            VIB_PIN,
            GPIO_PIN_RESET
        );

        UART_Send("ACK,VIBRATE,OFF\r\n");
    }
    else if (strcmp(cmd, "STOP") == 0)
    {
        Motor_Stop();

        HAL_GPIO_WritePin(
            VIB_PORT,
            VIB_PIN,
            GPIO_PIN_RESET
        );

        UART_Send("ACK,STOP\r\n");
    }
    else if (strncmp(cmd, "ROTATE,", 7) == 0)
    {
        char *endptr;
        long degrees = strtol(cmd + 7, &endptr, 10);

        if (endptr == cmd + 7 ||
            *endptr != '\0' ||
            degrees < -360 ||
            degrees > 360)
        {
            UART_Send("ERR,ROTATE\r\n");
            return;
        }

        Motor_Start((int32_t)degrees);
        UART_Send("ACK,ROTATE\r\n");
    }
    else if (strcmp(cmd, "PING") == 0)
    {
        UART_Send("PONG\r\n");
    }
    else
    {
        UART_Send("ERR,UNKNOWN_COMMAND\r\n");
    }
}

/* ---------------- UART interrupt reception ---------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;
    uint16_t next = (uart_rx_head + 1U) % UART_RX_RING_SIZE;
    if (next != uart_rx_tail) {
        uart_rx_ring[uart_rx_head] = uart_rx_byte;
        uart_rx_head = next;
    } else {
        uart_rx_fault = 1;
    }
    if (HAL_UART_Receive_IT(huart, &uart_rx_byte, 1) != HAL_OK)
        uart_rx_restart = 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;
    uart_rx_fault = 1;
    uart_rx_restart = 1;
}

/* Drain the interrupt buffer and execute commands in main context. */
static void UART_ReceivePoll(void)
{
    if (uart_rx_restart || uart_rx_fault) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uart_rx_tail = uart_rx_head;
        rx_index = 0;
        rx_discard = 1;
        uart_rx_fault = 0;
        if (uart_rx_restart) {
            HAL_UART_AbortReceive(&huart2);
            __HAL_UART_CLEAR_OREFLAG(&huart2);
            uart_rx_restart = (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1) != HAL_OK);
        }
        __set_PRIMASK(primask);
    }
    for (unsigned i = 0; i < UART_RX_RING_SIZE; ++i) {
        if (uart_rx_fault || uart_rx_restart || uart_rx_tail == uart_rx_head) break;
        uint8_t ch = uart_rx_ring[uart_rx_tail];
        uart_rx_tail = (uart_rx_tail + 1U) % UART_RX_RING_SIZE;
        if (ch == '\r' || ch == '\n')
        {
            if (rx_discard)
            {
                rx_discard = 0;
                rx_index = 0;
                continue;
            }

            if (rx_index > 0)
            {
                rx_buffer[rx_index] = '\0';

                UART_ProcessCommand(rx_buffer);

                rx_index = 0;
            }
        }
        else if (!rx_discard)
        {
            if (rx_index < sizeof(rx_buffer) - 1)
            {
                rx_buffer[rx_index++] = (char)ch;
            }
            else
            {
                rx_index = 0;
                rx_discard = 1;
            }
        }
        }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1) != HAL_OK)
        uart_rx_restart = 1;

    /* Keep motors off at startup */
    HAL_GPIO_WritePin(
        VIB_PORT,
        VIB_PIN,
        GPIO_PIN_RESET
    );

    Motor_Stop();
    AttitudeFilters_Reset(&complementary_state);
    AttitudeFilters_Reset(&mahony_state);
    AttitudeFilters_Reset(&aqua_state);
    AttitudeFilters_Reset(&ekf_state);
    /* Pereira's advice: tune model/measurement covariance ratio, not guess exact Q.
       Q=1e-5, R=0.025 are starting values, not experimentally validated. */
    EKF_SetNoise(1e-5f, 0.025f);
    cpu_clock_hz = HAL_RCC_GetHCLKFreq();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    HAL_Delay(200);

    icm_ok = ICM_Init();

    if (icm_ok)
    {
        mag_initialized = MAG_Init();
        mag_ok = 0;
        UART_Send("STATUS,ICM_READY\r\n");
        UART_Send(mag_initialized ? "STATUS,MAG_READY\r\n" : "STATUS,MAG_INIT_ERROR\r\n");
    }
    else
    {
        UART_Send("STATUS,ICM_ERROR\r\n");
    }

    uint32_t last_sample = HAL_GetTick();
    uint32_t sample_fraction = 0;
    uint32_t sample_interval_ms = 1000UL / IMU_SAMPLE_HZ;
    uint32_t last_retry = HAL_GetTick();

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        UART_ReceivePoll();

        Motor_Update();
        if ((uint32_t)(now - diag_last_tick) >= 2000UL && icm_ok)
        {
            diag_last_tick = now;
            UART_SendDiagnostics();
        }

        /* 60Hz average sampling: 16/17ms intervals via fractional scheduler. */
        if ((uint32_t)(now - last_sample) >= sample_interval_ms)
        {
            last_sample += sample_interval_ms;
            sample_fraction += 1000UL % IMU_SAMPLE_HZ;
            sample_interval_ms = 1000UL / IMU_SAMPLE_HZ;
            if (sample_fraction >= IMU_SAMPLE_HZ)
            {
                sample_fraction -= IMU_SAMPLE_HZ;
                sample_interval_ms++;
            }
            /* If work stalls, avoid an unbounded catch-up burst. */
            if ((uint32_t)(now - last_sample) > 100UL)
                last_sample = now;

            if (icm_ok)
            {
                if (ICM_ReadAll())
                {
                    UART_SendSensor();
                    Filters_UpdateAndSend();
                }
                else
                {
                    icm_ok = 0;
                    UART_Send("STATUS,ICM_ERROR\r\n");
                }
            }
        }

        /* Retry IMU initialization on failure */
        if (!icm_ok &&
            (uint32_t)(now - last_retry) >= 1000)
        {
            last_retry = now;

            icm_ok = ICM_Init();

            if (icm_ok)
            {
                mag_initialized = MAG_Init();
                mag_ok = 0;
                UART_Send("STATUS,ICM_READY\r\n");
                UART_Send(mag_initialized ? "STATUS,MAG_READY\r\n" : "STATUS,MAG_INIT_ERROR\r\n");
            }
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

    __disable_irq();

    while (1)
    {
    }

  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    (void)file;
    (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
