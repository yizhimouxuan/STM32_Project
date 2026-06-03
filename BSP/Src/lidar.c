#include "lidar.h"
#include "../../Core/Inc/usart.h"
#include "main.h"

#include <math.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

#define LIDAR_CMD_SYNC_BYTE         0xA5U
#define LIDAR_CMDFLAG_HAS_PAYLOAD   0x80U

#define LIDAR_CMD_SCAN              0x20U
#define LIDAR_CMD_FORCE_SCAN        0x21U
#define LIDAR_CMD_STOP              0x25U
#define LIDAR_CMD_GET_INFO          0x50U
#define LIDAR_CMD_GET_HEALTH        0x52U
#define LIDAR_CMD_HQ_MOTOR_SPEED    0xA8U
#define LIDAR_CMD_SET_MOTOR_PWM     0xF0U

#define LIDAR_DEFAULT_PWM           850U
#define LIDAR_STARTUP_PWM           1023U
#define LIDAR_STARTUP_BOOST_MS      2600U
#define LIDAR_FORWARD_INTERVAL_MS   100U
#define LIDAR_FORWARD_RAW_ENABLE    0U
#define LIDAR_FORWARD_RAW_INTERVAL_MS 300U
#define LIDAR_FORWARD_RAW_MAX_BYTES 64U

#define LIDAR_FIX_BAUD              460800U
#define LIDAR_RECOVERY_RETRY_MS     2600U
#define LIDAR_STREAM_LOST_MS        30000U
#define LIDAR_RESTART_GAP_MS        30000U
#define LIDAR_AUTO_RESTART_SCAN_EN  0U
#define LIDAR_BOOTSTRAP_KICK_MS     1800U
#define LIDAR_BOOTSTRAP_MAX_KICKS   3U
#define LIDAR_STARTUP_MULTI_MODE_ENABLE 0U
#define LIDAR_FRONT_CENTER_DEG      0.0f
#define LIDAR_FRONT_WINDOW_DEG      10.0f
/* Sensor angle direction is opposite to the car coordinate used by maze logic. */
#define LIDAR_RIGHT_CENTER_DEG      90.0f
#define LIDAR_RIGHT_FRONT_CENTER_DEG 45.0f
#define LIDAR_RIGHT_REAR_CENTER_DEG 135.0f
#define LIDAR_REAR_CENTER_DEG       180.0f
#define LIDAR_LEFT_REAR_CENTER_DEG  225.0f
#define LIDAR_LEFT_CENTER_DEG       270.0f
#define LIDAR_LEFT_FRONT_CENTER_DEG 315.0f
#define LIDAR_SIDE_WINDOW_DEG       12.0f
#define LIDAR_ANGLE_OFFSET_DEG      0.0f
#define LIDAR_FRONT_TIMEOUT_MS      1200U
#define LIDAR_MIN_DIST_MM           60U
#define LIDAR_MAX_DIST_MM           12000U
#define LIDAR_MIN_QUALITY           8U
#define LIDAR_FRONT_MAX_SAMPLES     64U
#define LIDAR_FRONT_MIN_SAMPLES     3U
#define LIDAR_ANGLE_MAX_Q6          (360U * 64U)
#define LIDAR_PARSE_WORK_LEN        2052U
#define LIDAR_SCAN_BIN_DEG          (360.0f / (float)LIDAR_SCAN_BIN_COUNT)
#define LIDAR_SCAN_BIN_MIN_SAMPLES  2U
#define LIDAR_SCAN_SELF_MASK_ENABLE 1U
#define LIDAR_SCAN_SELF_MASK_MAX_MM 140U
#define LIDAR_SCAN_SELF_MASK1_CENTER_DEG 180.0f
#define LIDAR_SCAN_SELF_MASK1_HALF_DEG   58.0f

#define LIDAR_MOTOR_MODE_PWM_F0     0U
#define LIDAR_MOTOR_MODE_RPM_A8     1U
#define LIDAR_MOTOR_MODE_RAW_F0     2U
#define LIDAR_DEFAULT_MOTOR_MODE    LIDAR_MOTOR_MODE_RPM_A8

/* Cache latest raw packet and forward via UART1. */
static uint8_t forward_buf[256];
static volatile uint16_t forward_len = 0;
static volatile uint16_t forward_last_len = 0;
static volatile uint8_t forward_ready = 0;

/* Runtime state. */
static volatile uint8_t lidar_baud_locked = 1U;
static volatile uint32_t last_any_rx_tick = 0U;
static volatile uint32_t last_desc_tick = 0U;
static volatile uint32_t last_data_tick = 0U;
static volatile uint32_t last_recovery_tick = 0U;
static volatile uint32_t last_restart_tick = 0U;
static volatile uint8_t motor_cmd_mode = LIDAR_DEFAULT_MOTOR_MODE;
static volatile uint8_t lidar_bootstrap_active = 0U;
static volatile uint8_t lidar_bootstrap_kick_count = 0U;
static volatile uint32_t lidar_bootstrap_last_tick = 0U;
static volatile uint8_t lidar_startup_boost_active = 0U;
static volatile uint32_t lidar_startup_boost_tick = 0U;
static uint8_t lidar_node_carry[4];
static uint8_t lidar_node_carry_len = 0U;
static volatile float lidar_front_dist_m = 0.0f;
static volatile uint32_t lidar_front_dist_tick = 0U;
static volatile float lidar_right_dist_m = 0.0f;
static volatile uint32_t lidar_right_dist_tick = 0U;
static volatile float lidar_right_front_dist_m = 0.0f;
static volatile uint32_t lidar_right_front_dist_tick = 0U;
static volatile float lidar_right_rear_dist_m = 0.0f;
static volatile uint32_t lidar_right_rear_dist_tick = 0U;
static volatile float lidar_rear_dist_m = 0.0f;
static volatile uint32_t lidar_rear_dist_tick = 0U;
static volatile float lidar_left_rear_dist_m = 0.0f;
static volatile uint32_t lidar_left_rear_dist_tick = 0U;
static volatile float lidar_left_dist_m = 0.0f;
static volatile uint32_t lidar_left_dist_tick = 0U;
static volatile float lidar_left_front_dist_m = 0.0f;
static volatile uint32_t lidar_left_front_dist_tick = 0U;
static volatile float lidar_any_dist_m = 0.0f;
static volatile uint32_t lidar_any_dist_tick = 0U;
static uint16_t lidar_front_accum_mm[LIDAR_FRONT_MAX_SAMPLES];
static uint16_t lidar_front_accum_count = 0U;
static uint16_t lidar_right_accum_mm[LIDAR_FRONT_MAX_SAMPLES];
static uint16_t lidar_right_accum_count = 0U;
static uint16_t lidar_right_front_accum_mm[LIDAR_FRONT_MAX_SAMPLES];
static uint16_t lidar_right_front_accum_count = 0U;
static uint16_t lidar_right_rear_accum_mm[LIDAR_FRONT_MAX_SAMPLES];
static uint16_t lidar_right_rear_accum_count = 0U;
static uint16_t lidar_rear_accum_mm[LIDAR_FRONT_MAX_SAMPLES];
static uint16_t lidar_rear_accum_count = 0U;
static uint16_t lidar_left_rear_accum_mm[LIDAR_FRONT_MAX_SAMPLES];
static uint16_t lidar_left_rear_accum_count = 0U;
static uint16_t lidar_left_accum_mm[LIDAR_FRONT_MAX_SAMPLES];
static uint16_t lidar_left_accum_count = 0U;
static uint16_t lidar_left_front_accum_mm[LIDAR_FRONT_MAX_SAMPLES];
static uint16_t lidar_left_front_accum_count = 0U;
static uint32_t lidar_scan_accum_sum_mm[LIDAR_SCAN_BIN_COUNT];
static uint16_t lidar_scan_accum_count[LIDAR_SCAN_BIN_COUNT];
static uint16_t lidar_scan_accum_masked_count = 0U;
static volatile uint16_t lidar_scan_bins_mm[LIDAR_SCAN_BIN_COUNT];
static volatile uint32_t lidar_scan_bins_tick = 0U;
static volatile uint16_t lidar_scan_masked_count = 0U;

/* TX diagnostics. */
static volatile uint32_t lidar_tx_count = 0U;
static volatile uint32_t lidar_tx_attempt_count = 0U;
static volatile uint32_t lidar_tx_error_count = 0U;

/* Carry + one full DMA frame (2048) for robust stream parsing. */
static uint8_t lidar_parse_work[LIDAR_PARSE_WORK_LEN];

static float lidar_abs_angle_diff_deg(float a, float b)
{
    float d = a - b;
    while (d > 180.0f)
    {
        d -= 360.0f;
    }
    while (d < -180.0f)
    {
        d += 360.0f;
    }
    return fabsf(d);
}

static float lidar_wrap_angle_deg(float angle_deg)
{
    while (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static int lidar_scan_self_masked(float angle_deg, uint16_t dist_mm)
{
#if LIDAR_SCAN_SELF_MASK_ENABLE
    if (dist_mm > LIDAR_SCAN_SELF_MASK_MAX_MM)
    {
        return 0;
    }

    if (lidar_abs_angle_diff_deg(angle_deg,
                                 LIDAR_SCAN_SELF_MASK1_CENTER_DEG) <=
        LIDAR_SCAN_SELF_MASK1_HALF_DEG)
    {
        return 1;
    }
#else
    (void)angle_deg;
    (void)dist_mm;
#endif
    return 0;
}

static int lidar_is_front_angle(float angle_deg)
{
    return (lidar_abs_angle_diff_deg(angle_deg, LIDAR_FRONT_CENTER_DEG) <= LIDAR_FRONT_WINDOW_DEG) ? 1 : 0;
}

static int lidar_is_right_angle(float angle_deg)
{
    return (lidar_abs_angle_diff_deg(angle_deg, LIDAR_RIGHT_CENTER_DEG) <= LIDAR_SIDE_WINDOW_DEG) ? 1 : 0;
}

static int lidar_is_right_front_angle(float angle_deg)
{
    return (lidar_abs_angle_diff_deg(angle_deg, LIDAR_RIGHT_FRONT_CENTER_DEG) <= LIDAR_SIDE_WINDOW_DEG) ? 1 : 0;
}

static int lidar_is_right_rear_angle(float angle_deg)
{
    return (lidar_abs_angle_diff_deg(angle_deg, LIDAR_RIGHT_REAR_CENTER_DEG) <= LIDAR_SIDE_WINDOW_DEG) ? 1 : 0;
}

static int lidar_is_rear_angle(float angle_deg)
{
    return (lidar_abs_angle_diff_deg(angle_deg, LIDAR_REAR_CENTER_DEG) <= LIDAR_SIDE_WINDOW_DEG) ? 1 : 0;
}

static int lidar_is_left_rear_angle(float angle_deg)
{
    return (lidar_abs_angle_diff_deg(angle_deg, LIDAR_LEFT_REAR_CENTER_DEG) <= LIDAR_SIDE_WINDOW_DEG) ? 1 : 0;
}

static int lidar_is_left_angle(float angle_deg)
{
    return (lidar_abs_angle_diff_deg(angle_deg, LIDAR_LEFT_CENTER_DEG) <= LIDAR_SIDE_WINDOW_DEG) ? 1 : 0;
}

static int lidar_is_left_front_angle(float angle_deg)
{
    return (lidar_abs_angle_diff_deg(angle_deg, LIDAR_LEFT_FRONT_CENTER_DEG) <= LIDAR_SIDE_WINDOW_DEG) ? 1 : 0;
}

static void lidar_commit_sector_accum(uint16_t *accum_mm,
                                      uint16_t *accum_count,
                                      volatile float *dist_m,
                                      volatile uint32_t *dist_tick,
                                      uint32_t now)
{
    uint16_t m = 0U;
    uint16_t n = 0U;
    uint16_t selected_mm = 0U;

    if (accum_mm == NULL || accum_count == NULL || dist_m == NULL || dist_tick == NULL)
    {
        return;
    }

    if (*accum_count < LIDAR_FRONT_MIN_SAMPLES)
    {
        *accum_count = 0U;
        return;
    }

    /* Ascending sort for robust percentile pick. */
    for (m = 1U; m < *accum_count; m++)
    {
        uint16_t key = accum_mm[m];
        uint16_t j = m;
        while (j > 0U && accum_mm[j - 1U] > key)
        {
            accum_mm[j] = accum_mm[j - 1U];
            j--;
        }
        accum_mm[j] = key;
    }

    /* Use lower quartile so far-wall points do not dominate obstacle distance. */
    n = (uint16_t)(*accum_count / 4U);
    selected_mm = accum_mm[n];
    *dist_m = (float)selected_mm / 1000.0f;
    *dist_tick = now;
    *accum_count = 0U;
}

static void lidar_commit_sector_accums(uint32_t now)
{
    lidar_commit_sector_accum(lidar_front_accum_mm,
                              &lidar_front_accum_count,
                              &lidar_front_dist_m,
                              &lidar_front_dist_tick,
                              now);
    lidar_commit_sector_accum(lidar_right_accum_mm,
                              &lidar_right_accum_count,
                              &lidar_right_dist_m,
                              &lidar_right_dist_tick,
                              now);
    lidar_commit_sector_accum(lidar_right_front_accum_mm,
                              &lidar_right_front_accum_count,
                              &lidar_right_front_dist_m,
                              &lidar_right_front_dist_tick,
                              now);
    lidar_commit_sector_accum(lidar_right_rear_accum_mm,
                              &lidar_right_rear_accum_count,
                              &lidar_right_rear_dist_m,
                              &lidar_right_rear_dist_tick,
                              now);
    lidar_commit_sector_accum(lidar_rear_accum_mm,
                              &lidar_rear_accum_count,
                              &lidar_rear_dist_m,
                              &lidar_rear_dist_tick,
                              now);
    lidar_commit_sector_accum(lidar_left_rear_accum_mm,
                              &lidar_left_rear_accum_count,
                              &lidar_left_rear_dist_m,
                              &lidar_left_rear_dist_tick,
                              now);
    lidar_commit_sector_accum(lidar_left_accum_mm,
                              &lidar_left_accum_count,
                              &lidar_left_dist_m,
                              &lidar_left_dist_tick,
                              now);
    lidar_commit_sector_accum(lidar_left_front_accum_mm,
                              &lidar_left_front_accum_count,
                              &lidar_left_front_dist_m,
                              &lidar_left_front_dist_tick,
                              now);
}

static void lidar_commit_scan_bins(uint32_t now)
{
    uint16_t i;

    for (i = 0U; i < LIDAR_SCAN_BIN_COUNT; i++)
    {
        if (lidar_scan_accum_count[i] >= LIDAR_SCAN_BIN_MIN_SAMPLES)
        {
            lidar_scan_bins_mm[i] = (uint16_t)((lidar_scan_accum_sum_mm[i] +
                                                (uint32_t)(lidar_scan_accum_count[i] / 2U)) /
                                               (uint32_t)lidar_scan_accum_count[i]);
        }
        else
        {
            lidar_scan_bins_mm[i] = 0U;
        }
        lidar_scan_accum_sum_mm[i] = 0U;
        lidar_scan_accum_count[i] = 0U;
    }
    lidar_scan_masked_count = lidar_scan_accum_masked_count;
    lidar_scan_accum_masked_count = 0U;
    lidar_scan_bins_tick = now;
}

static int lidar_is_scan_descriptor(const uint8_t *data, uint16_t len)
{
    /* Standard scan descriptor: A5 5A 05 00 00 40 81 */
    if (data == NULL || len != 7U)
    {
        return 0;
    }
    if (data[0] != 0xA5U || data[1] != 0x5AU)
    {
        return 0;
    }
    if (data[2] == 0x05U && data[3] == 0x00U && data[4] == 0x00U &&
        data[5] == 0x40U && data[6] == 0x81U)
    {
        return 1;
    }
    return 0;
}

static int lidar_is_response_descriptor_prefix(const uint8_t *data, uint16_t remain)
{
    if (data == NULL || remain < 7U)
    {
        return 0;
    }

    return (data[0] == 0xA5U && data[1] == 0x5AU) ? 1 : 0;
}

static void lidar_apply_baud(uint32_t baud)
{
    (void)HAL_UART_DeInit(&huart3);
    huart3.Init.BaudRate = baud;
    (void)HAL_UART_Init(&huart3);
    LIDAR_UART_Start();
}

static HAL_StatusTypeDef lidar_send_no_payload(uint8_t cmd)
{
    uint8_t req[2] = {LIDAR_CMD_SYNC_BYTE, cmd};
    HAL_StatusTypeDef st;

    lidar_tx_attempt_count++;
    st = HAL_UART_Transmit(&huart3, req, sizeof(req), 100);
    if (st == HAL_OK)
    {
        lidar_tx_count++;
    }
    else
    {
        lidar_tx_error_count++;
    }
    return st;
}

static HAL_StatusTypeDef lidar_send_raw_bytes(const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st;

    if (buf == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    lidar_tx_attempt_count++;
    st = HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 100);
    if (st == HAL_OK)
    {
        lidar_tx_count++;
    }
    else
    {
        lidar_tx_error_count++;
    }
    return st;
}

static HAL_StatusTypeDef lidar_send_with_payload(uint8_t cmd, const uint8_t *payload, uint8_t payload_size)
{
    uint8_t frame[3 + 32 + 1];
    uint8_t checksum = 0;
    uint8_t i = 0;
    HAL_StatusTypeDef st;

    if (payload == NULL || payload_size == 0U || payload_size > 32U)
    {
        return HAL_ERROR;
    }

    frame[0] = LIDAR_CMD_SYNC_BYTE;
    frame[1] = (uint8_t)(cmd | LIDAR_CMDFLAG_HAS_PAYLOAD);
    frame[2] = payload_size;
    memcpy(&frame[3], payload, payload_size);

    for (i = 0; i < (uint8_t)(3U + payload_size); i++)
    {
        checksum ^= frame[i];
    }
    frame[3 + payload_size] = checksum;

    lidar_tx_attempt_count++;
    st = HAL_UART_Transmit(&huart3, frame, (uint16_t)(4U + payload_size), 100);
    if (st == HAL_OK)
    {
        lidar_tx_count++;
    }
    else
    {
        lidar_tx_error_count++;
    }
    return st;
}

static void lidar_send_motor_pwm_mode(uint8_t mode, uint16_t pwm)
{
    uint8_t raw_pwm_cmd[4];
    uint8_t payload_pwm[2];

    if (pwm > 1023U)
    {
        pwm = 1023U;
    }

    payload_pwm[0] = (uint8_t)(pwm & 0xFFU);
    payload_pwm[1] = (uint8_t)((pwm >> 8) & 0xFFU);

    if (mode == LIDAR_MOTOR_MODE_PWM_F0)
    {
        (void)lidar_send_with_payload(LIDAR_CMD_SET_MOTOR_PWM, payload_pwm, sizeof(payload_pwm));
    }
    else if (mode == LIDAR_MOTOR_MODE_RPM_A8)
    {
        (void)lidar_send_with_payload(LIDAR_CMD_HQ_MOTOR_SPEED, payload_pwm, sizeof(payload_pwm));
    }
    else
    {
        raw_pwm_cmd[0] = LIDAR_CMD_SYNC_BYTE;
        raw_pwm_cmd[1] = LIDAR_CMD_SET_MOTOR_PWM;
        raw_pwm_cmd[2] = payload_pwm[0];
        raw_pwm_cmd[3] = payload_pwm[1];
        (void)lidar_send_raw_bytes(raw_pwm_cmd, sizeof(raw_pwm_cmd));
        HAL_Delay(3);
        (void)lidar_send_with_payload(LIDAR_CMD_SET_MOTOR_PWM, payload_pwm, sizeof(payload_pwm));
    }
}

static void lidar_set_startup_motor_pwm(uint16_t pwm)
{
#if LIDAR_STARTUP_MULTI_MODE_ENABLE
    /* C1-compatible startup: send both known motor command forms during spin-up. */
    lidar_send_motor_pwm_mode(LIDAR_MOTOR_MODE_RPM_A8, pwm);
    HAL_Delay(3);
    lidar_send_motor_pwm_mode(LIDAR_MOTOR_MODE_PWM_F0, pwm);
    HAL_Delay(3);
    lidar_send_motor_pwm_mode(LIDAR_MOTOR_MODE_RAW_F0, pwm);
#else
    lidar_send_motor_pwm_mode(motor_cmd_mode, pwm);
#endif
}

static uint8_t lidar_parse_scan_nodes(const uint8_t *data, uint16_t len)
{
    uint16_t total = 0U;
    uint16_t i = 0U;
    uint16_t best_any_mm = 0xFFFFU;
    uint8_t found_any = 0U;
    uint32_t now = HAL_GetTick();

    if (data == NULL || len == 0U)
    {
        return 0U;
    }

    if ((uint16_t)(lidar_node_carry_len + len) > (uint16_t)sizeof(lidar_parse_work))
    {
        len = (uint16_t)(sizeof(lidar_parse_work) - lidar_node_carry_len);
    }

    if (lidar_node_carry_len > 0U)
    {
        memcpy(lidar_parse_work, lidar_node_carry, lidar_node_carry_len);
    }
    memcpy(&lidar_parse_work[lidar_node_carry_len], data, len);
    total = (uint16_t)(lidar_node_carry_len + len);

    while ((uint16_t)(i + 5U) <= total)
    {
        if (lidar_is_response_descriptor_prefix(&lidar_parse_work[i], (uint16_t)(total - i)) != 0)
        {
            i = (uint16_t)(i + 7U);
            continue;
        }

        uint8_t b0 = lidar_parse_work[i];
        uint8_t b1 = lidar_parse_work[i + 1U];
        uint8_t b2 = lidar_parse_work[i + 2U];
        uint8_t b3 = lidar_parse_work[i + 3U];
        uint8_t b4 = lidar_parse_work[i + 4U];
        uint8_t s = (uint8_t)(b0 & 0x1U);
        uint8_t ns = (uint8_t)((b0 >> 1U) & 0x1U);
        uint8_t c = (uint8_t)(b1 & 0x1U);
        uint8_t new_scan = ((s == 1U) && (ns == 0U)) ? 1U : 0U;

        /* Standard scan node: start bits valid and check bit set. */
        if (((s ^ ns) != 1U) || (c != 1U))
        {
            i++;
            continue;
        }

        {
            uint16_t angle_q6 = (uint16_t)(((uint16_t)(b1 >> 1U)) | ((uint16_t)b2 << 7U));
            uint16_t dist_q2 = (uint16_t)(((uint16_t)b3) | ((uint16_t)b4 << 8U));
            uint16_t dist_mm = (uint16_t)(dist_q2 >> 2U);
            float angle_deg = lidar_wrap_angle_deg(((float)angle_q6 / 64.0f) +
                                                   LIDAR_ANGLE_OFFSET_DEG);
            uint8_t quality = (uint8_t)(b0 >> 2U);

            i = (uint16_t)(i + 5U);

            if (new_scan != 0U)
            {
                lidar_commit_sector_accums(now);
                lidar_commit_scan_bins(now);
            }

            if (quality < LIDAR_MIN_QUALITY)
            {
                continue;
            }

            if (angle_q6 >= LIDAR_ANGLE_MAX_Q6)
            {
                continue;
            }

            if (dist_mm < LIDAR_MIN_DIST_MM || dist_mm > LIDAR_MAX_DIST_MM)
            {
                continue;
            }

            if (dist_mm < best_any_mm)
            {
                best_any_mm = dist_mm;
                found_any = 1U;
            }

            {
                if (lidar_scan_self_masked(angle_deg, dist_mm) != 0)
                {
                    if (lidar_scan_accum_masked_count < 0xFFFFU)
                    {
                        lidar_scan_accum_masked_count++;
                    }
                }
                else
                {
                    uint16_t bin = (uint16_t)(angle_deg / LIDAR_SCAN_BIN_DEG);
                    if (bin >= LIDAR_SCAN_BIN_COUNT)
                    {
                        bin = (uint16_t)(LIDAR_SCAN_BIN_COUNT - 1U);
                    }
                    if (lidar_scan_accum_count[bin] < 0xFFFFU)
                    {
                        lidar_scan_accum_sum_mm[bin] += (uint32_t)dist_mm;
                        lidar_scan_accum_count[bin]++;
                    }
                }
            }

            if (lidar_is_front_angle(angle_deg) != 0)
            {
                if (lidar_front_accum_count < LIDAR_FRONT_MAX_SAMPLES)
                {
                    lidar_front_accum_mm[lidar_front_accum_count] = dist_mm;
                    lidar_front_accum_count++;
                }
            }

            if (lidar_is_right_angle(angle_deg) != 0)
            {
                if (lidar_right_accum_count < LIDAR_FRONT_MAX_SAMPLES)
                {
                    lidar_right_accum_mm[lidar_right_accum_count] = dist_mm;
                    lidar_right_accum_count++;
                }
            }

            if (lidar_is_right_front_angle(angle_deg) != 0)
            {
                if (lidar_right_front_accum_count < LIDAR_FRONT_MAX_SAMPLES)
                {
                    lidar_right_front_accum_mm[lidar_right_front_accum_count] = dist_mm;
                    lidar_right_front_accum_count++;
                }
            }

            if (lidar_is_right_rear_angle(angle_deg) != 0)
            {
                if (lidar_right_rear_accum_count < LIDAR_FRONT_MAX_SAMPLES)
                {
                    lidar_right_rear_accum_mm[lidar_right_rear_accum_count] = dist_mm;
                    lidar_right_rear_accum_count++;
                }
            }

            if (lidar_is_rear_angle(angle_deg) != 0)
            {
                if (lidar_rear_accum_count < LIDAR_FRONT_MAX_SAMPLES)
                {
                    lidar_rear_accum_mm[lidar_rear_accum_count] = dist_mm;
                    lidar_rear_accum_count++;
                }
            }

            if (lidar_is_left_rear_angle(angle_deg) != 0)
            {
                if (lidar_left_rear_accum_count < LIDAR_FRONT_MAX_SAMPLES)
                {
                    lidar_left_rear_accum_mm[lidar_left_rear_accum_count] = dist_mm;
                    lidar_left_rear_accum_count++;
                }
            }

            if (lidar_is_left_angle(angle_deg) != 0)
            {
                if (lidar_left_accum_count < LIDAR_FRONT_MAX_SAMPLES)
                {
                    lidar_left_accum_mm[lidar_left_accum_count] = dist_mm;
                    lidar_left_accum_count++;
                }
            }

            if (lidar_is_left_front_angle(angle_deg) != 0)
            {
                if (lidar_left_front_accum_count < LIDAR_FRONT_MAX_SAMPLES)
                {
                    lidar_left_front_accum_mm[lidar_left_front_accum_count] = dist_mm;
                    lidar_left_front_accum_count++;
                }
            }
        }
    }

    lidar_node_carry_len = (uint8_t)(total - i);
    if (lidar_node_carry_len > sizeof(lidar_node_carry))
    {
        lidar_node_carry_len = (uint8_t)sizeof(lidar_node_carry);
    }
    if (lidar_node_carry_len > 0U)
    {
        memcpy(lidar_node_carry, &lidar_parse_work[total - lidar_node_carry_len], lidar_node_carry_len);
    }

    if (found_any != 0U)
    {
        lidar_any_dist_m = (float)best_any_mm / 1000.0f;
        lidar_any_dist_tick = now;
    }

    return found_any;
}

static void lidar_restart_scan_sequence(void)
{
    (void)lidar_send_no_payload(LIDAR_CMD_STOP);
    HAL_Delay(20);
    lidar_startup_boost_active = 1U;
    lidar_startup_boost_tick = HAL_GetTick();
    lidar_set_startup_motor_pwm(LIDAR_STARTUP_PWM);
    HAL_Delay(20);
    (void)lidar_send_no_payload(LIDAR_CMD_SCAN);
}

void Lidar_ProcessFrame(uint8_t *data, uint16_t len)
{
    uint32_t now = HAL_GetTick();
    int is_desc = 0;
    uint8_t has_scan_nodes = 0U;
    uint16_t forward_copy_len = len;
    uint16_t parse_copy_len = len;

    if (data == NULL || len == 0U)
    {
        return;
    }

    if (forward_copy_len > sizeof(forward_buf))
    {
        forward_copy_len = sizeof(forward_buf);
    }
    if (parse_copy_len > (LIDAR_PARSE_WORK_LEN - (uint16_t)sizeof(lidar_node_carry)))
    {
        parse_copy_len = (LIDAR_PARSE_WORK_LEN - (uint16_t)sizeof(lidar_node_carry));
    }

    memcpy(forward_buf, data, forward_copy_len);
    forward_len = forward_copy_len;
    forward_last_len = forward_copy_len;
    forward_ready = 1U;

    is_desc = lidar_is_scan_descriptor(data, forward_copy_len);
    last_any_rx_tick = now;

    if (is_desc)
    {
        last_desc_tick = now;
    }
    else
    {
        has_scan_nodes = lidar_parse_scan_nodes(data, parse_copy_len);
        if (has_scan_nodes != 0U)
        {
            last_data_tick = now;
            lidar_bootstrap_active = 0U;
            lidar_bootstrap_kick_count = 0U;
            if (lidar_startup_boost_active != 0U)
            {
                lidar_startup_boost_active = 0U;
                Lidar_SetMotorPwm(LIDAR_DEFAULT_PWM);
            }
        }
    }
    lidar_baud_locked = 1U;
}

void Lidar_Stop(void)
{
    (void)lidar_send_no_payload(LIDAR_CMD_STOP);
}

void Lidar_SetMotorPwm(uint16_t pwm)
{
    lidar_send_motor_pwm_mode(motor_cmd_mode, pwm);
}

void Lidar_StartScan(void)
{
    (void)lidar_send_no_payload(LIDAR_CMD_SCAN);
}

void Lidar_RequestInfo(void)
{
    (void)lidar_send_no_payload(LIDAR_CMD_GET_INFO);
    HAL_Delay(10);
    (void)lidar_send_no_payload(LIDAR_CMD_GET_HEALTH);
}

uint16_t Lidar_CopyLastFrameHex(char *out, uint16_t maxlen)
{
    uint16_t max_hex_bytes = 0;
    uint16_t n = 0;
    uint16_t i = 0;
    static const char hex[] = "0123456789ABCDEF";

    if (out == NULL || forward_len == 0U || maxlen < 3U)
    {
        return 0U;
    }

    max_hex_bytes = (uint16_t)((maxlen - 1U) / 2U);
    n = (forward_len < max_hex_bytes) ? forward_len : max_hex_bytes;

    for (i = 0; i < n; i++)
    {
        out[2U * i] = hex[(forward_buf[i] >> 4) & 0x0FU];
        out[2U * i + 1U] = hex[forward_buf[i] & 0x0FU];
    }

    out[2U * n] = '\0';
    return (uint16_t)(2U * n);
}

uint16_t Lidar_GetLastFrameLen(void)
{
    return forward_last_len;
}

int Lidar_HasNewFrame(void)
{
    return (forward_ready != 0U) ? 1 : 0;
}

uint32_t Lidar_GetTxCount(void)
{
    return lidar_tx_count;
}

uint32_t Lidar_GetTxAttemptCount(void)
{
    return lidar_tx_attempt_count;
}

uint32_t Lidar_GetTxErrorCount(void)
{
    return lidar_tx_error_count;
}

uint32_t Lidar_GetCurrentBaud(void)
{
    return huart3.Init.BaudRate;
}

int Lidar_IsBaudLocked(void)
{
    return (lidar_baud_locked != 0U) ? 1 : 0;
}

uint32_t Lidar_GetMotorMode(void)
{
    return (uint32_t)motor_cmd_mode;
}

float Lidar_GetFrontDistanceM(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - lidar_front_dist_tick) > LIDAR_FRONT_TIMEOUT_MS)
    {
        return 0.0f;
    }
    return lidar_front_dist_m;
}

float Lidar_GetRightDistanceM(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - lidar_right_dist_tick) > LIDAR_FRONT_TIMEOUT_MS)
    {
        return 0.0f;
    }
    return lidar_right_dist_m;
}

float Lidar_GetRightFrontDistanceM(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - lidar_right_front_dist_tick) > LIDAR_FRONT_TIMEOUT_MS)
    {
        return 0.0f;
    }
    return lidar_right_front_dist_m;
}

float Lidar_GetRightRearDistanceM(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - lidar_right_rear_dist_tick) > LIDAR_FRONT_TIMEOUT_MS)
    {
        return 0.0f;
    }
    return lidar_right_rear_dist_m;
}

float Lidar_GetRearDistanceM(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - lidar_rear_dist_tick) > LIDAR_FRONT_TIMEOUT_MS)
    {
        return 0.0f;
    }
    return lidar_rear_dist_m;
}

float Lidar_GetLeftRearDistanceM(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - lidar_left_rear_dist_tick) > LIDAR_FRONT_TIMEOUT_MS)
    {
        return 0.0f;
    }
    return lidar_left_rear_dist_m;
}

float Lidar_GetLeftDistanceM(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - lidar_left_dist_tick) > LIDAR_FRONT_TIMEOUT_MS)
    {
        return 0.0f;
    }
    return lidar_left_dist_m;
}

float Lidar_GetLeftFrontDistanceM(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - lidar_left_front_dist_tick) > LIDAR_FRONT_TIMEOUT_MS)
    {
        return 0.0f;
    }
    return lidar_left_front_dist_m;
}

uint16_t Lidar_GetScanBinCount(void)
{
    return (uint16_t)LIDAR_SCAN_BIN_COUNT;
}

uint32_t Lidar_GetScanTick(void)
{
    return lidar_scan_bins_tick;
}

uint16_t Lidar_CopyScanBinsMm(uint16_t *out_bins_mm, uint16_t max_bins)
{
    uint16_t i;
    uint16_t count = (max_bins < (uint16_t)LIDAR_SCAN_BIN_COUNT) ?
                     max_bins :
                     (uint16_t)LIDAR_SCAN_BIN_COUNT;

    if (out_bins_mm == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < count; i++)
    {
        out_bins_mm[i] = lidar_scan_bins_mm[i];
    }

    return count;
}

uint16_t Lidar_GetScanMaskedCount(void)
{
    return lidar_scan_masked_count;
}

void Lidar_ForwardTask(void)
{
    static uint32_t last_forward_tick = 0U;
    uint32_t now = HAL_GetTick();
    uint8_t startup_action = 0U;
#if LIDAR_FORWARD_RAW_ENABLE
    static uint32_t last_raw_tick = 0U;
    char hexbuf[(LIDAR_FORWARD_RAW_MAX_BYTES * 2U) + 1U];
    uint16_t hexlen = 0U;
#endif

    if (lidar_bootstrap_active != 0U &&
        (now - last_data_tick) > LIDAR_BOOTSTRAP_KICK_MS &&
        (now - lidar_bootstrap_last_tick) > LIDAR_BOOTSTRAP_KICK_MS)
    {
        lidar_bootstrap_last_tick = now;
        if (lidar_bootstrap_kick_count < LIDAR_BOOTSTRAP_MAX_KICKS)
        {
            lidar_bootstrap_kick_count++;
            lidar_startup_boost_active = 1U;
            lidar_startup_boost_tick = now;
            lidar_set_startup_motor_pwm(LIDAR_STARTUP_PWM);
            (void)lidar_send_no_payload(LIDAR_CMD_SCAN);
            startup_action = 1U;
        }
        else
        {
            /* Escalate once to UART RX restart when repeated startup kicks fail. */
            LIDAR_UART_ForceRestart();
            lidar_bootstrap_kick_count = 0U;
            startup_action = 1U;
        }
        last_recovery_tick = now;
    }

    if (startup_action == 0U &&
        (now - last_data_tick) > LIDAR_RECOVERY_RETRY_MS &&
        (now - last_recovery_tick) > LIDAR_RECOVERY_RETRY_MS)
    {
        last_recovery_tick = now;
        LIDAR_UART_ForceRestart();
        lidar_startup_boost_active = 1U;
        lidar_startup_boost_tick = now;
        lidar_set_startup_motor_pwm(LIDAR_STARTUP_PWM);
        HAL_Delay(5);
        (void)lidar_send_no_payload(LIDAR_CMD_SCAN);
    }

    if (lidar_startup_boost_active != 0U &&
        (now - lidar_startup_boost_tick) > LIDAR_STARTUP_BOOST_MS)
    {
        lidar_startup_boost_active = 0U;
        Lidar_SetMotorPwm(LIDAR_DEFAULT_PWM);
    }

#if LIDAR_AUTO_RESTART_SCAN_EN
    /* Optional hard restart path. Keep disabled by default to avoid motor stop/start hiccups. */
    if ((now - last_any_rx_tick) > LIDAR_STREAM_LOST_MS &&
        (now - last_restart_tick) > LIDAR_RESTART_GAP_MS)
    {
        last_restart_tick = now;
        lidar_restart_scan_sequence();
    }
#endif

    if (forward_ready == 0U)
    {
        return;
    }

    if ((now - last_forward_tick) < LIDAR_FORWARD_INTERVAL_MS)
    {
        return;
    }
    last_forward_tick = now;

#if LIDAR_FORWARD_RAW_ENABLE
    if ((now - last_raw_tick) >= LIDAR_FORWARD_RAW_INTERVAL_MS)
    {
        last_raw_tick = now;
        hexlen = Lidar_CopyLastFrameHex(hexbuf, sizeof(hexbuf));
        if (hexlen > 0U)
        {
            (void)HAL_UART_Transmit(&huart1, (uint8_t *)"RAW:", 4, 20);
            (void)HAL_UART_Transmit(&huart1, (uint8_t *)hexbuf, hexlen, 20);
            (void)HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 20);
        }
    }
#endif

    forward_ready = 0U;
}

void Lidar_InitSequence(void)
{
    uint32_t now = HAL_GetTick();

    lidar_baud_locked = 1U;
    last_any_rx_tick = now;
    last_desc_tick = 0U;
    last_data_tick = now;
    last_recovery_tick = now;
    last_restart_tick = now;
    lidar_bootstrap_active = 1U;
    lidar_bootstrap_kick_count = 0U;
    lidar_bootstrap_last_tick = now;
    lidar_startup_boost_active = 1U;
    lidar_startup_boost_tick = now;
    motor_cmd_mode = LIDAR_DEFAULT_MOTOR_MODE;
    forward_ready = 0U;
    lidar_node_carry_len = 0U;
    lidar_front_accum_count = 0U;
    lidar_right_accum_count = 0U;
    lidar_right_front_accum_count = 0U;
    lidar_right_rear_accum_count = 0U;
    lidar_rear_accum_count = 0U;
    lidar_left_rear_accum_count = 0U;
    lidar_left_accum_count = 0U;
    lidar_left_front_accum_count = 0U;
    memset(lidar_scan_accum_sum_mm, 0, sizeof(lidar_scan_accum_sum_mm));
    memset(lidar_scan_accum_count, 0, sizeof(lidar_scan_accum_count));
    memset((void *)lidar_scan_bins_mm, 0, sizeof(lidar_scan_bins_mm));
    lidar_scan_accum_masked_count = 0U;
    lidar_scan_masked_count = 0U;
    lidar_scan_bins_tick = 0U;
    lidar_front_dist_m = 0.0f;
    lidar_front_dist_tick = 0U;
    lidar_right_dist_m = 0.0f;
    lidar_right_dist_tick = 0U;
    lidar_right_front_dist_m = 0.0f;
    lidar_right_front_dist_tick = 0U;
    lidar_right_rear_dist_m = 0.0f;
    lidar_right_rear_dist_tick = 0U;
    lidar_rear_dist_m = 0.0f;
    lidar_rear_dist_tick = 0U;
    lidar_left_rear_dist_m = 0.0f;
    lidar_left_rear_dist_tick = 0U;
    lidar_left_dist_m = 0.0f;
    lidar_left_dist_tick = 0U;
    lidar_left_front_dist_m = 0.0f;
    lidar_left_front_dist_tick = 0U;
    lidar_any_dist_m = 0.0f;
    lidar_any_dist_tick = 0U;

    lidar_apply_baud(LIDAR_FIX_BAUD);
    HAL_Delay(60);
    LIDAR_UART_ForceRestart();
    HAL_Delay(20);
    lidar_restart_scan_sequence();
    HAL_Delay(20);
    (void)lidar_send_no_payload(LIDAR_CMD_SCAN);
}
