/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../../BSP/Inc/bluetooth.h"
#include "../../BSP/Inc/encoder.h"
#include "../../BSP/Inc/inv_mpu.h"
#include "../../BSP/Inc/inv_mpu_dmp_motion_driver.h"
#include "../../BSP/Inc/lidar.h"
#include "../../BSP/Inc/motor.h"
#include "../../BSP/Inc/mpu6050.h"
#include "../../BSP/Inc/pid.h"
#include "../../BSP/Inc/ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  MAZE_STATE_FOLLOW = 0,
  MAZE_STATE_TURN_RIGHT,
  MAZE_STATE_TURN_LEFT,
  MAZE_STATE_TURN_BACK,
  MAZE_STATE_POST_TURN_FORWARD,
  MAZE_STATE_PRETURN_STOP
} maze_state_t;

typedef enum
{
  MAZE_SIMPLE_FOLLOW = 0,
  MAZE_SIMPLE_START_STRAIGHT,
  MAZE_SIMPLE_APPROACH_JUNCTION,
  MAZE_SIMPLE_CENTER_ENTRY,
  MAZE_SIMPLE_STOP_SCAN,
  MAZE_SIMPLE_TURN_RIGHT,
  MAZE_SIMPLE_TURN_LEFT,
  MAZE_SIMPLE_BACKUP,
  MAZE_SIMPLE_POST_FORWARD,
  MAZE_SIMPLE_TURN_BACK,
  MAZE_SIMPLE_PRE_TURN_FORWARD,
  MAZE_SIMPLE_POST_TURN_LOCK,
  MAZE_SIMPLE_START_ALIGN,
  MAZE_SIMPLE_YAW_RECOVER
} maze_simple_state_t;

typedef enum
{
  MAZE_RETURN_START_RESULT_OK = 0,
  MAZE_RETURN_START_RESULT_NO_PATH,
  MAZE_RETURN_START_RESULT_NO_POSE
} maze_return_start_result_t;

typedef enum
{
  MAZE_RETURN_PHASE_IDLE = 0,
  MAZE_RETURN_PHASE_BUILD_PLAN,
  MAZE_RETURN_PHASE_ALIGN_TO_PATH,
  MAZE_RETURN_PHASE_FOLLOW_ASTAR,
  MAZE_RETURN_PHASE_FOLLOW_REPLAY,
  MAZE_RETURN_PHASE_REPLAN,
  MAZE_RETURN_PHASE_HOME_APPROACH,
  MAZE_RETURN_PHASE_DONE,
  MAZE_RETURN_PHASE_FAIL
} maze_return_phase_t;

typedef enum
{
  MAZE_RETURN_PLAN_NONE = 0,
  MAZE_RETURN_PLAN_ASTAR,
  MAZE_RETURN_PLAN_TOPO_REPLAY,
  MAZE_RETURN_PLAN_ASTAR_FALLBACK
} maze_return_plan_t;

typedef enum
{
  MAZE_RETURN_UPDATE_STOP_SCAN = 0,
  MAZE_RETURN_UPDATE_FOLLOW
} maze_return_update_mode_t;

typedef enum
{
  MAZE_RETURN_ACTION_NONE = 0,
  MAZE_RETURN_ACTION_DRIVE,
  MAZE_RETURN_ACTION_STOP,
  MAZE_RETURN_ACTION_TURN_LEFT,
  MAZE_RETURN_ACTION_TURN_RIGHT,
  MAZE_RETURN_ACTION_TURN_BACK,
  MAZE_RETURN_ACTION_DONE,
  MAZE_RETURN_ACTION_FAIL,
  MAZE_RETURN_ACTION_HANDLED
} maze_return_action_t;

typedef struct
{
  float front_m;
  uint8_t wait_timeout;
  uint8_t front_clear;
  uint8_t right_open_stop;
  uint8_t right_probe;
  uint8_t right_front_open;
  uint8_t left_open;
  uint8_t left_front_open;
  uint8_t topo_deadend;
  uint32_t *astar_block_tick;
} maze_return_stop_scan_input_t;

typedef struct
{
  float front_m;
  uint8_t right_open;
  uint8_t left_open;
  uint32_t *astar_block_tick;
  uint32_t *left_branch_candidate_tick;
  uint32_t *right_branch_candidate_tick;
  uint32_t *stall_start_tick;
} maze_return_follow_input_t;

typedef struct
{
  maze_return_update_mode_t mode;
  const maze_return_stop_scan_input_t *stop_scan;
  const maze_return_follow_input_t *follow;
} maze_return_update_input_t;

typedef enum
{
  MAZE_CENTERLINE_IDLE = 0,
  MAZE_CENTERLINE_BRAKE,
  MAZE_CENTERLINE_TURN_OUT,
  MAZE_CENTERLINE_SHIFT,
  MAZE_CENTERLINE_TURN_BACK,
  MAZE_CENTERLINE_SETTLE
} maze_centerline_phase_t;

typedef enum
{
  CALIB_STATE_IDLE = 0,
  CALIB_STATE_PREPARE,
  CALIB_STATE_STRAIGHT,
  CALIB_STATE_PAUSE,
  CALIB_STATE_TURN
} calib_state_t;

typedef struct
{
  uint16_t id;
  float x_m;
  float y_m;
  int8_t heading_quadrant;
  uint8_t openings;
  uint8_t open_world;
  uint8_t decision;
  uint8_t tried;
  uint8_t kind;
  uint8_t visits;
  uint16_t front_mm;
  uint16_t right_mm;
  uint16_t rear_mm;
  uint16_t left_mm;
  uint32_t tick;
} maze_topo_node_t;

typedef struct
{
  uint32_t tick;
  float x_m;
  float y_m;
  float theta_rad;
  uint8_t valid;
} ogm_pose_sample_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_OUT  100.0f
#define MAX_IOUT 45.0f
#define MAZE_PROFILE_DORM_L 1U
#define MAZE_PROFILE_LAB_70CM 2U
#ifndef MAZE_PROFILE
#define MAZE_PROFILE MAZE_PROFILE_LAB_70CM
#endif

#if MAZE_PROFILE == MAZE_PROFILE_DORM_L
#define FW_TUNE_TAG "T90M54D"
#define MAZE_PROFILE_NAME "DORM_L"
#define PROFILE_DRIVE_YAW_HOLD_KP_PWM_PER_DEG 1.35f
#define PROFILE_DRIVE_YAW_HOLD_CORR_MAX_PWM 48.0f
#define PROFILE_START_ALIGN_MAX_CORR_DEG 8.0f
#define PROFILE_APPROACH_FRONT_MIN_M 0.28f
#define PROFILE_POST_FORWARD_MS 420U
#define PROFILE_PRE_TURN_BRAKE_MS 120U
#define PROFILE_PRE_TURN_FORWARD_MS 560U
#define PROFILE_PRE_TURN_FORWARD_M 0.075f
#define PROFILE_PRE_TURN_FRONT_MIN_M 0.26f
#define PROFILE_RIGHT_GAP_ENTRY_M 0.55f
#define PROFILE_LEFT_GAP_ENTRY_M 0.42f
#define PROFILE_RIGHT_CENTER_FRONT_MIN_M 0.34f
#define PROFILE_LEFT_CENTER_FRONT_MIN_M 0.30f
#define PROFILE_CENTER_SLOW_FRONT_M 0.42f
#define PROFILE_RIGHT_OPEN_M 0.62f
#define PROFILE_LEFT_OPEN_M 0.52f
#define PROFILE_FRONT_CLEAR_M 0.58f
#define PROFILE_SIDE_DIAG_OPEN_M 0.42f
#define PROFILE_RIGHT_FRONT_OPEN_M 0.50f
#define PROFILE_BACKUP_CENTER_M 0.36f
#define PROFILE_BACKUP_CENTER_MIN_M 0.18f
#define PROFILE_BACKUP_CENTER_MS 1800U
#define PROFILE_BACKUP_TURN_OPPOSITE_SAFE_M 0.31f
#define PROFILE_BACKUP_TURN_SAME_SAFE_M 0.30f
#define PROFILE_BACKUP_TURN_FRONT_SAFE_M 0.28f
#define PROFILE_BACKUP_TURN_REAR_MIN_M 0.14f
#define PROFILE_BACKUP_TURN_EXTRA_M 0.18f
#define PROFILE_BACKUP_TURN_EXTRA_MS 1200U
#define PROFILE_AXIS_DRIVE_ENABLE 0U
#define PROFILE_MANHATTAN_SIDE_PANIC_M 0.085f
#define PROFILE_MANHATTAN_FRONT_DIAG_PANIC_M 0.115f
#define PROFILE_POST_TURN_SIDE_IGNORE_MS 950U
#define PROFILE_FORWARD_SIDE_DANGER_M 0.180f
#define PROFILE_FORWARD_SIDE_PANIC_M 0.130f
#define PROFILE_FORWARD_STEER_PWM 34.0f
#define PROFILE_FORWARD_SLOW_PWM 235.0f
#elif MAZE_PROFILE == MAZE_PROFILE_LAB_70CM
#define FW_TUNE_TAG "T90M132L"
#define MAZE_PROFILE_NAME "LAB_70"
#define PROFILE_DRIVE_YAW_HOLD_KP_PWM_PER_DEG 1.65f
#define PROFILE_DRIVE_YAW_HOLD_CORR_MAX_PWM 64.0f
#define PROFILE_START_ALIGN_MAX_CORR_DEG 10.0f
#define PROFILE_APPROACH_FRONT_MIN_M 0.34f
#define PROFILE_POST_FORWARD_MS 620U
#define PROFILE_PRE_TURN_BRAKE_MS 90U
#define PROFILE_PRE_TURN_FORWARD_MS 0U
#define PROFILE_PRE_TURN_FORWARD_M 0.0f
#define PROFILE_PRE_TURN_FRONT_MIN_M 0.40f
#define PROFILE_RIGHT_GAP_ENTRY_M 0.46f
#define PROFILE_LEFT_GAP_ENTRY_M 0.46f
#define PROFILE_RIGHT_CENTER_FRONT_MIN_M 0.30f
#define PROFILE_LEFT_CENTER_FRONT_MIN_M 0.30f
#define PROFILE_CENTER_SLOW_FRONT_M 1.05f
#define PROFILE_RIGHT_OPEN_M 1.05f
#define PROFILE_LEFT_OPEN_M 0.95f
#define PROFILE_FRONT_CLEAR_M 0.56f
#define PROFILE_SIDE_DIAG_OPEN_M 0.95f
#define PROFILE_RIGHT_FRONT_OPEN_M 1.02f
#define PROFILE_BACKUP_CENTER_M 0.26f
#define PROFILE_BACKUP_CENTER_MIN_M 0.12f
#define PROFILE_BACKUP_CENTER_MS 1200U
#define PROFILE_BACKUP_TURN_OPPOSITE_SAFE_M 0.22f
#define PROFILE_BACKUP_TURN_SAME_SAFE_M 0.22f
#define PROFILE_BACKUP_TURN_FRONT_SAFE_M 0.22f
#define PROFILE_BACKUP_TURN_REAR_MIN_M 0.12f
#define PROFILE_BACKUP_TURN_EXTRA_M 0.09f
#define PROFILE_BACKUP_TURN_EXTRA_MS 650U
#define PROFILE_AXIS_DRIVE_ENABLE 1U
#define PROFILE_MANHATTAN_SIDE_PANIC_M 0.180f
#define PROFILE_MANHATTAN_FRONT_DIAG_PANIC_M 0.260f
#define PROFILE_POST_TURN_SIDE_IGNORE_MS 1600U
#define PROFILE_FORWARD_SIDE_DANGER_M 0.190f
#define PROFILE_FORWARD_SIDE_PANIC_M 0.190f
#define PROFILE_FORWARD_STEER_PWM 12.0f
#define PROFILE_FORWARD_SLOW_PWM 265.0f
#else
#error "Unsupported MAZE_PROFILE"
#endif

#define SELFDBG_ENABLE 1U
#define MAZE_TILT_ABORT_ENABLE 1U
#define MAZE_TILT_ABORT_ROLL_DEG 18.0f
#define MAZE_TILT_ABORT_PITCH_DEG 14.0f
#define DRIVE_PID_ENABLE 1U
#define DRIVE_PID_WINDOW_TICKS 10U
/* Feedforward map: PWM * this factor ~= encoder pulses per control tick (1ms). */
#define DRIVE_PID_TARGET_PULSE_PER_PWM_TICK 0.0038f
#define DRIVE_PID_IDLE_RESET_MS 120U
/* Manual straight trim applied before motor mapping; keep at 0 while yaw hold is tuned. */
#define DRIVE_STRAIGHT_TRIM_FORWARD_PWM 0.0f
#define DRIVE_STRAIGHT_TRIM_REVERSE_PWM 0.0f
#define DRIVE_YAW_HOLD_ENABLE 1U
#define DRIVE_YAW_HOLD_KP_PWM_PER_DEG PROFILE_DRIVE_YAW_HOLD_KP_PWM_PER_DEG
#define DRIVE_YAW_HOLD_REVERSE_KP_PWM_PER_DEG 1.25f
#define DRIVE_YAW_HOLD_CORR_MAX_PWM PROFILE_DRIVE_YAW_HOLD_CORR_MAX_PWM
#define DRIVE_YAW_HOLD_REVERSE_CORR_MAX_PWM 36.0f
#define DRIVE_YAW_HOLD_CMD_DIFF_MAX_PWM 12.0f
#define DRIVE_AUTO_KICK_ENABLE 0U
#define DRIVE_STRAIGHT_SUPPRESS_KICK 1U
#define DRIVE_AUTO_STRAIGHT_PWM_LIMIT 300.0f
#define DRIVE_AUTO_TURN_PWM_LIMIT 300.0f
#define COUNTS_PER_METER 6139.0f
#define DIST_VALID_MIN_M 0.03f
#define DIST_VALID_MAX_M 12.0f
#define DIST_HOLD_MS 2500U
#define DIST_LPF_ALPHA_FALL 0.35f
#define DIST_LPF_ALPHA_RISE 0.08f
#define DIST_FAST_UPDATE_MS 20U
#define LIDAR_READY_STABLE_COUNT 3U
#define LIDAR_IDLE_REINIT_ENABLE 1U
#define LIDAR_IDLE_REINIT_MS 10000U
#define LIDAR_IDLE_REINIT_COOLDOWN_MS 10000U

#define MAZE_PWM_BASE 282.0f
#define MAZE_PWM_TURN 300.0f
#define MAZE_PWM_MIN 270.0f
#define MAZE_PWM_MAX 420.0f
#define MAZE_KP_PWM_PER_M 180.0f
#define MAZE_PWM_KICK 340.0f
#define MAZE_KICK_MS 260U
#define MAZE_CORR_PWM_MAX 55.0f
#define MAZE_NEAR_SLOW_M 0.65f
#define MAZE_NEAR_SLOW_SCALE 1.00f

#define MAZE_DECIDE_INTERVAL_MS 260U
#define MAZE_TURN_RIGHT_MS 650U
#define MAZE_TURN_LEFT_MS 650U
#define MAZE_TURN_BACK_MS 1250U
#define MAZE_TURN_MIN_MS 180U
#define MAZE_TURN_MAX_EXTRA_MS 1600U
#define MAZE_SIMPLE_RULE_ENABLE 1U
#define MAZE_TURN_RIGHT_RAD 1.505f
#define MAZE_TURN_LEFT_RAD 1.500f
#define MAZE_TURN_BACK_RAD 3.10f
#define MAZE_SIMPLE_TURN_BACK_STEP_RAD MAZE_TURN_LEFT_RAD
#define MAZE_TURN_SLOWDOWN_RATIO 0.42f
#define MAZE_TURN_PWM_FINAL 235.0f
#define MAZE_POST_TURN_FORWARD_MS 220U
#define MAZE_SIMPLE_START_STRAIGHT_MS 700U
#define MAZE_SIMPLE_START_STRAIGHT_M 0.12f
#define MAZE_SIMPLE_START_FRONT_MIN_M 0.35f
#define MAZE_SIMPLE_APPROACH_JUNCTION_MS 900U
#define MAZE_SIMPLE_APPROACH_FRONT_MIN_M PROFILE_APPROACH_FRONT_MIN_M
#define MAZE_SIMPLE_POST_FORWARD_MS PROFILE_POST_FORWARD_MS
#define MAZE_SIMPLE_POST_TURN_COUNTER_BRAKE_MS 60U
#define MAZE_SIMPLE_POST_TURN_COUNTER_BRAKE_PWM 220.0f
#define MAZE_SIMPLE_POST_TURN_BRAKE_MS 170U
#define MAZE_SIMPLE_POST_TURN_BRAKE_PWM 420.0f
#define MAZE_SIMPLE_POST_TURN_SETTLE_MS 320U
#define MAZE_SIMPLE_POST_LOCK_ENABLE 1U
#define MAZE_SIMPLE_POST_LOCK_BRAKE_MS 140U
#define MAZE_SIMPLE_POST_LOCK_SETTLE_MS 160U
#define MAZE_SIMPLE_POST_LOCK_YAW_DEADBAND_DEG 1.4f
#define MAZE_SIMPLE_POST_LOCK_YAW_MAX_MS 720U
#define MAZE_SIMPLE_POST_LOCK_KP_PWM_PER_DEG 12.0f
#define MAZE_SIMPLE_POST_LOCK_PWM_MIN 225.0f
#define MAZE_SIMPLE_POST_LOCK_PWM_MAX 285.0f
#define MAZE_SIMPLE_START_ALIGN_ENABLE 1U
#define MAZE_SIMPLE_START_ALIGN_PHYSICAL_ENABLE 1U
#define MAZE_SIMPLE_START_ALIGN_WAIT_MS 550U
#define MAZE_SIMPLE_START_ALIGN_SCAN_MAX_MS 2600U
#define MAZE_SIMPLE_START_ALIGN_TURN_MAX_MS 1600U
#define MAZE_SIMPLE_START_ALIGN_SETTLE_MS 480U
#define MAZE_SIMPLE_START_ALIGN_DEADBAND_DEG 1.5f
#define MAZE_SIMPLE_START_ALIGN_GAIN 0.85f
#define MAZE_SIMPLE_START_ALIGN_MIN_CORR_DEG 2.0f
#define MAZE_SIMPLE_START_ALIGN_MAX_CORR_DEG PROFILE_START_ALIGN_MAX_CORR_DEG
#define MAZE_SIMPLE_START_ALIGN_MAX_ANGLE_DEG 18.0f
#define MAZE_SIMPLE_START_ALIGN_KP_PWM_PER_DEG 12.0f
#define MAZE_SIMPLE_START_ALIGN_PWM_MIN 210.0f
#define MAZE_SIMPLE_START_ALIGN_PWM_MAX 260.0f
#define MAZE_SIMPLE_START_ALIGN_MIN_POINTS 4U
#define MAZE_SIMPLE_START_ALIGN_MIN_SUPPORT 18U
#define MAZE_SIMPLE_START_ALIGN_MIN_SIDE_M 0.12f
#define MAZE_SIMPLE_START_ALIGN_MAX_SIDE_M 0.85f
#define MAZE_SIMPLE_START_ALIGN_MAX_ABS_X_M 0.68f
#define MAZE_SIMPLE_YAW_GUARD_ENABLE 1U
#define MAZE_SIMPLE_YAW_GUARD_ERR_DEG 24.0f
#define MAZE_SIMPLE_YAW_GUARD_CONFIRM_MS 280U
#define MAZE_SIMPLE_YAW_RECOVER_BRAKE_MS 120U
#define MAZE_SIMPLE_YAW_RECOVER_SETTLE_MS 180U
#define MAZE_SIMPLE_YAW_RECOVER_MAX_MS 1200U
#define MAZE_SIMPLE_YAW_RECOVER_DEADBAND_DEG 2.0f
#define MAZE_SIMPLE_YAW_RECOVER_KP_PWM_PER_DEG 10.0f
#define MAZE_SIMPLE_YAW_RECOVER_PWM_MIN 220.0f
#define MAZE_SIMPLE_YAW_RECOVER_PWM_MAX 285.0f
#define MAZE_TOPO_COMMIT_DEBUG 1U
#define MAZE_TOPO_COMMIT_MIN_MS 250U
#define MAZE_TOPO_COMMIT_SIDE_MIN_LEAVE_M 0.18f
#define MAZE_TOPO_COMMIT_SIDE_LEAVE_WAIT_MS 1800U
#define MAZE_TOPO_COMMIT_SIDE_QUALITY_MIN_MS 520U
#define MAZE_TOPO_COMMIT_SIDE_FRONT_MIN_M 0.22f
#define MAZE_TOPO_COMMIT_SIDE_DIAG_MIN_M 0.12f
#define MAZE_TOPO_COMMIT_SIDE_CLEAR_MIN_M 0.14f
#define MAZE_TOPO_REVISIT_TURN_FRONT_MIN_M 0.34f
#define MAZE_TOPO_REVISIT_TURN_DIAG_MIN_M 0.30f
#define MAZE_TOPO_REVISIT_TURN_SIDE_MIN_M 0.28f
#define MAZE_TOPO_SIDE_ENTRY_PANIC_FRONT_M 0.18f
#define MAZE_TOPO_SIDE_ENTRY_PANIC_DIAG_M 0.10f
#define MAZE_TOPO_SIDE_ENTRY_PANIC_SIDE_M 0.12f
#define MAZE_SIMPLE_STOP_BRAKE_MS 120U
#define MAZE_SIMPLE_STOP_BRAKE_PWM 270.0f
#define MAZE_SIMPLE_PRE_TURN_BRAKE_MS PROFILE_PRE_TURN_BRAKE_MS
#define MAZE_SIMPLE_PRE_TURN_FORWARD_MS PROFILE_PRE_TURN_FORWARD_MS
#define MAZE_SIMPLE_PRE_TURN_FORWARD_M PROFILE_PRE_TURN_FORWARD_M
#define MAZE_SIMPLE_PRE_TURN_FRONT_MIN_M PROFILE_PRE_TURN_FRONT_MIN_M
#define MAZE_SIMPLE_CENTER_ENTRY_MS 2900U
#define MAZE_SIMPLE_CENTER_TURN_MIN_M 0.14f
#define MAZE_SIMPLE_CENTER_TURN_FRONT_HARD_MIN_M 0.24f
#define MAZE_SIMPLE_RIGHT_GAP_ENTRY_M PROFILE_RIGHT_GAP_ENTRY_M
#define MAZE_SIMPLE_LEFT_GAP_ENTRY_M PROFILE_LEFT_GAP_ENTRY_M
#define MAZE_SIMPLE_RIGHT_CENTER_FRONT_MIN_M PROFILE_RIGHT_CENTER_FRONT_MIN_M
#define MAZE_SIMPLE_LEFT_CENTER_FRONT_MIN_M PROFILE_LEFT_CENTER_FRONT_MIN_M
#define MAZE_SIMPLE_CENTER_SLOW_FRONT_M PROFILE_CENTER_SLOW_FRONT_M
#define MAZE_SIMPLE_CENTER_FRONT_MIN_M MAZE_SIMPLE_RIGHT_CENTER_FRONT_MIN_M
#define MAZE_SIMPLE_STOP_SCAN_MS 850U
#define MAZE_SIMPLE_DECISION_FRESH_MS 320U
#define MAZE_SIMPLE_DECISION_WAIT_MAX_MS 1500U
#define MAZE_SIMPLE_RIGHT_OPEN_M PROFILE_RIGHT_OPEN_M
#define MAZE_SIMPLE_RIGHT_FRONT_OPEN_M PROFILE_RIGHT_FRONT_OPEN_M
#define MAZE_SIMPLE_RIGHT_FRONT_CORNER_MIN_M 0.16f
#define MAZE_SIMPLE_RIGHT_PROBE_SIDE_MIN_M 0.46f
#define MAZE_SIMPLE_RIGHT_PROBE_FRONT_MIN_M 0.54f
#define MAZE_SIMPLE_RIGHT_PROBE_ROOM_MIN_M 0.36f
#define MAZE_SIMPLE_LEFT_OPEN_M PROFILE_LEFT_OPEN_M
#define MAZE_SIMPLE_FRONT_CLEAR_M PROFILE_FRONT_CLEAR_M
#define MAZE_SIMPLE_FRONT_BLOCK_M 0.43f
#define MAZE_SIMPLE_RIGHT_BRANCH_CONFIRM_MS 420U
#define MAZE_SIMPLE_RIGHT_BRANCH_COOLDOWN_MS 1800U
#define MAZE_SIMPLE_RIGHT_BRANCH_FRONT_MIN_M 0.70f
#define MAZE_SIMPLE_RIGHT_BRANCH_STRONG_M 1.05f
#define MAZE_SIMPLE_LEFT_BRANCH_CONFIRM_MS 360U
#define MAZE_SIMPLE_LEFT_BRANCH_COOLDOWN_MS 1800U
#define MAZE_SIMPLE_LEFT_BRANCH_FRONT_MIN_M 0.70f
#define MAZE_SIMPLE_BACKUP_MIN_MS 650U
#define MAZE_SIMPLE_BACKUP_MAX_MS 2200U
#define MAZE_SIMPLE_BACKUP_MAX_M 0.58f
#define MAZE_SIMPLE_BACKUP_CENTER_M PROFILE_BACKUP_CENTER_M
#define MAZE_SIMPLE_BACKUP_CENTER_MIN_M PROFILE_BACKUP_CENTER_MIN_M
#define MAZE_SIMPLE_BACKUP_CENTER_MS PROFILE_BACKUP_CENTER_MS
#define MAZE_SIMPLE_BACKUP_CENTER_MIN_MS 450U
#define MAZE_SIMPLE_BACKUP_REAR_STOP_M 0.10f
#define MAZE_SIMPLE_BACKUP_REAR_EMERGENCY_M 0.08f
#define MAZE_SIMPLE_BACKUP_REAR_HARD_STOP_M 0.08f
#define MAZE_SIMPLE_BACKUP_REAR_DIAG_HARD_STOP_M 0.08f
#define MAZE_SIMPLE_BACKUP_PWM 270.0f
#define MAZE_SIMPLE_BACKUP_DISABLE_KICK 1U
#define MAZE_SIMPLE_BACKUP_TRIM_PWM 0.0f
#define MAZE_SIMPLE_BACKUP_TURN_OPPOSITE_SAFE_M PROFILE_BACKUP_TURN_OPPOSITE_SAFE_M
#define MAZE_SIMPLE_BACKUP_TURN_SAME_SAFE_M PROFILE_BACKUP_TURN_SAME_SAFE_M
#define MAZE_SIMPLE_BACKUP_TURN_FRONT_SAFE_M PROFILE_BACKUP_TURN_FRONT_SAFE_M
#define MAZE_SIMPLE_BACKUP_TURN_REAR_MIN_M PROFILE_BACKUP_TURN_REAR_MIN_M
#define MAZE_SIMPLE_BACKUP_TURN_EXTRA_M PROFILE_BACKUP_TURN_EXTRA_M
#define MAZE_SIMPLE_BACKUP_TURN_EXTRA_MS PROFILE_BACKUP_TURN_EXTRA_MS
#define MAZE_SIMPLE_BACKUP_TURN_PWM 250.0f
#define MAZE_SIMPLE_BACKUP_TURN_RIGHT_RAD 1.505f
#define MAZE_SIMPLE_BACKUP_TURN_LEFT_RAD 1.500f
#define MAZE_SIMPLE_BACKUP_TURN_SLOWDOWN_RATIO 0.40f
#define MAZE_SIMPLE_BACKUP_TURN_PWM_FINAL 220.0f
#define MAZE_SIMPLE_BACKUP_SIDE_OPEN_M 0.43f
#define MAZE_SIMPLE_BACKUP_TOPO_SIDE_OPEN_M 0.62f
#define MAZE_SIMPLE_BACKUP_TOPO_FRONT_MIN_M 0.34f
#define MAZE_SIMPLE_BACKUP_TOPO_DIAG_MIN_M 0.24f
#define MAZE_SIMPLE_BACKUP_SIDE_MARGIN_M 0.08f
#define MAZE_SIMPLE_BACKUP_SIDE_DANGER_M 0.22f
#define MAZE_SIMPLE_BACKUP_SIDE_PANIC_M 0.15f
#define MAZE_SIMPLE_BACKUP_STEER_PWM 64.0f
#define MAZE_SIMPLE_BACKUP_WALL_NUDGE_M 0.21f
#define MAZE_SIMPLE_BACKUP_WALL_PANIC_M 0.15f
#define MAZE_SIMPLE_BACKUP_WALL_NUDGE_PWM 0.0f
#define MAZE_SIMPLE_BACKUP_WALL_PANIC_PWM 0.0f
#define MAZE_SIMPLE_BACKUP_SLOW_PWM 235.0f
#define MAZE_SIMPLE_BACKUP_ALIGN_ENABLE 1U
#define MAZE_SIMPLE_BACKUP_ALIGN_DEADBAND_DEG 1.6f
#define MAZE_SIMPLE_BACKUP_ALIGN_MAX_MS 1600U
#define MAZE_SIMPLE_BACKUP_ALIGN_BRAKE_MS 140U
#define MAZE_SIMPLE_BACKUP_ALIGN_KP_PWM_PER_DEG 12.0f
#define MAZE_SIMPLE_BACKUP_ALIGN_PWM_MIN 270.0f
#define MAZE_SIMPLE_BACKUP_ALIGN_PWM_MAX 320.0f
#define MAZE_SIMPLE_BACKUP_SIDE_TURN_ENABLE 1U
#define MAZE_SIMPLE_TURN_CLEAR_GUARD_ENABLE 0U
#define MAZE_SIMPLE_RIGHT_TURN_SIDE_SAFE_M 0.28f
#define MAZE_SIMPLE_RIGHT_TURN_REAR_SAFE_M 0.14f
#define MAZE_SIMPLE_RIGHT_TURN_FRONT_ROOM_M 0.36f
#define MAZE_SIMPLE_TURN_SIDE_SAFE_M 0.22f
#define MAZE_SIMPLE_TURN_FRONT_DIAG_SAFE_M 0.22f
#define MAZE_SIMPLE_SIDE_DIAG_OPEN_M PROFILE_SIDE_DIAG_OPEN_M
#define MAZE_SIMPLE_SIDE_DIAG_BLOCK_M 0.14f
#define MAZE_SIMPLE_SIDE_DIAG_SCORE_SCALE 0.88f
#define MAZE_MANHATTAN_GRID_ENABLE 1U
#define MAZE_MANHATTAN_SIDE_GUARD_BACKUP_ENABLE 0U
#define MAZE_SIMPLE_AXIS_DRIVE_ENABLE PROFILE_AXIS_DRIVE_ENABLE
#define MAZE_MANHATTAN_SIDE_PANIC_M PROFILE_MANHATTAN_SIDE_PANIC_M
#define MAZE_MANHATTAN_FRONT_DIAG_PANIC_M PROFILE_MANHATTAN_FRONT_DIAG_PANIC_M
#define MAZE_MANHATTAN_GUARD_FRONT_MIN_M 0.30f
#define MAZE_SIMPLE_POST_TURN_SIDE_IGNORE_MS PROFILE_POST_TURN_SIDE_IGNORE_MS
#define MAZE_SIMPLE_LOCK_ENABLE 1U
#define MAZE_SIMPLE_LOCK_YAW_DEADBAND_DEG 2.0f
#define MAZE_SIMPLE_LOCK_YAW_MAX_MS 900U
#define MAZE_SIMPLE_LOCK_YAW_KP_PWM_PER_DEG 12.0f
#define MAZE_SIMPLE_LOCK_YAW_PWM_MIN 230.0f
#define MAZE_SIMPLE_LOCK_YAW_PWM_MAX 290.0f
#define MAZE_SIMPLE_LOCK_SETTLE_MS 160U
#define MAZE_SIMPLE_LOCK_FRAME_MS 70U
#define MAZE_SIMPLE_LOCK_STABLE_FRAMES 3U
#define MAZE_SIMPLE_LOCK_DECISION_MAX_MS 1600U
#define MAZE_SIMPLE_FORWARD_SIDE_DANGER_M PROFILE_FORWARD_SIDE_DANGER_M
#define MAZE_SIMPLE_FORWARD_SIDE_PANIC_M PROFILE_FORWARD_SIDE_PANIC_M
#define MAZE_SIMPLE_FORWARD_DIAG_DANGER_M 0.220f
#define MAZE_SIMPLE_FORWARD_STEER_PWM PROFILE_FORWARD_STEER_PWM
#define MAZE_SIMPLE_FORWARD_SLOW_PWM PROFILE_FORWARD_SLOW_PWM
#define MAZE_SIMPLE_FORWARD_CENTER_ENABLE 1U
#define MAZE_SIMPLE_FORWARD_CENTER_MIN_WIDTH_M 0.42f
#define MAZE_SIMPLE_FORWARD_CENTER_MAX_WIDTH_M 1.35f
#define MAZE_SIMPLE_FORWARD_CENTER_DEADBAND_M 0.045f
#define MAZE_SIMPLE_FORWARD_CENTER_KP_PWM_PER_M 36.0f
#define MAZE_SIMPLE_FORWARD_CENTER_CORR_MAX_PWM 8.0f
#define MAZE_SIMPLE_KICK_SUPPRESS_SIDE_M 0.24f
#define MAZE_SIMPLE_POST_SIDE_DANGER_M 0.22f
#define MAZE_SIMPLE_POST_SIDE_PANIC_M 0.14f
#define MAZE_SIMPLE_POST_STEER_PWM 72.0f
#define MAZE_SIMPLE_POST_SLOW_PWM 230.0f
#define MAZE_SIMPLE_POST_ANTI_LEFT_PWM 18.0f
#define MAZE_CENTERLINE_ENABLE 0U
#define MAZE_CENTERLINE_DEADBAND_M 0.025f
#define MAZE_CENTERLINE_MAX_ERROR_M 0.090f
#define MAZE_CENTERLINE_SIDE_MIN_M 0.090f
#define MAZE_CENTERLINE_SIDE_MAX_M 0.420f
#define MAZE_CENTERLINE_WIDTH_MIN_M 0.260f
#define MAZE_CENTERLINE_WIDTH_MAX_M 0.700f
#define MAZE_CENTERLINE_FRONT_ROOM_M 0.360f
#define MAZE_CENTERLINE_PARALLEL_MAX_M 0.090f
#define MAZE_CENTERLINE_SHIFT_ANGLE_DEG 10.0f
#define MAZE_CENTERLINE_SHIFT_GAIN 0.95f
#define MAZE_CENTERLINE_SHIFT_MIN_M 0.060f
#define MAZE_CENTERLINE_SHIFT_MAX_M 0.210f
#define MAZE_CENTERLINE_BRAKE_MS 90U
#define MAZE_CENTERLINE_SETTLE_MS 130U
#define MAZE_CENTERLINE_TURN_MAX_MS 900U
#define MAZE_CENTERLINE_SHIFT_MAX_MS 1100U
#define MAZE_CENTERLINE_TURN_DEADBAND_DEG 1.4f
#define MAZE_CENTERLINE_TURN_KP_PWM_PER_DEG 13.0f
#define MAZE_CENTERLINE_TURN_PWM_MIN 270.0f
#define MAZE_CENTERLINE_TURN_PWM_MAX 315.0f
#define MAZE_CENTERLINE_SHIFT_PWM 235.0f
#define MAZE_CENTERLINE_RUN_ON_STOP_SCAN 0U
#define MAZE_CENTERLINE_RUN_AFTER_TURN 0U
#define MAZE_INTERSECTION_STOP_MS 260U
#define MAZE_FOLLOW_SETTLE_MS 420U
#define MAZE_TURN_NEAR_M 0.55f
#define MAZE_TURN_PWM_NEAR 300.0f
#define MAZE_MAP_PACE_ENABLE 0U
#define MAZE_MAP_RUN_MS 650U
#define MAZE_MAP_STOP_MS 220U
#define MAZE_MAP_FRONT_SAFE_M 0.52f
#define MAZE_INTERSECTION_CONFIRM_MS 220U
#define MAZE_INTERSECTION_FRONT_CLEAR_M 0.62f
#define MAZE_TURN_RULE_CONFIRM_MS 520U
#define MAZE_SYNC_KP_PWM_PER_PULSE 16.0f
#define MAZE_SYNC_CORR_PWM_MAX 34.0f

#define MAZE_FRONT_BLOCK_M 0.45f
#define MAZE_FRONT_BLOCK_RELEASE_M 0.52f
#define MAZE_FRONT_EMERGENCY_M 0.34f
#define MAZE_FRONT_PANIC_M 0.24f
#define MAZE_FRONT_USE_RIGHTFRONT_M 0.34f
#define MAZE_RIGHT_OPEN_M 0.55f
#define MAZE_RIGHT_OPEN_STRICT_M 0.62f
#define MAZE_RIGHT_FRONT_OPEN_STRICT_M 0.52f
#define MAZE_RIGHT_FRONT_OPEN_M 0.45f
#define MAZE_RIGHT_TURN_SIDE_CLEAR_M 0.46f
#define MAZE_RIGHT_DANGER_M 0.24f
#define MAZE_RIGHT_FRONT_DANGER_M 0.32f
#define MAZE_RIGHT_DANGER_BIAS_PWM 48.0f
#define MAZE_LEFT_DANGER_M 0.18f
#define MAZE_LEFT_FRONT_DANGER_M 0.32f
#define MAZE_SIMPLE_CLEARANCE_BIAS_PWM 42.0f
#define MAZE_LEFT_OPEN_M 0.45f
#define MAZE_FRONT_LOCK_MS 800U
#define MAZE_FRONT_LOCK_RELEASE_M 0.56f
#define MAZE_RIGHT_TARGET_M 0.33f
#define MAZE_RIGHT_CLEARANCE_BIAS_M 0.06f
#define MAZE_RIGHT_WALL_CAPTURE_M 0.50f
#define MAZE_RIGHT_WALL_LOST_M 0.78f
#define MAZE_STUCK_CMD_PWM 170.0f
#define MAZE_STUCK_PULSE_EPS 0.60f
#define MAZE_STUCK_DETECT_MS 350U
#define MAZE_ESCAPE_BACK_MS 320U
#define MAZE_ESCAPE_TURN_MS 420U
#define MAZE_LOOP_WINDOW_MS 4500U
#define MAZE_LOOP_MIN_PROGRESS_M 0.20f
#define MAZE_LOOP_MIN_TURN_RAD 4.20f
#define MAZE_FORCE_DIR_HOLD_MS 2400U
#define MAZE_UTURN_COOLDOWN_MS 3600U
#define MAZE_DEADEND_CONFIRM_MS 380U
#define MAZE_POST_UTURN_COMMIT_MS 520U
#define MAZE_UTURN_ESCAPE_SIDE_SCALE 0.85f
#define MAZE_RIGHT_MOTOR_REVERSED 1U
#define REMOTE_PWM_BASE 270.0f
#define REMOTE_PWM_TURN 235.0f
#define MANUAL_TURN_TARGET_LEFT_RAD 1.585f
#define MANUAL_TURN_TARGET_RIGHT_RAD 1.600f
#define MANUAL_TURN_FINISH_MARGIN_RAD 0.00f
#define MANUAL_TURN_MIN_MS 80U
#define MANUAL_TURN_COUNTER_BRAKE_MS 90U
#define MANUAL_TURN_COUNTER_BRAKE_PWM 260.0f
#define MANUAL_TURN_TIMEOUT_MS 1800U
#define MANUAL_TURN_SETTLE_MS 220U
#define ODOM_OGM_ENABLE 1U
#define OGM_RUNTIME_ENABLE 1U
#define OGM_MAPDBG_ENABLE 1U

#define PI_F 3.14159265358979323846f
#define ODOM_WHEEL_BASE_M 0.165f
#define ODOM_LEFT_PULSE_SIGN 1.0f
#define ODOM_RIGHT_PULSE_SIGN 1.0f
#define ODOM_FORWARD_SCALE 1.000f
#define ODOM_REVERSE_SCALE 1.000f
#define CALIB_PREPARE_MS 1000U
#define CALIB_STRAIGHT_MS 3200U
#define CALIB_PAUSE_MS 700U
#define CALIB_TURN_TIMEOUT_MS 9000U
#define CALIB_PWM_STRAIGHT MAZE_PWM_BASE
#define CALIB_PWM_TURN MAZE_PWM_TURN
#define CALIB_TRUE_DIST_M 0.60f
#define CALIB_TURN_TARGET_RAD (2.0f * PI_F)
#define CALIB_TURN_MIN_IMU_RAD 4.50f
#define CALIB_TURN_PULSE_RATIO_MIN 0.30f
#define CALIB_STRAIGHT_YAW_KP 120.0f
#define CALIB_STRAIGHT_YAW_CORR_MAX 70.0f
#define IMU_STALE_MS 500U
#define IMU_REINIT_RETRY_MS 2000U
#define CALIB_TURN_STALL_MOTION_RAD 0.0015f
#define CALIB_TURN_REKICK_MS 260U
#define CALIB_TURN_STALL_ABORT_MS 1500U

#define OGM_RESOLUTION_M 0.05f
#define OGM_WIDTH 80
#define OGM_HEIGHT 96
#define OGM_MAP_CENTER_X_M 0.0f
#define OGM_MAP_CENTER_Y_M (-1.65f)
#define OGM_CELL_FREE_DELTA (-3)
#define OGM_CELL_OCC_DELTA 9
#define OGM_CELL_MIN (-100)
#define OGM_CELL_MAX 100
#define OGM_UPDATE_INTERVAL_MS 55U
#define OGM_STOP_SCAN_INTERVAL_MS 220U
#define OGM_STOP_SCAN_SETTLE_MS 260U
#define OGM_MIN_UPDATE_DS_M 0.012f
#define OGM_MIN_UPDATE_DTHETA_RAD 0.070f
#define OGM_MAPPING_MAX_DTHETA_RAD 0.090f
#define OGM_MAX_ABS_PITCH_DEG 12.0f
#define OGM_MAX_ABS_ROLL_DEG 12.0f
#define OGM_STOP_CMD_EPS_PWM 1.0f
#define OGM_RANGE_MIN_M 0.10f
#define OGM_RANGE_STALE_MS 260U
#define OGM_FILTER_ALPHA_STABLE 0.22f
#define OGM_FILTER_ALPHA_JUMP 0.08f
#define OGM_FILTER_JUMP_M 0.25f
#define OGM_FILTER_HARD_JUMP_M 0.85f
#define OGM_OCC_NEIGHBOR_DELTA 1
#define OGM_DISPLAY_OCC_EVIDENCE_MIN 6
#define OGM_DISPLAY_OCC_MIN 9
#define OGM_DISPLAY_OCC_STRONG 30
#define OGM_DISPLAY_FREE_MIN (-36)
#define OGM_DISPLAY_FREE_STRONG (-72)
#define OGM_DISPLAY_WEAK_OCC_REQUIRE_FREE 1U
#define OGM_DISPLAY_OCC_SURFACE_ONLY 1U
#define OGM_DISPLAY_DIAGONAL_BRIDGE_ENABLE 0U
#define OGM_NAV_OCC_MIN 18
#define OGM_NAV_FREE_MIN (-36)
#define OGM_NAV_BRIDGE_MAX_GAP 2U
#define OGM_NAV_MIN_WALL_RUN 4U
#define OGM_NAV_PRUNE_PASSES 2U
#define OGM_NAV_DIRECT_FROM_MAP_ENABLE 1U
#define OGM_NAV_DIRECT_CLEAN_ENABLE 1U
#define OGM_NAV_DIRECT_MANHATTAN_ENABLE 0U
#define OGM_NAV_NORMALIZE_ENABLE 1U
#define OGM_NAV_THICKNESS_MIN 1U
#define OGM_NAV_THICKNESS_MAX 1U
#define OGM_NAV_THICKNESS_SAMPLE_MAX 6U
#define OGM_NAV_AXIS_BAND_GAP 2U
#define OGM_NAV_AXIS_BAND_MAX_SPAN 12U
#define OGM_NAV_AXIS_BAND_MIN_POINTS 4U
#define OGM_NAV_CONSTRAIN_OCC_RADIUS 2
#define OGM_VIEW_MARGIN_CELLS 3
#define OGM_IMU_HEADING_SIGN 1.0f
#define OGM_GRID_HEADING_ENABLE 1U
#define OGM_MAP_REVERSE_BACKUP_ENABLE 1U
#define OGM_MAP_RIGHT_FRONT_ENABLE 1U
#define OGM_DIAGONAL_MAP_MIN_M 0.32f
#define OGM_DIAGONAL_FREE_DELTA 0
#define OGM_DIAGONAL_OCC_DELTA 1
#define OGM_SCAN_RANGE_MIN_M 0.11f
#define OGM_SCAN_OCC_MAX_M 1.25f
#define OGM_SCAN_FREE_MAX_M 1.60f
#define OGM_SCAN_BIN_DEG (360.0f / (float)LIDAR_SCAN_BIN_COUNT)
#define OGM_LIDAR_YAW_OFFSET_DEG 0.0f
#define OGM_SCAN_STALE_MS 350U
#define OGM_SCAN_POSE_DELAY_MS 50U
#define OGM_SCAN_POSE_MAX_AGE_MS 500U
#define OGM_SCAN_MIN_VALID_BINS 8U
#define OGM_SCAN_FREE_DELTA (-2)
#define OGM_SCAN_OCC_DELTA 3
#define OGM_SCAN_BACKUP_FREE_DELTA (-1)
#define OGM_SCAN_BACKUP_OCC_DELTA 1
#define OGM_SCAN_OCC_NEIGHBOR_DELTA 0
#define OGM_FREE_PROTECT_OCC_MIN OGM_NAV_OCC_MIN
#define OGM_SCAN_SUPPORT_WINDOW 2U
#define OGM_SCAN_NEAR_SUPPORT_MAX_M 0.90f
#define OGM_SCAN_NEAR_SUPPORT_MIN_COUNT 2U
#define OGM_SCAN_SUPPORT_MIN_COUNT 3U
#define OGM_SCAN_SUPPORT_DIFF_BASE_M 0.18f
#define OGM_SCAN_SUPPORT_DIFF_RATIO 0.28f
#define OGM_SCAN_SKIP_NONE 0U
#define OGM_SCAN_SKIP_NO_SCAN 1U
#define OGM_SCAN_SKIP_DUPLICATE 2U
#define OGM_SCAN_SKIP_STALE 3U
#define OGM_SCAN_SKIP_COPY_EMPTY 4U
#define OGM_SCAN_SKIP_TOO_FEW_BINS 5U
#define OGM_SCAN_SKIP_IDLE 6U
#define OGM_SCAN_SKIP_IMU 7U
#define OGM_SCAN_SKIP_STOP_WAIT 8U
#define OGM_SCAN_SKIP_INTERVAL 9U
#define OGM_SCAN_SKIP_SMALL_DELTA 10U
#define OGM_SCAN_SKIP_STATE 11U
#define OGM_SCAN_SKIP_TURNING 12U
#define OGM_POSE_HISTORY_LEN 16U
#define OGM_LIDAR_ORIGIN_X_M (-0.050f)
#define OGM_LIDAR_ORIGIN_Y_M 0.000f
#define OGM_WALL_HEADING_CORRECT_ENABLE 0U
#define OGM_WALL_HEADING_MIN_SIDE_M 0.12f
#define OGM_WALL_HEADING_MAX_SIDE_M 0.85f
#define OGM_WALL_HEADING_MAX_ABS_X_M 0.55f
#define OGM_WALL_HEADING_MIN_POINTS 4U
#define OGM_WALL_HEADING_GAIN 0.04f
#define OGM_WALL_HEADING_MAX_STEP_RAD 0.015f
#define OGM_START_REAR_WALL_ENABLE 1U
#define OGM_START_REAR_WALL_BACK_M 0.120f
#define OGM_START_REAR_WALL_HALF_WIDTH_M 0.140f
#define OGM_START_REAR_WALL_DELTA 18
#define OGM_FIXED_MAP_FRAME_ENABLE 1U
#define OGM_MAZE_LENGTH_M (((float)OGM_WIDTH) * OGM_RESOLUTION_M)
#define OGM_MAZE_WIDTH_M (((float)OGM_HEIGHT) * OGM_RESOLUTION_M)
#define OGM_MAZE_ENTRY_OFFSET_M 0.20f
#define OGM_MAP_START_X_M ((-0.5f * OGM_MAZE_LENGTH_M) + OGM_MAZE_ENTRY_OFFSET_M)
#define OGM_MAP_START_Y_M (-0.65f)
#define OGM_POSE_CORRECT_ENABLE 1U
#define OGM_POSE_CORRECT_MIN_RANGE_M 0.12f
#define OGM_POSE_CORRECT_MAX_RANGE_M 1.20f
#define OGM_POSE_CORRECT_FRESH_MS 180U
#define OGM_POSE_CORRECT_SEARCH_CELLS 3
#define OGM_POSE_CORRECT_OCC_MIN 12
#define OGM_POSE_CORRECT_GAIN 0.12f
#define OGM_POSE_CORRECT_MAX_STEP_M 0.012f
#define MAZE_TOPO_ENABLE 1U
#define MAZE_FIXED_FAST_EXIT_ENABLE 1U
#define MAZE_TOPO_MAX_NODES 64U
#define MAZE_TOPO_MERGE_DIST_M 0.18f
#define MAZE_TOPO_REVISIT_MERGE_DIST_M 0.42f
#define MAZE_TOPO_FAIL_NEAR_M 0.38f
#define MAZE_TOPO_FAIL_ADJACENT_DIRS_ENABLE 1U
#define MAZE_TOPO_EXHAUSTED_BACKUP_ENABLE 1U
#define MAZE_TOPO_PENDING_BRANCH_MS 7000U
#define MAZE_RETURN_HOME_RADIUS_M 0.10f
#define MAZE_RETURN_FINAL_M 0.55f
#define MAZE_RETURN_FINAL_MIN_M 0.18f
#define MAZE_RETURN_FINAL_MAX_M 1.20f
#define MAZE_RETURN_FINAL_MARGIN_M 0.03f
#define MAZE_RETURN_FRONT_STOP_M 0.22f
#define MAZE_RETURN_FINAL_MS 3200U
#define MAZE_RETURN_ASTAR_HOME_RADIUS_M 0.25f
#define MAZE_RETURN_ASTAR_FRONT_RADIUS_M 0.85f
#define MAZE_RETURN_ASTAR_FINAL_RADIUS_M 0.55f
#define MAZE_RETURN_ASTAR_TIMEOUT_RADIUS_M 0.85f
#define MAZE_RETURN_ENABLE 1U
#define MAZE_TOPO_OPEN_FRONT 0x01U
#define MAZE_TOPO_OPEN_RIGHT 0x02U
#define MAZE_TOPO_OPEN_BACK  0x04U
#define MAZE_TOPO_OPEN_LEFT  0x08U
#define MAZE_TOPO_WORLD_N 0x01U
#define MAZE_TOPO_WORLD_E 0x02U
#define MAZE_TOPO_WORLD_S 0x04U
#define MAZE_TOPO_WORLD_W 0x08U
#define MAZE_TOPO_DECISION_NONE 0U
#define MAZE_TOPO_DECISION_STRAIGHT 1U
#define MAZE_TOPO_DECISION_RIGHT 2U
#define MAZE_TOPO_DECISION_LEFT 3U
#define MAZE_TOPO_DECISION_BACKUP 4U
#define MAZE_SELFDBG_TOPO_HOLD_MS 3000U
#define MAZE_TOPO_KIND_PASS 0U
#define MAZE_TOPO_KIND_JUNCTION 1U
#define MAZE_TOPO_KIND_CORNER 2U
#define MAZE_TOPO_KIND_DEADEND 3U
#define MAZE_ASTAR_ENABLE 1U
#define MAZE_ASTAR_MAX_PATH_CELLS 256U
#define MAZE_ASTAR_FLAG_OPEN 0x01U
#define MAZE_ASTAR_FLAG_CLOSED 0x02U
#define MAZE_ASTAR_PARENT_NONE 0xFFFFU
#define MAZE_ASTAR_COST_INF 0xFFFFU
#define MAZE_ASTAR_STEP_COST 10U
#define MAZE_ASTAR_TURN_COST 8U
#define MAZE_ASTAR_NEAREST_FREE_RADIUS 5
#define MAZE_ASTAR_CENTER_BIAS_ENABLE 1U
#define MAZE_ASTAR_CENTER_BIAS_RADIUS_CELLS 2
#define MAZE_ASTAR_WALL_ADJ_COST 28U
#define MAZE_ASTAR_WALL_NEAR_COST 9U
#define MAZE_ASTAR_TURN_GATE_CELLS 5
#define MAZE_ASTAR_DBG_LOOKAHEAD_CELLS 4U
#define MAZE_ASTAR_DYNAMIC_GOAL_CELLS 3U
#define MAZE_ASTAR_REPLAN_DEVIATION_CELLS 6U
#define MAZE_ASTAR_REPLAN_COOLDOWN_MS 1800U
#define MAZE_ASTAR_REPLAN_BLOCK_MS 1200U
#define MAZE_ASTAR_POST_ALIGN_REPLAN_GRACE_MS 1600U
#define MAZE_ASTAR_POST_ALIGN_FRONT_MIN_M 0.26f
#define MAZE_ASTAR_BLOCK_MAX_CELLS 12U
#define MAZE_ASTAR_BLOCK_AHEAD_CELLS 3U
#define MAZE_ASTAR_MAX_SEGMENTS 96U
#define MAZE_ASTAR_SEGMENT_ARRIVE_CELLS 2U
#define MAZE_ASTAR_SEGMENT_LATERAL_CELLS 3U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
char Bluetooth_buf[512];

int Encoder_A, Encoder_B;
static uint16_t lastCount_A, lastCount_B;

pid_t pid_A, pid_B;
const float pid_k_A[] = {5.4f, 0.08f, 0.35f};
const float pid_k_B[] = {5.4f, 0.08f, 0.35f};

float fdb_A;
float fdb_B;
float set_A = 1;
float set_B = 1;
static float pid_cmd_acc_A = 0.0f;
static float pid_cmd_acc_B = 0.0f;
static float pid_fdb_acc_A = 0.0f;
static float pid_fdb_acc_B = 0.0f;
static uint16_t pid_window_samples = 0U;
static float pid_corr_pwm_A = 0.0f;
static float pid_corr_pwm_B = 0.0f;
static int8_t pid_last_sign_A = 0;
static int8_t pid_last_sign_B = 0;
static uint32_t pid_last_move_tick = 0U;
static float drive_dbg_left_cmd_pwm = 0.0f;
static float drive_dbg_right_cmd_pwm = 0.0f;
static float drive_dbg_load_left_pwm = 0.0f;
static float drive_dbg_load_right_pwm = 0.0f;
static float drive_dbg_yaw_err_deg = 0.0f;
static float drive_dbg_yaw_corr_pwm = 0.0f;
static float drive_dbg_straight_trim_pwm = 0.0f;
static int8_t drive_dbg_straight_sign = 0;

float pitch, roll, yaw;
short gyrox, gyroy, gyroz;
short accx, accy, accz;

volatile int32_t pulse_sum = 0;
volatile int32_t pulse_sum_A = 0;
volatile int32_t pulse_sum_B = 0;
float fordist = 0.0f;
float rightdist = 0.0f;
float rightfrontdist = 0.0f;
float rightreardist = 0.0f;
float reardist = 0.0f;
float leftreardist = 0.0f;
float leftdist = 0.0f;
float leftfrontdist = 0.0f;
float speed_mps = 0.0f;
float left_speed_mps = 0.0f;
float right_speed_mps = 0.0f;

static float fordist_filtered = 0.0f;
static float rightdist_filtered = 0.0f;
static float rightfrontdist_filtered = 0.0f;
static float rightreardist_filtered = 0.0f;
static float reardist_filtered = 0.0f;
static float leftreardist_filtered = 0.0f;
static float leftdist_filtered = 0.0f;
static float leftfrontdist_filtered = 0.0f;
static uint32_t fordist_last_valid_tick = 0U;
static uint32_t rightdist_last_valid_tick = 0U;
static uint32_t rightfrontdist_last_valid_tick = 0U;
static uint32_t rightreardist_last_valid_tick = 0U;
static uint32_t reardist_last_valid_tick = 0U;
static uint32_t leftreardist_last_valid_tick = 0U;
static uint32_t leftdist_last_valid_tick = 0U;
static uint32_t leftfrontdist_last_valid_tick = 0U;
static uint32_t lidar_fast_update_tick = 0U;
static uint16_t lidar_scan_dump_bins[LIDAR_SCAN_BIN_COUNT];

static maze_state_t maze_state = MAZE_STATE_FOLLOW;
static maze_simple_state_t maze_simple_state = MAZE_SIMPLE_FOLLOW;
#if ODOM_OGM_ENABLE
static float maze_simple_start_x_m = 0.0f;
static float maze_simple_start_y_m = 0.0f;
static uint8_t maze_simple_start_valid = 0U;
static float maze_simple_backup_start_x_m = 0.0f;
static float maze_simple_backup_start_y_m = 0.0f;
static uint8_t maze_simple_backup_start_valid = 0U;
static float maze_simple_center_start_x_m = 0.0f;
static float maze_simple_center_start_y_m = 0.0f;
static uint8_t maze_simple_center_start_valid = 0U;
static float maze_simple_pre_turn_start_x_m = 0.0f;
static float maze_simple_pre_turn_start_y_m = 0.0f;
static uint8_t maze_simple_pre_turn_start_valid = 0U;
#endif
static int8_t maze_simple_prefer_turn_dir = 0;
static maze_simple_state_t maze_simple_pending_turn_state = MAZE_SIMPLE_FOLLOW;
static uint8_t maze_simple_pending_turn_valid = 0U;
static int8_t maze_simple_center_turn_dir = 0;
static uint8_t maze_simple_center_turn_topo_openings = 0U;
static maze_simple_state_t maze_simple_yaw_recover_resume_state = MAZE_SIMPLE_FOLLOW;
static uint8_t maze_simple_yaw_recover_phase = 0U; /* 0:brake, 1:turn, 2:settle */
static uint32_t maze_simple_yaw_recover_tick = 0U;
static uint32_t maze_simple_yaw_guard_tick = 0U;
static float maze_simple_yaw_recover_target_rad = 0.0f;
static uint8_t maze_simple_turn_from_backup = 0U;
static uint8_t maze_simple_turn_back_step = 0U;
static int8_t maze_simple_post_turn_brake_dir = 0;
static float maze_simple_backup_ref_yaw_rad = 0.0f;
static uint8_t maze_simple_backup_ref_yaw_valid = 0U;
static uint8_t maze_simple_backup_align_phase = 0U; /* 0:idle, 1:turning, 2:braking */
static uint32_t maze_simple_backup_align_tick = 0U;
static uint8_t selfdbg_backup_align_phase = 0U;
static uint32_t selfdbg_backup_align_age_ms = 0U;
static float selfdbg_backup_align_err_deg = 0.0f;
static float selfdbg_backup_align_ref_deg = 0.0f;
static float selfdbg_backup_align_pwm = 0.0f;
static uint8_t maze_simple_lock_phase = 0U; /* 0:init, 1:yaw, 2:settle, 3:verify */
static uint32_t maze_simple_lock_tick = 0U;
static uint32_t maze_simple_lock_sample_tick = 0U;
static uint8_t maze_simple_lock_stable_count = 0U;
static uint16_t maze_simple_lock_last_signature = 0xFFFFU;
static float maze_simple_lock_target_yaw_rad = 0.0f;
static float maze_simple_lock_dbg_yaw_err_deg = 0.0f;
static uint8_t maze_simple_cardinal_ref_valid = 0U;
static float maze_simple_cardinal_ref_yaw_rad = 0.0f;
static uint8_t maze_simple_post_lock_phase = 0U; /* 0:init, 1:brake, 2:yaw, 3:settle */
static uint32_t maze_simple_post_lock_tick = 0U;
static float maze_simple_post_lock_target_yaw_rad = 0.0f;
static uint8_t maze_simple_start_align_phase = 0U; /* 0:scan, 1:turn, 2:settle */
static uint32_t maze_simple_start_align_tick = 0U;
static float maze_simple_start_align_target_yaw_rad = 0.0f;
static uint8_t maze_simple_start_align_preserve_map = 0U;
static uint8_t maze_simple_start_align_ref_valid = 0U;
static float maze_simple_start_align_ref_yaw_rad = 0.0f;
static float maze_simple_start_align_ref_map_heading_rad = 0.0f;
static maze_centerline_phase_t maze_centerline_phase = MAZE_CENTERLINE_IDLE;
static uint8_t maze_centerline_done_this_stop = 0U;
static uint8_t maze_centerline_done_after_turn = 0U;
static int8_t maze_centerline_dir = 0; /* +1: shift left, -1: shift right */
static float maze_centerline_ref_yaw_rad = 0.0f;
static float maze_centerline_target_yaw_rad = 0.0f;
static float maze_centerline_start_x_m = 0.0f;
static float maze_centerline_start_y_m = 0.0f;
static float maze_centerline_error_m = 0.0f;
static float maze_centerline_shift_target_m = 0.0f;
static float maze_centerline_dbg_yaw_err_deg = 0.0f;
static uint32_t maze_centerline_phase_tick = 0U;
static uint32_t maze_state_tick = 0U;
static uint32_t maze_last_decide_tick = 0U;
static uint32_t maze_drive_kick_tick = 0U;
static float maze_prev_left_cmd = 0.0f;
static float maze_prev_right_cmd = 0.0f;
static uint8_t maze_have_right_wall = 0U;
static float maze_turn_start_theta_rad = 0.0f;
static uint8_t maze_turn_start_valid = 0U;
static float maze_turn_start_yaw_rad = 0.0f;
static uint8_t maze_turn_start_yaw_valid = 0U;
static uint8_t maze_simple_turn_timeout_failed = 0U;
static float maze_simple_turn_last_turned_rad = 0.0f;
static float maze_simple_turn_last_signed_rad = 0.0f;
static float maze_simple_turn_last_target_rad = 0.0f;
static uint8_t maze_simple_turn_last_yaw_gate = 0U;
static maze_state_t maze_pending_turn_state = MAZE_STATE_FOLLOW;
static uint8_t maze_pending_turn_valid = 0U;
static uint32_t maze_intersection_candidate_tick = 0U;
static uint8_t maze_rule_turn_candidate_dir = 0U; /* 0:none, 1:right, 2:left, 3:deadend */
static uint32_t maze_rule_turn_candidate_tick = 0U;
static uint8_t maze_force_dir = 0U; /* 0:none, 1:right, 2:left, 3:back */
static uint32_t maze_force_dir_until_tick = 0U;
static int8_t maze_topo_pending_branch_dir = 0;
static uint32_t maze_topo_pending_branch_until_tick = 0U;
static uint8_t maze_topo_commit_active = 0U;
static uint16_t maze_topo_commit_idx = MAZE_TOPO_MAX_NODES;
static uint8_t maze_topo_commit_decision = MAZE_TOPO_DECISION_NONE;
static uint8_t maze_topo_commit_world_bit = 0U;
static uint8_t maze_topo_commit_openings = 0U;
static int8_t maze_topo_commit_heading = 0;
static uint32_t maze_topo_commit_tick = 0U;
static float maze_topo_commit_x_m = 0.0f;
static float maze_topo_commit_y_m = 0.0f;
static uint8_t maze_topo_arrival_back_world_bit = 0U;
static uint16_t maze_topo_arrival_source_idx = MAZE_TOPO_MAX_NODES;
static uint8_t maze_map_pause_active = 0U;
static uint32_t maze_map_pace_tick = 0U;
static uint32_t maze_uturn_block_until_tick = 0U;
static uint32_t maze_post_uturn_commit_until_tick = 0U;
static uint32_t maze_deadend_confirm_tick = 0U;
static uint8_t selfdbg_side_open_suppressed = 0U;
static uint8_t selfdbg_backup_stop_pending = 0U;
static int8_t selfdbg_backup_turn_dir = 0;
static uint8_t selfdbg_backup_turn_extra_active = 0U;
static int8_t selfdbg_topo_decision_dir = 0;
static int8_t selfdbg_topo_pending_dir = 0;
static uint8_t selfdbg_topo_block_reason = 0U;
static uint32_t selfdbg_topo_hold_until_tick = 0U;
static uint8_t lidar_ready = 0U;
static uint8_t lidar_ready_stable_count = 0U;
static uint32_t lidar_last_nonzero_tick = 0U;
static volatile uint32_t control_tick_pending = 0U;
static uint8_t imu_data_valid = 0U;
static uint32_t imu_last_ok_tick = 0U;
static uint32_t imu_last_reinit_tick = 0U;
static uint8_t control_prev_maze_enable = 0U;
static uint8_t manual_turn_active = 0U; /* 0:none, 1:right, 2:left */
static uint8_t manual_turn_dir = 0U;
static uint8_t manual_turn_brake_dir = 0U; /* 1:right, 2:left */
static uint32_t manual_turn_start_tick = 0U;
static uint32_t manual_turn_brake_until_tick = 0U;
static uint32_t manual_turn_settle_until_tick = 0U;
static float manual_turn_start_theta_rad = 0.0f;
static float manual_turn_start_yaw_rad = 0.0f;
static uint8_t manual_turn_start_yaw_valid = 0U;
static uint8_t calib_start_armed = 0U;
static uint8_t calib_wait_imu_reported = 0U;
static calib_state_t calib_state = CALIB_STATE_IDLE;
static uint32_t calib_state_tick = 0U;

#if ODOM_OGM_ENABLE
static float odom_x_m = 0.0f;
static float odom_y_m = 0.0f;
static float odom_theta_rad = 0.0f;
volatile int32_t odom_pulse_accum_left = 0;
volatile int32_t odom_pulse_accum_right = 0;
static float calib_start_x_m = 0.0f;
static float calib_start_y_m = 0.0f;
static float calib_straight_odom_m = 0.0f;
static float calib_straight_ref_yaw_rad = 0.0f;
static float calib_turn_accum_rad = 0.0f;
static float calib_turn_imu_accum_rad = 0.0f;
static float calib_last_theta_rad = 0.0f;
static float calib_last_yaw_rad = 0.0f;
static float calib_turn_abs_pulse_left = 0.0f;
static float calib_turn_abs_pulse_right = 0.0f;
static uint32_t calib_turn_stall_tick = 0U;
static uint32_t calib_turn_last_rekick_tick = 0U;

static int8_t ogm_grid[OGM_HEIGHT][OGM_WIDTH];
static float ogm_fordist_f = 0.0f;
static float ogm_rightdist_f = 0.0f;
static float ogm_rightfrontdist_f = 0.0f;
static float ogm_rightreardist_f = 0.0f;
static float ogm_reardist_f = 0.0f;
static float ogm_leftreardist_f = 0.0f;
static float ogm_leftdist_f = 0.0f;
static float ogm_leftfrontdist_f = 0.0f;
static float ogm_map_x_m = 0.0f;
static float ogm_map_y_m = 0.0f;
static uint8_t ogm_map_pose_valid = 0U;
static uint8_t ogm_grid_heading_valid = 0U;
static float ogm_grid_heading_rad = 0.0f;
static uint8_t ogm_heading_ref_valid = 0U;
static float ogm_heading_ref_yaw_rad = 0.0f;
static float ogm_heading_ref_theta_rad = 0.0f;
static char ogm_display_map[OGM_HEIGHT][OGM_WIDTH];
static char ogm_display_src[OGM_HEIGHT][OGM_WIDTH];
static char ogm_nav_center_map[OGM_HEIGHT][OGM_WIDTH];
static char ogm_nav_source_map[OGM_HEIGHT][OGM_WIDTH];
static uint8_t ogm_nav_row_support[OGM_HEIGHT];
static uint8_t ogm_nav_col_support[OGM_WIDTH];
static uint8_t ogm_nav_axis_support[OGM_WIDTH > OGM_HEIGHT ? OGM_WIDTH : OGM_HEIGHT];
static char ogm_map_row[OGM_WIDTH + 2];
static ogm_pose_sample_t ogm_pose_history[OGM_POSE_HISTORY_LEN];
static uint8_t ogm_pose_history_index = 0U;
static uint16_t ogm_scan_bins_mm[LIDAR_SCAN_BIN_COUNT];
static uint16_t ogm_scan_filtered_bins_mm[LIDAR_SCAN_BIN_COUNT];
static uint32_t ogm_last_scan_tick_mapped = 0U;
static uint16_t ogm_dbg_scan_raw_valid_bins = 0U;
static uint16_t ogm_dbg_scan_filtered_bins = 0U;
static uint16_t ogm_dbg_scan_far_bins = 0U;
static uint16_t ogm_dbg_scan_masked_count = 0U;
static uint32_t ogm_dbg_scan_tick = 0U;
static uint32_t ogm_dbg_scan_update_count = 0U;
static uint8_t ogm_dbg_scan_used = 0U;
static uint8_t ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_NO_SCAN;
#if MAZE_TOPO_ENABLE
static maze_topo_node_t maze_topo_nodes[MAZE_TOPO_MAX_NODES];
static uint8_t maze_topo_failed_world[MAZE_TOPO_MAX_NODES];
static uint16_t maze_topo_count = 0U;
static uint16_t maze_topo_next_id = 1U;
static uint16_t maze_topo_write_index = 0U;
static uint16_t maze_topo_path_nodes[MAZE_TOPO_MAX_NODES];
static uint8_t maze_topo_path_decisions[MAZE_TOPO_MAX_NODES];
static uint8_t maze_topo_path_world_bits[MAZE_TOPO_MAX_NODES];
static uint16_t maze_topo_path_count = 0U;
static uint8_t maze_return_active = 0U;
static uint8_t maze_return_done = 0U;
static uint16_t maze_return_remaining = 0U;
static uint8_t maze_return_astar_active = 0U;
static uint8_t maze_return_astar_fallback = 0U;
static maze_return_phase_t maze_return_phase = MAZE_RETURN_PHASE_IDLE;
static maze_return_plan_t maze_return_plan = MAZE_RETURN_PLAN_NONE;
static maze_return_action_t maze_return_last_action = MAZE_RETURN_ACTION_NONE;
static uint8_t maze_return_target_dir = 4U;
static uint8_t maze_return_latched_decision = MAZE_TOPO_DECISION_NONE;
static uint32_t maze_return_phase_tick = 0U;
static uint32_t maze_return_align_done_tick = 0U;
static uint8_t maze_return_home_pending = 0U;
static uint32_t maze_return_home_tick = 0U;
static uint8_t maze_return_home_drive_started = 0U;
static uint32_t maze_return_home_drive_tick = 0U;
static float maze_return_home_start_x_m = 0.0f;
static float maze_return_home_start_y_m = 0.0f;
static float maze_return_final_target_m = MAZE_RETURN_FINAL_M;
static uint8_t maze_return_align_pending = 0U;
#if MAZE_ASTAR_ENABLE
static uint8_t maze_astar_flags[OGM_WIDTH * OGM_HEIGHT];
static uint16_t maze_astar_g_cost[OGM_WIDTH * OGM_HEIGHT];
static uint16_t maze_astar_parent[OGM_WIDTH * OGM_HEIGHT];
static uint16_t maze_astar_path_cells[MAZE_ASTAR_MAX_PATH_CELLS];
static uint16_t maze_astar_path_count = 0U;
static uint8_t maze_astar_path_valid = 0U;
static uint8_t maze_astar_plan_decisions[MAZE_TOPO_MAX_NODES];
static uint16_t maze_astar_plan_turn_cells[MAZE_TOPO_MAX_NODES];
static uint16_t maze_astar_plan_count = 0U;
static float maze_astar_final_segment_m = 0.0f;
static uint8_t maze_astar_last_fail_pose = 0U;
static uint16_t maze_astar_return_progress_i = 0U;
static uint8_t maze_astar_last_dynamic_decision = MAZE_TOPO_DECISION_NONE;
static uint8_t maze_astar_last_dynamic_goal = 0U;
static uint32_t maze_astar_return_replan_tick = 0U;
static uint16_t maze_astar_temp_block_cells[MAZE_ASTAR_BLOCK_MAX_CELLS];
static uint8_t maze_astar_temp_block_count = 0U;
typedef struct
{
  uint16_t start_i;
  uint16_t end_i;
  uint8_t dir;
} maze_astar_segment_t;
static maze_astar_segment_t maze_astar_segments[MAZE_ASTAR_MAX_SEGMENTS];
static uint16_t maze_astar_segment_count = 0U;
static uint16_t maze_astar_active_segment_i = 0U;
static uint8_t maze_astar_segment_turn_pending = 0U;
typedef struct
{
  int cur_x;
  int cur_y;
  uint16_t progress_i;
  uint16_t nearest_i;
  int nearest_x;
  int nearest_y;
  uint16_t nearest_d2;
  uint16_t target_i;
  int target_x;
  int target_y;
  uint16_t target_d2;
  uint16_t gate_i;
  int gate_x;
  int gate_y;
  uint16_t gate_d2;
  uint8_t gate_valid;
  uint8_t gate_ready;
  uint8_t astar_heading;
  uint8_t next_dir;
  uint8_t next_decision;
} maze_astar_debug_t;
#endif
#endif
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ReadSensor(void);
void Control(void);
void Send_Data_EverySecond(void);
void RefreshLidarDistances(uint32_t now_tick);
void MazeControlTask(uint32_t now_tick);
static void MazeDrive(float left_pwm, float right_pwm, uint32_t now_tick);
static void MazeSimpleControlTask(uint32_t now_tick);
static void maze_simple_enter(maze_simple_state_t state, uint32_t now_tick);
static void maze_simple_schedule_turn(maze_simple_state_t turn_state, uint32_t now_tick);
static void maze_simple_turn_drive(maze_simple_state_t state, uint32_t now_tick);
static void maze_simple_drive_straight(uint32_t now_tick);
static void maze_simple_drive_stop_brake(uint32_t now_tick);
static float maze_simple_nearest_cardinal_yaw_rad(float yaw_rad);
static uint8_t maze_simple_side_entry_quality_ok(uint8_t decision, uint32_t now_tick);
static uint8_t maze_simple_side_entry_panic(uint8_t decision, uint32_t now_tick);
static uint8_t maze_simple_revisit_side_turn_ready(uint8_t decision, uint32_t now_tick);
static uint8_t maze_simple_yaw_guard_check(uint32_t now_tick);
static uint8_t maze_simple_yaw_recover_step(uint32_t now_tick);
static uint8_t maze_simple_turn_clear_for_state(maze_simple_state_t state, uint32_t now_tick);
static uint8_t maze_simple_state_allows_mapping(void);
static void maze_simple_lock_reset(void);
static void maze_centerline_reset(void);
static uint8_t maze_centerline_step(uint32_t now_tick);
static uint8_t ManualTurnTask(uint32_t now_tick);
static void maze_enter_state(maze_state_t state, uint32_t now_tick);
static uint8_t lidar_is_ready_for_runtime(void);
static uint8_t imu_is_ready_for_runtime(void);
static uint8_t lidar_ranges_all_zero(void);
static void Lidar_HandleReinitRequest(void);
static void OGM_HandleCommand(void);
static void MazeTopo_HandleCommand(void);
static void Lidar_HandleScanDumpCommand(void);
static void Calibration_HandleCommand(void);
static uint8_t Calibration_Task(uint32_t now_tick);
static void Calibration_Print(const char *msg);
#if ODOM_OGM_ENABLE
static void Calibration_Begin(uint32_t now_tick);
static void Calibration_Abort(const char *reason);
static void OGM_NoteMazeTurnComplete(maze_simple_state_t turn_state);
static void OGM_SeedStartRearWall(void);
#if MAZE_TOPO_ENABLE
static void MazeTopo_Reset(void);
static void MazeTopo_RecordDecision(uint8_t openings, uint8_t decision, uint32_t now_tick);
static void MazeTopo_CommitPendingDecision(uint8_t decision, uint32_t now_tick);
static uint8_t MazeTopo_AbortPendingSideIfBlocked(uint32_t now_tick);
static void MazeTopo_CancelPendingDecision(void);
static void MazeTopo_MarkFailedWorldNear(uint16_t source_idx, uint8_t world_bit);
static void MazeTopo_ClearFailedWorldNear(uint16_t source_idx, uint8_t world_bit);
static uint8_t MazeTopo_FailedMaskForWorldBit(uint8_t world_bit);
static void MazeTopo_PrintCommitDebug(uint16_t current_idx,
                                      float ds_m,
                                      uint8_t ok,
                                      char reason,
                                      uint32_t now_tick);
static uint8_t MazeTopo_SelectExploreDecision(uint8_t openings, uint32_t now_tick, uint8_t from_backup);
static uint8_t MazeTopo_CurrentNodeRevisited(void);
static void MazeTopo_PrintDecisionDebug(uint8_t openings,
                                        uint8_t decision,
                                        uint8_t from_backup,
                                        int8_t pending_dir,
                                        uint32_t now_tick);
static void MazeTopo_StartReturn(uint32_t now_tick);
static maze_return_start_result_t MazeTopo_StartReturnAfterAlign(uint32_t now_tick);
static void MazeTopo_AbortPendingReturnAlign(uint32_t now_tick);
static void MazeTopo_ClearReturn(uint32_t now_tick);
static void MazeTopo_StopReturn(uint8_t done, uint32_t now_tick);
static int8_t MazeTopo_HeadingQuadrant(void);
static uint8_t MazeTopo_PathReturnDecision(uint16_t path_i);
static void MazeReturn_SetPhase(maze_return_phase_t phase, uint32_t now_tick);
static void MazeReturn_SetPlan(maze_return_plan_t plan);
static const char *MazeReturn_PhaseName(maze_return_phase_t phase);
static const char *MazeReturn_PlanName(maze_return_plan_t plan);
static char MazeReturn_ActionChar(maze_return_action_t action);
static maze_return_action_t MazeReturn_EnterFollow(uint32_t now_tick);
static maze_return_action_t MazeReturn_ScheduleTurnAction(maze_simple_state_t turn_state,
                                                          uint32_t now_tick);
static maze_return_action_t MazeReturn_EnterTurnBack(uint32_t now_tick);
static maze_return_action_t MazeReturn_StopDrive(uint32_t now_tick);
static maze_return_action_t MazeReturn_StopScanBrake(uint32_t now_tick);
static maze_return_action_t MazeReturn_DriveStraightAction(uint32_t now_tick);
static void MazeReturn_BeginPathAlign(uint8_t decision, uint8_t path_dir, uint32_t now_tick);
#if MAZE_ASTAR_ENABLE
static void MazeReturn_BeginAstarPathAlign(uint32_t now_tick, const char *tag);
#endif
static void MazeReturn_FinishPathAlign(uint32_t now_tick);
static maze_return_action_t MazeReturn_Update(uint32_t now_tick,
                                              const maze_return_update_input_t *in);
static maze_return_action_t MazeReturn_ActionFromState(void);
static maze_return_action_t MazeReturn_HandleStopScan(uint32_t now_tick,
                                                      const maze_return_stop_scan_input_t *in);
static maze_return_action_t MazeReturn_HandleFollowAstar(uint32_t now_tick,
                                                         const maze_return_follow_input_t *in);
static int8_t MazeTopo_DecisionDebugValue(uint8_t decision);
static char MazeTopo_DecisionChar(uint8_t decision);
static void OGM_PrintViewMap(char map[OGM_HEIGHT][OGM_WIDTH]);
#if MAZE_ASTAR_ENABLE
static void MazeAstar_ResetPlan(void);
static void MazeAstar_ClearTempBlocks(void);
static uint8_t MazeAstar_BuildReturnPlan(uint32_t now_tick);
static uint8_t MazeAstar_BuildSegments(void);
static uint8_t MazeAstar_DebugSnapshot(maze_astar_debug_t *dbg);
static uint8_t MazeAstar_ReturnDynamicDecision(uint8_t *decision, uint8_t *goal_ready);
static void MazeAstar_ReturnBeginHome(uint32_t now_tick);
static uint8_t MazeAstar_ReturnFirstDecision(uint8_t *decision, uint8_t *path_dir);
static uint8_t MazeAstar_ReturnReplanForBlock(uint32_t now_tick,
                                              float front_m,
                                              uint32_t blocked_ms,
                                              uint8_t decision);
static uint8_t MazeAstar_CellOnPath(int x, int y);
#endif
#endif
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if ODOM_OGM_ENABLE
static float wrap_pi(float a)
{
  while (a > PI_F)
  {
    a -= 2.0f * PI_F;
  }
  while (a < -PI_F)
  {
    a += 2.0f * PI_F;
  }
  return a;
}

static float deg_to_rad(float deg)
{
  return deg * (PI_F / 180.0f);
}

static int absi(int v)
{
  return (v < 0) ? -v : v;
}

static void Odom_Reset(void)
{
  odom_x_m = 0.0f;
  odom_y_m = 0.0f;
  odom_theta_rad = 0.0f;
}

static void Odom_UpdateFromPulses(float pulse_left, float pulse_right)
{
  float dl_m = (pulse_left * ODOM_LEFT_PULSE_SIGN) / COUNTS_PER_METER;
  float dr_m = (pulse_right * ODOM_RIGHT_PULSE_SIGN) / COUNTS_PER_METER;
  float raw_ds_m = 0.5f * (dl_m + dr_m);
  float odom_scale = (raw_ds_m < 0.0f) ? ODOM_REVERSE_SCALE : ODOM_FORWARD_SCALE;
  float ds_m;
  float dtheta;
  float theta_mid;

  dl_m *= odom_scale;
  dr_m *= odom_scale;
  ds_m = 0.5f * (dl_m + dr_m);
  dtheta = (dr_m - dl_m) / ODOM_WHEEL_BASE_M;
  theta_mid = odom_theta_rad + (0.5f * dtheta);

  odom_x_m += ds_m * cosf(theta_mid);
  odom_y_m += ds_m * sinf(theta_mid);
  odom_theta_rad = wrap_pi(odom_theta_rad + dtheta);
}

static void OGM_Reset(void)
{
  memset(ogm_grid, 0, sizeof(ogm_grid));
  ogm_fordist_f = 0.0f;
  ogm_rightdist_f = 0.0f;
  ogm_rightfrontdist_f = 0.0f;
  ogm_rightreardist_f = 0.0f;
  ogm_reardist_f = 0.0f;
  ogm_leftreardist_f = 0.0f;
  ogm_leftdist_f = 0.0f;
  ogm_leftfrontdist_f = 0.0f;
#if OGM_FIXED_MAP_FRAME_ENABLE
  ogm_map_x_m = OGM_MAP_START_X_M;
  ogm_map_y_m = OGM_MAP_START_Y_M;
#else
  ogm_map_x_m = 0.0f;
  ogm_map_y_m = 0.0f;
#endif
  ogm_map_pose_valid = 0U;
  ogm_grid_heading_valid = 0U;
  ogm_grid_heading_rad = 0.0f;
  ogm_heading_ref_valid = 0U;
  ogm_heading_ref_yaw_rad = 0.0f;
  ogm_heading_ref_theta_rad = 0.0f;
  memset(ogm_pose_history, 0, sizeof(ogm_pose_history));
  ogm_pose_history_index = 0U;
  ogm_last_scan_tick_mapped = 0U;
  ogm_dbg_scan_raw_valid_bins = 0U;
  ogm_dbg_scan_filtered_bins = 0U;
  ogm_dbg_scan_far_bins = 0U;
  ogm_dbg_scan_masked_count = 0U;
  ogm_dbg_scan_tick = 0U;
  ogm_dbg_scan_update_count = 0U;
  ogm_dbg_scan_used = 0U;
  ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_NO_SCAN;
#if MAZE_TOPO_ENABLE
  MazeTopo_Reset();
#endif
  OGM_SeedStartRearWall();
}

static uint8_t OGM_MazeGridModeActive(void)
{
#if OGM_GRID_HEADING_ENABLE && MAZE_SIMPLE_RULE_ENABLE
  return (Maze_Enable != 0U || ogm_grid_heading_valid != 0U) ? 1U : 0U;
#else
  return 0U;
#endif
}

static void OGM_EnsureGridHeading(float fallback_theta_rad)
{
  if (ogm_grid_heading_valid == 0U)
  {
    ogm_grid_heading_rad = wrap_pi(fallback_theta_rad);
    ogm_grid_heading_valid = 1U;
  }
}

static void OGM_EnsureMapPose(void)
{
  if (ogm_map_pose_valid == 0U)
  {
#if OGM_FIXED_MAP_FRAME_ENABLE
    ogm_map_x_m = OGM_MAP_START_X_M;
    ogm_map_y_m = OGM_MAP_START_Y_M;
#else
    ogm_map_x_m = odom_x_m;
    ogm_map_y_m = odom_y_m;
#endif
    ogm_map_pose_valid = 1U;
  }
}

static void OGM_AdvanceMapPose(float signed_ds_m, float map_theta_rad)
{
  OGM_EnsureMapPose();

  if (OGM_MazeGridModeActive() != 0U)
  {
    ogm_map_x_m += signed_ds_m * cosf(map_theta_rad);
    ogm_map_y_m += signed_ds_m * sinf(map_theta_rad);
  }
  else
  {
    ogm_map_x_m = odom_x_m;
    ogm_map_y_m = odom_y_m;
  }
}

static uint8_t imu_attitude_within(float roll_limit_deg, float pitch_limit_deg)
{
  float abs_roll;

  if (imu_is_ready_for_runtime() == 0U)
  {
    return 0U;
  }

  if (fabsf(pitch) > pitch_limit_deg)
  {
    return 0U;
  }

  abs_roll = fabsf(roll);
  if (abs_roll <= roll_limit_deg ||
      fabsf(abs_roll - 180.0f) <= roll_limit_deg)
  {
    return 1U;
  }

  return 0U;
}

static uint8_t OGM_AttitudeStableForMapping(void)
{
  return imu_attitude_within(OGM_MAX_ABS_ROLL_DEG, OGM_MAX_ABS_PITCH_DEG);
}

static float OGM_GetMapThetaRad(void)
{
#if OGM_GRID_HEADING_ENABLE && MAZE_SIMPLE_RULE_ENABLE
  if (OGM_MazeGridModeActive() != 0U)
  {
    OGM_EnsureGridHeading(odom_theta_rad);
    return ogm_grid_heading_rad;
  }
#endif

  if (imu_is_ready_for_runtime() != 0U)
  {
    float yaw_now_rad = deg_to_rad(yaw);

    if (ogm_heading_ref_valid == 0U)
    {
      ogm_heading_ref_yaw_rad = yaw_now_rad;
      ogm_heading_ref_theta_rad = odom_theta_rad;
      ogm_heading_ref_valid = 1U;
    }

    return wrap_pi(ogm_heading_ref_theta_rad +
                   (OGM_IMU_HEADING_SIGN * wrap_pi(yaw_now_rad - ogm_heading_ref_yaw_rad)));
  }

  ogm_heading_ref_valid = 0U;
  return odom_theta_rad;
}

static float OGM_GetMapX(void)
{
  if (OGM_MazeGridModeActive() != 0U)
  {
    OGM_EnsureMapPose();
    return ogm_map_x_m;
  }

  return odom_x_m;
}

static float OGM_GetMapY(void)
{
  if (OGM_MazeGridModeActive() != 0U)
  {
    OGM_EnsureMapPose();
    return ogm_map_y_m;
  }

  return odom_y_m;
}

static void OGM_RecordPoseSample(uint32_t tick,
                                 float x_m,
                                 float y_m,
                                 float theta_rad)
{
  ogm_pose_history[ogm_pose_history_index].tick = tick;
  ogm_pose_history[ogm_pose_history_index].x_m = x_m;
  ogm_pose_history[ogm_pose_history_index].y_m = y_m;
  ogm_pose_history[ogm_pose_history_index].theta_rad = wrap_pi(theta_rad);
  ogm_pose_history[ogm_pose_history_index].valid = 1U;

  ogm_pose_history_index++;
  if (ogm_pose_history_index >= OGM_POSE_HISTORY_LEN)
  {
    ogm_pose_history_index = 0U;
  }
}

static void OGM_ClearPoseHistory(void)
{
  memset(ogm_pose_history, 0, sizeof(ogm_pose_history));
  ogm_pose_history_index = 0U;
}

static uint8_t OGM_GetPoseAtTick(uint32_t tick,
                                 float *x_m,
                                 float *y_m,
                                 float *theta_rad)
{
  uint8_t found = 0U;
  uint32_t best_age = OGM_SCAN_POSE_MAX_AGE_MS + 1U;

  if (x_m == NULL || y_m == NULL || theta_rad == NULL)
  {
    return 0U;
  }

  for (uint8_t i = 0U; i < OGM_POSE_HISTORY_LEN; i++)
  {
    uint32_t age;

    if (ogm_pose_history[i].valid == 0U)
    {
      continue;
    }

    age = (ogm_pose_history[i].tick >= tick) ?
          (ogm_pose_history[i].tick - tick) :
          (tick - ogm_pose_history[i].tick);
    if (age < best_age)
    {
      best_age = age;
      *x_m = ogm_pose_history[i].x_m;
      *y_m = ogm_pose_history[i].y_m;
      *theta_rad = ogm_pose_history[i].theta_rad;
      found = 1U;
    }
  }

  return (found != 0U && best_age <= OGM_SCAN_POSE_MAX_AGE_MS) ? 1U : 0U;
}

static void OGM_NoteMazeTurnComplete(maze_simple_state_t turn_state)
{
  if (OGM_MazeGridModeActive() == 0U)
  {
    return;
  }

  OGM_EnsureGridHeading(odom_theta_rad);
  if (turn_state == MAZE_SIMPLE_TURN_RIGHT)
  {
    ogm_grid_heading_rad = wrap_pi(ogm_grid_heading_rad - (PI_F * 0.5f));
  }
  else if (turn_state == MAZE_SIMPLE_TURN_LEFT)
  {
    ogm_grid_heading_rad = wrap_pi(ogm_grid_heading_rad + (PI_F * 0.5f));
  }
  else if (turn_state == MAZE_SIMPLE_TURN_BACK)
  {
    ogm_grid_heading_rad = wrap_pi(ogm_grid_heading_rad + PI_F);
  }

  OGM_ClearPoseHistory();
}

#if MAZE_TOPO_ENABLE
static void MazeReturn_SetPhase(maze_return_phase_t phase, uint32_t now_tick)
{
  if (maze_return_phase != phase)
  {
    maze_return_phase = phase;
    maze_return_phase_tick = now_tick;
  }
}

static void MazeReturn_SetPlan(maze_return_plan_t plan)
{
  maze_return_plan = plan;
}

static uint8_t MazeReturn_PostAlignReplanGuardActive(uint32_t now_tick)
{
#if MAZE_ASTAR_ENABLE
  if (maze_return_active == 0U ||
      maze_return_astar_active == 0U ||
      maze_return_align_done_tick == 0U)
  {
    return 0U;
  }

  return ((now_tick - maze_return_align_done_tick) <
          MAZE_ASTAR_POST_ALIGN_REPLAN_GRACE_MS) ? 1U : 0U;
#else
  (void)now_tick;
  return 0U;
#endif
}

static const char *MazeReturn_PhaseName(maze_return_phase_t phase)
{
  switch (phase)
  {
    case MAZE_RETURN_PHASE_IDLE:          return "IDLE";
    case MAZE_RETURN_PHASE_BUILD_PLAN:    return "PLAN";
    case MAZE_RETURN_PHASE_ALIGN_TO_PATH: return "ALIGN";
    case MAZE_RETURN_PHASE_FOLLOW_ASTAR:  return "ASTAR";
    case MAZE_RETURN_PHASE_FOLLOW_REPLAY: return "REPLAY";
    case MAZE_RETURN_PHASE_REPLAN:        return "REPLAN";
    case MAZE_RETURN_PHASE_HOME_APPROACH: return "HOME";
    case MAZE_RETURN_PHASE_DONE:          return "DONE";
    case MAZE_RETURN_PHASE_FAIL:          return "FAIL";
    default:                              return "?";
  }
}

static const char *MazeReturn_PlanName(maze_return_plan_t plan)
{
  switch (plan)
  {
    case MAZE_RETURN_PLAN_NONE:           return "NONE";
    case MAZE_RETURN_PLAN_ASTAR:          return "ASTAR";
    case MAZE_RETURN_PLAN_TOPO_REPLAY:    return "REPLAY";
    case MAZE_RETURN_PLAN_ASTAR_FALLBACK: return "AFB";
    default:                              return "?";
  }
}

static char MazeReturn_ActionChar(maze_return_action_t action)
{
  switch (action)
  {
    case MAZE_RETURN_ACTION_NONE:       return '-';
    case MAZE_RETURN_ACTION_DRIVE:      return 'D';
    case MAZE_RETURN_ACTION_STOP:       return 'S';
    case MAZE_RETURN_ACTION_TURN_LEFT:  return 'L';
    case MAZE_RETURN_ACTION_TURN_RIGHT: return 'R';
    case MAZE_RETURN_ACTION_TURN_BACK:  return 'B';
    case MAZE_RETURN_ACTION_DONE:       return 'O';
    case MAZE_RETURN_ACTION_FAIL:       return 'F';
    case MAZE_RETURN_ACTION_HANDLED:    return 'H';
    default:                            return '?';
  }
}

static maze_return_action_t MazeReturn_EnterFollow(uint32_t now_tick)
{
  maze_simple_prefer_turn_dir = 0;
  maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
  return MAZE_RETURN_ACTION_DRIVE;
}

static maze_return_action_t MazeReturn_ScheduleTurnAction(maze_simple_state_t turn_state,
                                                          uint32_t now_tick)
{
  maze_simple_prefer_turn_dir = 0;
  maze_simple_schedule_turn(turn_state, now_tick);
  if (turn_state == MAZE_SIMPLE_TURN_RIGHT)
  {
    return MAZE_RETURN_ACTION_TURN_RIGHT;
  }
  if (turn_state == MAZE_SIMPLE_TURN_LEFT)
  {
    return MAZE_RETURN_ACTION_TURN_LEFT;
  }
  if (turn_state == MAZE_SIMPLE_TURN_BACK)
  {
    return MAZE_RETURN_ACTION_TURN_BACK;
  }
  return MazeReturn_ActionFromState();
}

static maze_return_action_t MazeReturn_EnterTurnBack(uint32_t now_tick)
{
  maze_simple_prefer_turn_dir = 0;
  maze_simple_enter(MAZE_SIMPLE_TURN_BACK, now_tick);
  return MAZE_RETURN_ACTION_TURN_BACK;
}

static maze_return_action_t MazeReturn_StopDrive(uint32_t now_tick)
{
  MazeDrive(0.0f, 0.0f, now_tick);
  return MAZE_RETURN_ACTION_STOP;
}

static maze_return_action_t MazeReturn_StopScanBrake(uint32_t now_tick)
{
  maze_simple_enter(MAZE_SIMPLE_STOP_SCAN, now_tick);
  maze_simple_drive_stop_brake(now_tick);
  return MAZE_RETURN_ACTION_STOP;
}

static maze_return_action_t MazeReturn_DriveStraightAction(uint32_t now_tick)
{
  maze_simple_drive_straight(now_tick);
  return MAZE_RETURN_ACTION_DRIVE;
}

static void MazeReturn_BeginPathAlign(uint8_t decision, uint8_t path_dir, uint32_t now_tick)
{
  maze_return_target_dir = path_dir;
  maze_return_latched_decision = decision;
  maze_return_align_done_tick = 0U;

  if (decision == MAZE_TOPO_DECISION_RIGHT)
  {
    MazeReturn_SetPhase(MAZE_RETURN_PHASE_ALIGN_TO_PATH, now_tick);
    maze_simple_enter(MAZE_SIMPLE_TURN_RIGHT, now_tick);
    maze_simple_turn_drive(MAZE_SIMPLE_TURN_RIGHT, now_tick);
  }
  else if (decision == MAZE_TOPO_DECISION_LEFT)
  {
    MazeReturn_SetPhase(MAZE_RETURN_PHASE_ALIGN_TO_PATH, now_tick);
    maze_simple_enter(MAZE_SIMPLE_TURN_LEFT, now_tick);
    maze_simple_turn_drive(MAZE_SIMPLE_TURN_LEFT, now_tick);
  }
  else if (decision == MAZE_TOPO_DECISION_BACKUP)
  {
    MazeReturn_SetPhase(MAZE_RETURN_PHASE_ALIGN_TO_PATH, now_tick);
    maze_simple_enter(MAZE_SIMPLE_TURN_BACK, now_tick);
    maze_simple_turn_drive(MAZE_SIMPLE_TURN_BACK, now_tick);
  }
  else
  {
    maze_return_align_done_tick = now_tick;
#if MAZE_ASTAR_ENABLE
    maze_astar_segment_turn_pending = 0U;
#endif
    MazeReturn_SetPhase((maze_return_home_pending != 0U) ?
                        MAZE_RETURN_PHASE_HOME_APPROACH :
                        MAZE_RETURN_PHASE_FOLLOW_ASTAR,
                        now_tick);
    maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
  }
}

static void MazeReturn_FinishPathAlign(uint32_t now_tick)
{
  char msg[72];
  int n;

  if (maze_return_active == 0U ||
      maze_return_astar_active == 0U ||
      maze_return_phase != MAZE_RETURN_PHASE_ALIGN_TO_PATH)
  {
    return;
  }

  if (maze_return_target_dir <= 3U)
  {
    uint8_t heading = (uint8_t)(MazeTopo_HeadingQuadrant() & 0x03);
    if (heading != maze_return_target_dir)
    {
      n = snprintf(msg, sizeof(msg),
                   "RET:ALIGN_FAIL HEAD H=%u T=%u\r\n",
                   (unsigned)heading,
                   (unsigned)maze_return_target_dir);
      if (n > 0)
      {
        uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
      }
      MazeTopo_StopReturn(0U, now_tick);
      return;
    }
  }

  MazeReturn_SetPhase((maze_return_home_pending != 0U) ?
                      MAZE_RETURN_PHASE_HOME_APPROACH :
                      MAZE_RETURN_PHASE_FOLLOW_ASTAR,
                      now_tick);
  maze_return_align_done_tick = now_tick;
  maze_return_latched_decision = MAZE_TOPO_DECISION_STRAIGHT;
#if MAZE_ASTAR_ENABLE
  maze_astar_segment_turn_pending = 0U;
#endif
  n = snprintf(msg, sizeof(msg),
               "RET:ALIGN_DONE DIR=%u\r\n",
               (unsigned)maze_return_target_dir);
  if (n > 0)
  {
    uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
  }
}

#if MAZE_ASTAR_ENABLE
static void MazeReturn_BeginAstarPathAlign(uint32_t now_tick, const char *tag)
{
  char msg[72];
  int n;
  uint8_t astar_start_decision = MAZE_TOPO_DECISION_NONE;
  uint8_t astar_start_dir = 4U;

  if (maze_return_remaining == 0U)
  {
    maze_return_target_dir = 4U;
    maze_return_latched_decision = MAZE_TOPO_DECISION_NONE;
    MazeReturn_SetPhase(MAZE_RETURN_PHASE_HOME_APPROACH, now_tick);
    maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
    return;
  }

  if (MazeAstar_ReturnFirstDecision(&astar_start_decision, &astar_start_dir) == 0U)
  {
    maze_return_target_dir = 4U;
    maze_return_latched_decision = MAZE_TOPO_DECISION_NONE;
    MazeReturn_SetPhase(MAZE_RETURN_PHASE_FOLLOW_ASTAR, now_tick);
    maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
    return;
  }

  MazeReturn_BeginPathAlign(astar_start_decision, astar_start_dir, now_tick);

  n = snprintf(msg, sizeof(msg),
               "RET:%s_ALIGN D=%c DIR=%u\r\n",
               (tag != NULL) ? tag : "PATH",
               MazeTopo_DecisionChar(astar_start_decision),
               (unsigned)astar_start_dir);
  if (n > 0)
  {
    uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
  }
}
#endif

static void MazeTopo_ReturnResetState(void)
{
  maze_return_active = 0U;
  maze_return_done = 0U;
  maze_return_remaining = 0U;
  maze_return_astar_active = 0U;
  maze_return_astar_fallback = 0U;
  maze_return_phase = MAZE_RETURN_PHASE_IDLE;
  maze_return_plan = MAZE_RETURN_PLAN_NONE;
  maze_return_last_action = MAZE_RETURN_ACTION_NONE;
  maze_return_target_dir = 4U;
  maze_return_latched_decision = MAZE_TOPO_DECISION_NONE;
  maze_return_phase_tick = 0U;
  maze_return_align_done_tick = 0U;
  maze_return_home_pending = 0U;
  maze_return_home_tick = 0U;
  maze_return_home_drive_started = 0U;
  maze_return_home_drive_tick = 0U;
  maze_return_home_start_x_m = 0.0f;
  maze_return_home_start_y_m = 0.0f;
  maze_return_final_target_m = MAZE_RETURN_FINAL_M;
  maze_return_align_pending = 0U;
  maze_simple_start_align_preserve_map = 0U;
#if MAZE_ASTAR_ENABLE
  MazeAstar_ClearTempBlocks();
#endif
}

static void MazeTopo_Reset(void)
{
  memset(maze_topo_nodes, 0, sizeof(maze_topo_nodes));
  memset(maze_topo_failed_world, 0, sizeof(maze_topo_failed_world));
  maze_topo_count = 0U;
  maze_topo_next_id = 1U;
  maze_topo_write_index = 0U;
  MazeTopo_CancelPendingDecision();
  maze_topo_arrival_back_world_bit = 0U;
  maze_topo_arrival_source_idx = MAZE_TOPO_MAX_NODES;
  memset(maze_topo_path_nodes, 0, sizeof(maze_topo_path_nodes));
  memset(maze_topo_path_decisions, 0, sizeof(maze_topo_path_decisions));
  memset(maze_topo_path_world_bits, 0, sizeof(maze_topo_path_world_bits));
  maze_topo_path_count = 0U;
  MazeTopo_ReturnResetState();
#if MAZE_ASTAR_ENABLE
  MazeAstar_ResetPlan();
#endif
}

static int8_t MazeTopo_HeadingQuadrant(void)
{
  float theta = wrap_pi(OGM_GetMapThetaRad());
  float q = theta / (PI_F * 0.5f);
  int h = (q >= 0.0f) ? (int)(q + 0.5f) : (int)(q - 0.5f);

  h %= 4;
  if (h < 0)
  {
    h += 4;
  }

  return (int8_t)h;
}

static uint8_t MazeTopo_ClassifyWorld(uint8_t world_open)
{
  uint8_t choices = 0U;

  if ((world_open & MAZE_TOPO_WORLD_N) != 0U)
  {
    choices++;
  }
  if ((world_open & MAZE_TOPO_WORLD_E) != 0U)
  {
    choices++;
  }
  if ((world_open & MAZE_TOPO_WORLD_S) != 0U)
  {
    choices++;
  }
  if ((world_open & MAZE_TOPO_WORLD_W) != 0U)
  {
    choices++;
  }

  if (choices <= 1U)
  {
    return MAZE_TOPO_KIND_DEADEND;
  }
  if (choices == 2U)
  {
    return MAZE_TOPO_KIND_CORNER;
  }
  return MAZE_TOPO_KIND_JUNCTION;
}

static uint8_t MazeTopo_ShouldRecord(uint8_t openings)
{
  if ((openings & (MAZE_TOPO_OPEN_RIGHT | MAZE_TOPO_OPEN_LEFT)) != 0U)
  {
    return 1U;
  }

  if ((openings & MAZE_TOPO_OPEN_FRONT) == 0U)
  {
    return 1U;
  }

  return 0U;
}

static uint16_t MazeTopo_DistToMm(float dist_m)
{
  if (dist_m <= 0.0f)
  {
    return 0U;
  }
  if (dist_m > 6.0f)
  {
    return 6000U;
  }
  return (uint16_t)((dist_m * 1000.0f) + 0.5f);
}

static uint8_t MazeTopo_WorldBitCount(uint8_t mask)
{
  uint8_t count = 0U;

  if ((mask & MAZE_TOPO_WORLD_N) != 0U) { count++; }
  if ((mask & MAZE_TOPO_WORLD_E) != 0U) { count++; }
  if ((mask & MAZE_TOPO_WORLD_S) != 0U) { count++; }
  if ((mask & MAZE_TOPO_WORLD_W) != 0U) { count++; }

  return count;
}

static uint16_t MazeTopo_FindMergeIndex(float x_m, float y_m)
{
  const float merge_dist2 = MAZE_TOPO_MERGE_DIST_M * MAZE_TOPO_MERGE_DIST_M;
  uint16_t best_index = MAZE_TOPO_MAX_NODES;
  float best_dist2 = merge_dist2;
  uint16_t i;

  for (i = 0U; i < maze_topo_count; i++)
  {
    float dx = maze_topo_nodes[i].x_m - x_m;
    float dy = maze_topo_nodes[i].y_m - y_m;
    float dist2 = (dx * dx) + (dy * dy);

    if (dist2 <= best_dist2)
    {
      best_dist2 = dist2;
      best_index = i;
    }
  }

  return best_index;
}

static uint16_t MazeTopo_FindRevisitMergeIndex(float x_m,
                                               float y_m,
                                               uint8_t world_open)
{
  const float merge_dist2 =
    MAZE_TOPO_REVISIT_MERGE_DIST_M * MAZE_TOPO_REVISIT_MERGE_DIST_M;
  uint16_t best_index = MAZE_TOPO_MAX_NODES;
  float best_dist2 = merge_dist2;
  uint16_t i;

  if (MazeTopo_WorldBitCount(world_open) < 3U)
  {
    return MAZE_TOPO_MAX_NODES;
  }

  for (i = 0U; i < maze_topo_count; i++)
  {
    uint8_t merged_open = (uint8_t)(maze_topo_nodes[i].open_world | world_open);
    uint8_t shared_open = (uint8_t)(maze_topo_nodes[i].open_world & world_open);
    uint8_t blocked = (uint8_t)(maze_topo_nodes[i].tried | maze_topo_failed_world[i]);
    uint8_t untried_now = (uint8_t)(world_open & (uint8_t)(~blocked));
    float dx;
    float dy;
    float dist2;

    if (maze_topo_nodes[i].open_world == 0U ||
        MazeTopo_WorldBitCount(merged_open) < 3U ||
        shared_open == 0U ||
        untried_now == 0U)
    {
      continue;
    }

    dx = maze_topo_nodes[i].x_m - x_m;
    dy = maze_topo_nodes[i].y_m - y_m;
    dist2 = (dx * dx) + (dy * dy);
    if (dist2 <= best_dist2)
    {
      best_dist2 = dist2;
      best_index = i;
    }
  }

  return best_index;
}

static void MazeTopo_MarkFailedWorldNear(uint16_t source_idx, uint8_t world_bit)
{
  const float fail_dist2 = MAZE_TOPO_FAIL_NEAR_M * MAZE_TOPO_FAIL_NEAR_M;
  uint8_t fail_mask = MazeTopo_FailedMaskForWorldBit(world_bit);
  float sx;
  float sy;
  uint16_t i;

  if (source_idx >= maze_topo_count || fail_mask == 0U)
  {
    return;
  }

  sx = maze_topo_nodes[source_idx].x_m;
  sy = maze_topo_nodes[source_idx].y_m;
  for (i = 0U; i < maze_topo_count; i++)
  {
    float dx = maze_topo_nodes[i].x_m - sx;
    float dy = maze_topo_nodes[i].y_m - sy;
    float dist2 = (dx * dx) + (dy * dy);

    if (dist2 <= fail_dist2)
    {
      maze_topo_failed_world[i] |= fail_mask;
    }
  }
}

static void MazeTopo_ClearFailedWorldNear(uint16_t source_idx, uint8_t world_bit)
{
  const float fail_dist2 = MAZE_TOPO_FAIL_NEAR_M * MAZE_TOPO_FAIL_NEAR_M;
  uint8_t fail_mask = MazeTopo_FailedMaskForWorldBit(world_bit);
  float sx;
  float sy;
  uint16_t i;

  if (source_idx >= maze_topo_count || fail_mask == 0U)
  {
    return;
  }

  sx = maze_topo_nodes[source_idx].x_m;
  sy = maze_topo_nodes[source_idx].y_m;
  for (i = 0U; i < maze_topo_count; i++)
  {
    float dx = maze_topo_nodes[i].x_m - sx;
    float dy = maze_topo_nodes[i].y_m - sy;
    float dist2 = (dx * dx) + (dy * dy);

    if (dist2 <= fail_dist2)
    {
      maze_topo_failed_world[i] =
        (uint8_t)(maze_topo_failed_world[i] & (uint8_t)(~fail_mask));
    }
  }
}

static uint8_t MazeTopo_QuadrantWorldBit(int8_t quadrant)
{
  int8_t q = quadrant % 4;
  if (q < 0)
  {
    q += 4;
  }

  switch (q)
  {
    case 0:
      return MAZE_TOPO_WORLD_E;
    case 1:
      return MAZE_TOPO_WORLD_N;
    case 2:
      return MAZE_TOPO_WORLD_W;
    case 3:
    default:
      return MAZE_TOPO_WORLD_S;
  }
}

static uint8_t MazeTopo_WorldBitOpposite(uint8_t world_bit)
{
  switch (world_bit)
  {
    case MAZE_TOPO_WORLD_N:
      return MAZE_TOPO_WORLD_S;
    case MAZE_TOPO_WORLD_E:
      return MAZE_TOPO_WORLD_W;
    case MAZE_TOPO_WORLD_S:
      return MAZE_TOPO_WORLD_N;
    case MAZE_TOPO_WORLD_W:
      return MAZE_TOPO_WORLD_E;
    default:
      return 0U;
  }
}

static uint8_t MazeTopo_FailedMaskForWorldBit(uint8_t world_bit)
{
#if MAZE_TOPO_FAIL_ADJACENT_DIRS_ENABLE
  switch (world_bit)
  {
    case MAZE_TOPO_WORLD_N:
      return (uint8_t)(MAZE_TOPO_WORLD_N | MAZE_TOPO_WORLD_E | MAZE_TOPO_WORLD_W);
    case MAZE_TOPO_WORLD_E:
      return (uint8_t)(MAZE_TOPO_WORLD_E | MAZE_TOPO_WORLD_N | MAZE_TOPO_WORLD_S);
    case MAZE_TOPO_WORLD_S:
      return (uint8_t)(MAZE_TOPO_WORLD_S | MAZE_TOPO_WORLD_E | MAZE_TOPO_WORLD_W);
    case MAZE_TOPO_WORLD_W:
      return (uint8_t)(MAZE_TOPO_WORLD_W | MAZE_TOPO_WORLD_N | MAZE_TOPO_WORLD_S);
    default:
      return 0U;
  }
#else
  return world_bit;
#endif
}

static uint8_t MazeTopo_IsForwardDecision(uint8_t decision)
{
  return (decision == MAZE_TOPO_DECISION_STRAIGHT ||
          decision == MAZE_TOPO_DECISION_RIGHT ||
          decision == MAZE_TOPO_DECISION_LEFT) ? 1U : 0U;
}

static uint8_t MazeTopo_InverseDecision(uint8_t decision)
{
  switch (decision)
  {
    case MAZE_TOPO_DECISION_RIGHT:
      return MAZE_TOPO_DECISION_LEFT;
    case MAZE_TOPO_DECISION_LEFT:
      return MAZE_TOPO_DECISION_RIGHT;
    case MAZE_TOPO_DECISION_STRAIGHT:
      return MAZE_TOPO_DECISION_STRAIGHT;
    case MAZE_TOPO_DECISION_BACKUP:
      return MAZE_TOPO_DECISION_BACKUP;
    case MAZE_TOPO_DECISION_NONE:
    default:
      return MAZE_TOPO_DECISION_NONE;
  }
}

static uint8_t MazeTopo_DecisionOpenBit(uint8_t decision)
{
  switch (decision)
  {
    case MAZE_TOPO_DECISION_STRAIGHT:
      return MAZE_TOPO_OPEN_FRONT;
    case MAZE_TOPO_DECISION_RIGHT:
      return MAZE_TOPO_OPEN_RIGHT;
    case MAZE_TOPO_DECISION_LEFT:
      return MAZE_TOPO_OPEN_LEFT;
    case MAZE_TOPO_DECISION_BACKUP:
      return MAZE_TOPO_OPEN_BACK;
    case MAZE_TOPO_DECISION_NONE:
    default:
      return 0U;
  }
}

static uint8_t MazeTopo_DecisionWorldBit(uint8_t decision, int8_t heading_quadrant)
{
  int8_t q = heading_quadrant;

  switch (decision)
  {
    case MAZE_TOPO_DECISION_STRAIGHT:
      break;
    case MAZE_TOPO_DECISION_RIGHT:
      q--;
      break;
    case MAZE_TOPO_DECISION_LEFT:
      q++;
      break;
    case MAZE_TOPO_DECISION_BACKUP:
      q += 2;
      break;
    case MAZE_TOPO_DECISION_NONE:
    default:
      return 0U;
  }

  return MazeTopo_QuadrantWorldBit(q);
}

static uint8_t MazeTopo_WorldBitToDecision(uint8_t world_bit, int8_t heading_quadrant)
{
  if (world_bit == MazeTopo_DecisionWorldBit(MAZE_TOPO_DECISION_STRAIGHT, heading_quadrant))
  {
    return MAZE_TOPO_DECISION_STRAIGHT;
  }
  if (world_bit == MazeTopo_DecisionWorldBit(MAZE_TOPO_DECISION_RIGHT, heading_quadrant))
  {
    return MAZE_TOPO_DECISION_RIGHT;
  }
  if (world_bit == MazeTopo_DecisionWorldBit(MAZE_TOPO_DECISION_LEFT, heading_quadrant))
  {
    return MAZE_TOPO_DECISION_LEFT;
  }
  if (world_bit == MazeTopo_DecisionWorldBit(MAZE_TOPO_DECISION_BACKUP, heading_quadrant))
  {
    return MAZE_TOPO_DECISION_BACKUP;
  }

  return MAZE_TOPO_DECISION_NONE;
}

static uint8_t MazeTopo_WorldOpenMask(uint8_t openings, int8_t heading_quadrant)
{
  uint8_t mask = 0U;

  if ((openings & MAZE_TOPO_OPEN_FRONT) != 0U)
  {
    mask |= MazeTopo_DecisionWorldBit(MAZE_TOPO_DECISION_STRAIGHT, heading_quadrant);
  }
  if ((openings & MAZE_TOPO_OPEN_RIGHT) != 0U)
  {
    mask |= MazeTopo_DecisionWorldBit(MAZE_TOPO_DECISION_RIGHT, heading_quadrant);
  }
  if ((openings & MAZE_TOPO_OPEN_BACK) != 0U)
  {
    mask |= MazeTopo_DecisionWorldBit(MAZE_TOPO_DECISION_BACKUP, heading_quadrant);
  }
  if ((openings & MAZE_TOPO_OPEN_LEFT) != 0U)
  {
    mask |= MazeTopo_DecisionWorldBit(MAZE_TOPO_DECISION_LEFT, heading_quadrant);
  }

  return mask;
}

static uint8_t MazeTopo_DecisionIsUntried(uint8_t openings,
                                          uint8_t tried_world,
                                          int8_t heading_quadrant,
                                          uint8_t decision)
{
  uint8_t open_bit = MazeTopo_DecisionOpenBit(decision);
  uint8_t world_bit = MazeTopo_DecisionWorldBit(decision, heading_quadrant);

  if (open_bit == 0U || world_bit == 0U)
  {
    return 0U;
  }
  if ((openings & open_bit) == 0U)
  {
    return 0U;
  }
  return ((tried_world & world_bit) == 0U) ? 1U : 0U;
}

static uint8_t MazeTopo_PickUntriedDecision(uint8_t openings,
                                            uint8_t tried_world,
                                            int8_t heading_quadrant,
                                            uint8_t first_visit,
                                            uint8_t from_backup)
{
#if MAZE_FIXED_FAST_EXIT_ENABLE
  (void)tried_world;
  (void)first_visit;
  (void)from_backup;

  if (MazeTopo_DecisionIsUntried(openings, 0U, heading_quadrant, MAZE_TOPO_DECISION_LEFT) != 0U)
  {
    return MAZE_TOPO_DECISION_LEFT;
  }
  if (MazeTopo_DecisionIsUntried(openings, 0U, heading_quadrant, MAZE_TOPO_DECISION_RIGHT) != 0U)
  {
    return MAZE_TOPO_DECISION_RIGHT;
  }
  if (MazeTopo_DecisionIsUntried(openings, 0U, heading_quadrant, MAZE_TOPO_DECISION_STRAIGHT) != 0U)
  {
    return MAZE_TOPO_DECISION_STRAIGHT;
  }

  return MAZE_TOPO_DECISION_BACKUP;
#else
  uint8_t world_available =
    (uint8_t)(MazeTopo_WorldOpenMask((uint8_t)(openings & (MAZE_TOPO_OPEN_FRONT |
                                                           MAZE_TOPO_OPEN_RIGHT |
                                                           MAZE_TOPO_OPEN_LEFT)),
                                     heading_quadrant) &
              (uint8_t)(MAZE_TOPO_WORLD_N |
                        MAZE_TOPO_WORLD_E |
                        MAZE_TOPO_WORLD_S |
                        MAZE_TOPO_WORLD_W));
  uint8_t untried_world = (uint8_t)(world_available & (uint8_t)(~tried_world));

  if (from_backup == 0U &&
      first_visit != 0U &&
      (openings & MAZE_TOPO_OPEN_FRONT) != 0U)
  {
    return MAZE_TOPO_DECISION_STRAIGHT;
  }

  if (untried_world == 0U)
  {
    return MAZE_TOPO_DECISION_BACKUP;
  }

  if (from_backup != 0U || first_visit == 0U)
  {
    if (MazeTopo_DecisionIsUntried(openings, tried_world, heading_quadrant, MAZE_TOPO_DECISION_RIGHT) != 0U)
    {
      return MAZE_TOPO_DECISION_RIGHT;
    }
    if (MazeTopo_DecisionIsUntried(openings, tried_world, heading_quadrant, MAZE_TOPO_DECISION_LEFT) != 0U)
    {
      return MAZE_TOPO_DECISION_LEFT;
    }
    if (MazeTopo_DecisionIsUntried(openings, tried_world, heading_quadrant, MAZE_TOPO_DECISION_STRAIGHT) != 0U)
    {
      return MAZE_TOPO_DECISION_STRAIGHT;
    }
  }
  else
  {
    if (MazeTopo_DecisionIsUntried(openings, tried_world, heading_quadrant, MAZE_TOPO_DECISION_STRAIGHT) != 0U)
    {
      return MAZE_TOPO_DECISION_STRAIGHT;
    }
    if (MazeTopo_DecisionIsUntried(openings, tried_world, heading_quadrant, MAZE_TOPO_DECISION_RIGHT) != 0U)
    {
      return MAZE_TOPO_DECISION_RIGHT;
    }
    if (MazeTopo_DecisionIsUntried(openings, tried_world, heading_quadrant, MAZE_TOPO_DECISION_LEFT) != 0U)
    {
      return MAZE_TOPO_DECISION_LEFT;
    }
  }

  return MAZE_TOPO_DECISION_NONE;
#endif
}

static uint16_t MazeTopo_TouchNode(uint8_t openings, uint32_t now_tick, uint8_t count_visit)
{
  uint16_t idx;
  maze_topo_node_t *node;
  float x_m = OGM_GetMapX();
  float y_m = OGM_GetMapY();
  int8_t heading_quadrant = MazeTopo_HeadingQuadrant();
  uint8_t world_open;
  uint8_t revisit_merge = 0U;

  if (MazeTopo_ShouldRecord(openings) == 0U)
  {
    return MAZE_TOPO_MAX_NODES;
  }

  world_open = MazeTopo_WorldOpenMask(openings, heading_quadrant);
  idx = MazeTopo_FindMergeIndex(x_m, y_m);
  if (idx >= MAZE_TOPO_MAX_NODES)
  {
    idx = MazeTopo_FindRevisitMergeIndex(x_m, y_m, world_open);
    if (idx < MAZE_TOPO_MAX_NODES)
    {
      revisit_merge = 1U;
    }
  }
  if (idx >= MAZE_TOPO_MAX_NODES)
  {
    if (maze_topo_count < MAZE_TOPO_MAX_NODES)
    {
      idx = maze_topo_count;
      maze_topo_count++;
    }
    else
    {
      idx = maze_topo_write_index;
      maze_topo_write_index++;
      if (maze_topo_write_index >= MAZE_TOPO_MAX_NODES)
      {
        maze_topo_write_index = 0U;
      }
    }

    node = &maze_topo_nodes[idx];
    memset(node, 0, sizeof(*node));
    node->id = maze_topo_next_id;
    maze_topo_next_id++;
    if (maze_topo_next_id == 0U)
    {
      maze_topo_next_id = 1U;
    }
    node->x_m = x_m;
    node->y_m = y_m;
  }
  else
  {
    node = &maze_topo_nodes[idx];
    if (revisit_merge != 0U)
    {
      node->x_m = (node->x_m * 0.90f) + (x_m * 0.10f);
      node->y_m = (node->y_m * 0.90f) + (y_m * 0.10f);
    }
    else
    {
      node->x_m = (node->x_m * 0.75f) + (x_m * 0.25f);
      node->y_m = (node->y_m * 0.75f) + (y_m * 0.25f);
    }
  }

  node->heading_quadrant = heading_quadrant;
  node->openings |= openings;
  node->open_world |= world_open;
  if (maze_topo_arrival_back_world_bit != 0U &&
      idx != maze_topo_arrival_source_idx)
  {
    node->open_world |= maze_topo_arrival_back_world_bit;
    node->tried |= maze_topo_arrival_back_world_bit;
    maze_topo_arrival_back_world_bit = 0U;
    maze_topo_arrival_source_idx = MAZE_TOPO_MAX_NODES;
  }
  node->kind = MazeTopo_ClassifyWorld(node->open_world);
  node->front_mm = MazeTopo_DistToMm(fordist);
  node->right_mm = MazeTopo_DistToMm(rightdist);
  node->rear_mm = MazeTopo_DistToMm(reardist);
  node->left_mm = MazeTopo_DistToMm(leftdist);
  if (count_visit != 0U && node->visits < 255U)
  {
    node->visits++;
  }
  node->tick = now_tick;

  return idx;
}

static void MazeTopo_RecordPathNode(uint16_t idx, uint8_t decision, uint8_t world_bit)
{
  uint16_t i;

  if (maze_return_active != 0U || MazeTopo_IsForwardDecision(decision) == 0U)
  {
    return;
  }

  for (i = 0U; i < maze_topo_path_count; i++)
  {
    if (maze_topo_path_nodes[i] == idx)
    {
      maze_topo_path_count = i + 1U;
      maze_topo_path_decisions[i] = decision;
      maze_topo_path_world_bits[i] = world_bit;
      return;
    }
  }

  if (maze_topo_path_count > 0U &&
      maze_topo_path_nodes[maze_topo_path_count - 1U] == idx)
  {
    maze_topo_path_decisions[maze_topo_path_count - 1U] = decision;
    maze_topo_path_world_bits[maze_topo_path_count - 1U] = world_bit;
    return;
  }

  if (maze_topo_path_count < MAZE_TOPO_MAX_NODES)
  {
    maze_topo_path_nodes[maze_topo_path_count] = idx;
    maze_topo_path_decisions[maze_topo_path_count] = decision;
    maze_topo_path_world_bits[maze_topo_path_count] = world_bit;
    maze_topo_path_count++;
  }
}

static uint8_t MazeTopo_ReturnPeekDecision(void)
{
  uint8_t decision;

  if (maze_return_active == 0U || maze_return_remaining == 0U)
  {
    return MAZE_TOPO_DECISION_NONE;
  }

#if MAZE_ASTAR_ENABLE
  if (maze_return_astar_active != 0U)
  {
    return MAZE_TOPO_DECISION_NONE;
  }
#endif

  decision = MazeTopo_PathReturnDecision((uint16_t)(maze_return_remaining - 1U));
  maze_return_latched_decision = decision;
  maze_return_target_dir = 4U;
  return decision;
}

static uint8_t MazeTopo_PathReturnDecision(uint16_t path_i)
{
  if (path_i >= maze_topo_path_count)
  {
    return MAZE_TOPO_DECISION_NONE;
  }

  if (maze_topo_path_world_bits[path_i] != 0U)
  {
    return MazeTopo_WorldBitToDecision(MazeTopo_WorldBitOpposite(maze_topo_path_world_bits[path_i]),
                                       MazeTopo_HeadingQuadrant());
  }

  return MazeTopo_InverseDecision(maze_topo_path_decisions[path_i]);
}

static void MazeTopo_ReturnConsumeDecision(uint32_t now_tick)
{
  if (maze_return_remaining > 0U)
  {
    maze_return_remaining--;
  }

  if (maze_return_remaining == 0U)
  {
    maze_return_home_pending = 1U;
    maze_return_home_tick = now_tick;
    maze_return_home_drive_started = 0U;
    maze_return_home_drive_tick = 0U;
    maze_return_home_start_x_m = OGM_GetMapX();
    maze_return_home_start_y_m = OGM_GetMapY();
    maze_return_final_target_m = MAZE_RETURN_FINAL_M;
    MazeReturn_SetPhase(MAZE_RETURN_PHASE_HOME_APPROACH, now_tick);
#if MAZE_ASTAR_ENABLE
    if (maze_return_astar_active != 0U && maze_astar_final_segment_m > 0.0f)
    {
      maze_return_final_target_m = maze_astar_final_segment_m - MAZE_RETURN_FINAL_MARGIN_M;
      if (maze_return_final_target_m < MAZE_RETURN_FINAL_MIN_M)
      {
        maze_return_final_target_m = MAZE_RETURN_FINAL_MIN_M;
      }
      if (maze_return_final_target_m > MAZE_RETURN_FINAL_MAX_M)
      {
        maze_return_final_target_m = MAZE_RETURN_FINAL_MAX_M;
      }
    }
#endif
  }
}

static uint8_t MazeTopo_ReturnHomeReached(uint32_t now_tick)
{
  float dx_start;
  float dy_start;
  float dist_start_m;
  float dx_seg;
  float dy_seg;
  float dist_seg_m;
  uint32_t home_drive_elapsed;

  if (maze_return_active == 0U || maze_return_home_pending == 0U)
  {
    return 0U;
  }

  if (maze_return_home_drive_started == 0U)
  {
    if (maze_simple_state != MAZE_SIMPLE_FOLLOW)
    {
      return 0U;
    }

    maze_return_home_drive_started = 1U;
    maze_return_home_drive_tick = now_tick;
    maze_return_home_start_x_m = OGM_GetMapX();
    maze_return_home_start_y_m = OGM_GetMapY();
  }

  dx_start = OGM_GetMapX() - OGM_MAP_START_X_M;
  dy_start = OGM_GetMapY() - OGM_MAP_START_Y_M;
  dist_start_m = sqrtf((dx_start * dx_start) + (dy_start * dy_start));
  home_drive_elapsed = now_tick - maze_return_home_drive_tick;

#if MAZE_ASTAR_ENABLE
  if (maze_return_astar_fallback != 0U)
  {
    if (dist_start_m <= MAZE_RETURN_ASTAR_HOME_RADIUS_M &&
        home_drive_elapsed >= 350U)
    {
      return 1U;
    }
    return 0U;
  }

  if (maze_return_astar_active != 0U)
  {
    if (dist_start_m <= MAZE_RETURN_ASTAR_HOME_RADIUS_M &&
        home_drive_elapsed >= 350U)
    {
      return 1U;
    }

    if (fordist > 0.0f &&
        fordist <= MAZE_RETURN_FRONT_STOP_M &&
        dist_start_m <= MAZE_RETURN_ASTAR_FRONT_RADIUS_M &&
        home_drive_elapsed >= 250U)
    {
      return 1U;
    }

    dx_seg = OGM_GetMapX() - maze_return_home_start_x_m;
    dy_seg = OGM_GetMapY() - maze_return_home_start_y_m;
    dist_seg_m = sqrtf((dx_seg * dx_seg) + (dy_seg * dy_seg));
    if (dist_seg_m >= maze_return_final_target_m &&
        dist_start_m <= MAZE_RETURN_ASTAR_FINAL_RADIUS_M)
    {
      return 1U;
    }

    if (home_drive_elapsed >= MAZE_RETURN_FINAL_MS &&
        dist_start_m <= MAZE_RETURN_ASTAR_TIMEOUT_RADIUS_M)
    {
      return 1U;
    }

    return 0U;
  }
#endif

  if (dist_start_m <= MAZE_RETURN_HOME_RADIUS_M &&
      home_drive_elapsed >= 350U)
  {
    return 1U;
  }

  if (fordist > 0.0f &&
      fordist <= MAZE_RETURN_FRONT_STOP_M &&
      dist_start_m <= MAZE_RETURN_ASTAR_FRONT_RADIUS_M &&
      home_drive_elapsed >= 250U)
  {
    return 1U;
  }

  dx_seg = OGM_GetMapX() - maze_return_home_start_x_m;
  dy_seg = OGM_GetMapY() - maze_return_home_start_y_m;
  dist_seg_m = sqrtf((dx_seg * dx_seg) + (dy_seg * dy_seg));
  if (dist_seg_m >= maze_return_final_target_m &&
      dist_start_m <= MAZE_RETURN_ASTAR_FINAL_RADIUS_M)
  {
    return 1U;
  }

  if (home_drive_elapsed >= MAZE_RETURN_FINAL_MS &&
      dist_start_m <= MAZE_RETURN_ASTAR_TIMEOUT_RADIUS_M)
  {
    return 1U;
  }

  return 0U;
}

static void MazeTopo_StopReturn(uint8_t done, uint32_t now_tick)
{
  maze_return_active = 0U;
  maze_return_done = done;
  maze_return_remaining = 0U;
  maze_return_astar_active = 0U;
  maze_return_astar_fallback = 0U;
  MazeReturn_SetPlan(MAZE_RETURN_PLAN_NONE);
  MazeReturn_SetPhase((done != 0U) ? MAZE_RETURN_PHASE_DONE : MAZE_RETURN_PHASE_FAIL, now_tick);
  maze_return_align_done_tick = 0U;
  maze_return_target_dir = 4U;
  maze_return_latched_decision = MAZE_TOPO_DECISION_NONE;
  maze_return_home_pending = 0U;
  maze_return_home_drive_started = 0U;
  maze_return_home_drive_tick = 0U;
  Maze_Enable = 0U;
  maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
  MazeDrive(0.0f, 0.0f, now_tick);
#if MAZE_ASTAR_ENABLE
  MazeAstar_ClearTempBlocks();
#endif
}

static void MazeTopo_AbortPendingReturnAlign(uint32_t now_tick)
{
  if (maze_return_align_pending == 0U)
  {
    return;
  }

  maze_return_align_pending = 0U;
  maze_simple_start_align_preserve_map = 0U;
  MazeReturn_SetPlan(MAZE_RETURN_PLAN_NONE);
  MazeReturn_SetPhase(MAZE_RETURN_PHASE_IDLE, now_tick);
  maze_return_target_dir = 4U;
  maze_return_latched_decision = MAZE_TOPO_DECISION_NONE;
  Maze_Enable = 0U;
  maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
  MazeDrive(0.0f, 0.0f, now_tick);
}

static maze_return_start_result_t MazeTopo_StartReturnAfterAlign(uint32_t now_tick)
{
  char msg[96];
  int n;
  uint8_t astar_ok = 0U;

  maze_return_align_pending = 0U;
  maze_simple_start_align_preserve_map = 0U;
  MazeReturn_SetPlan(MAZE_RETURN_PLAN_NONE);
  MazeReturn_SetPhase(MAZE_RETURN_PHASE_BUILD_PLAN, now_tick);
  maze_return_align_done_tick = 0U;
  maze_return_target_dir = 4U;
  maze_return_latched_decision = MAZE_TOPO_DECISION_NONE;

#if MAZE_ASTAR_ENABLE
  MazeAstar_ClearTempBlocks();
  astar_ok = MazeAstar_BuildReturnPlan(now_tick);
#endif

  if (astar_ok == 0U)
  {
#if MAZE_ASTAR_ENABLE
    if (maze_astar_last_fail_pose != 0U)
    {
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)"RET:NO_POSE\r\n", 13U, 80);
      MazeReturn_SetPhase(MAZE_RETURN_PHASE_FAIL, now_tick);
      Maze_Enable = 0U;
      maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
      MazeDrive(0.0f, 0.0f, now_tick);
      return MAZE_RETURN_START_RESULT_NO_POSE;
    }
#endif

    if (maze_topo_path_count == 0U)
    {
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)"RET:NO_PATH\r\n", 13U, 80);
      MazeReturn_SetPhase(MAZE_RETURN_PHASE_FAIL, now_tick);
      Maze_Enable = 0U;
      maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
      MazeDrive(0.0f, 0.0f, now_tick);
      return MAZE_RETURN_START_RESULT_NO_PATH;
    }
  }

  maze_return_active = 1U;
  maze_return_done = 0U;
  maze_return_astar_active = astar_ok;
  maze_return_astar_fallback = 0U;
  MazeReturn_SetPlan((astar_ok != 0U) ? MAZE_RETURN_PLAN_ASTAR : MAZE_RETURN_PLAN_TOPO_REPLAY);
  maze_return_remaining =
#if MAZE_ASTAR_ENABLE
    (astar_ok != 0U) ? ((maze_astar_path_count > 0U) ? (uint16_t)(maze_astar_path_count - 1U) : 0U) :
#endif
    maze_topo_path_count;
  maze_return_home_pending = 0U;
  maze_return_home_drive_started = 0U;
  maze_return_home_drive_tick = 0U;
  if (maze_return_remaining == 0U)
  {
    maze_return_home_pending = 1U;
    maze_return_home_tick = now_tick;
    maze_return_home_drive_started = 0U;
    maze_return_home_drive_tick = 0U;
    maze_return_home_start_x_m = OGM_GetMapX();
    maze_return_home_start_y_m = OGM_GetMapY();
    maze_return_final_target_m = MAZE_RETURN_FINAL_M;
    MazeReturn_SetPhase(MAZE_RETURN_PHASE_HOME_APPROACH, now_tick);
#if MAZE_ASTAR_ENABLE
    if (maze_return_astar_active != 0U && maze_astar_final_segment_m > 0.0f)
    {
      maze_return_final_target_m = maze_astar_final_segment_m - MAZE_RETURN_FINAL_MARGIN_M;
      if (maze_return_final_target_m < MAZE_RETURN_FINAL_MIN_M)
      {
        maze_return_final_target_m = MAZE_RETURN_FINAL_MIN_M;
      }
      if (maze_return_final_target_m > MAZE_RETURN_FINAL_MAX_M)
      {
        maze_return_final_target_m = MAZE_RETURN_FINAL_MAX_M;
      }
    }
#endif
  }
  Maze_Enable = 1U;
  maze_simple_prefer_turn_dir = 0;
  maze_simple_turn_from_backup = 0U;
#if MAZE_ASTAR_ENABLE
  if (astar_ok != 0U)
  {
    MazeReturn_BeginAstarPathAlign(now_tick, "PATH");
  }
  else
#endif
  {
    MazeReturn_SetPhase((maze_return_home_pending != 0U) ?
                        MAZE_RETURN_PHASE_HOME_APPROACH :
                        MAZE_RETURN_PHASE_FOLLOW_REPLAY,
                        now_tick);
    maze_return_latched_decision = MAZE_TOPO_DECISION_BACKUP;
    maze_simple_enter(MAZE_SIMPLE_TURN_BACK, now_tick);
  }

  n = snprintf(msg, sizeof(msg),
               "RET:START MODE=%s N=%u\r\n",
               (astar_ok != 0U) ? "ASTAR" : "REPLAY",
               (unsigned)maze_return_remaining);
  if (n > 0)
  {
    uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
  }

  return MAZE_RETURN_START_RESULT_OK;
}

static void MazeTopo_StartReturn(uint32_t now_tick)
{
  if (calib_start_armed != 0U || calib_state != CALIB_STATE_IDLE)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"RET:BUSY CAL\r\n", 14U, 80);
    return;
  }

  if (maze_return_active != 0U || maze_return_align_pending != 0U)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"RET:BUSY\r\n", 10U, 80);
    return;
  }

#if MAZE_SIMPLE_START_ALIGN_ENABLE
  maze_return_align_pending = 1U;
  maze_simple_start_align_preserve_map = 1U;
  MazeReturn_SetPlan(MAZE_RETURN_PLAN_NONE);
  MazeReturn_SetPhase(MAZE_RETURN_PHASE_BUILD_PLAN, now_tick);
  maze_return_target_dir = 4U;
  maze_return_latched_decision = MAZE_TOPO_DECISION_NONE;
  Maze_Enable = 1U;
  maze_simple_prefer_turn_dir = 0;
  maze_simple_turn_from_backup = 0U;
  maze_simple_enter(MAZE_SIMPLE_START_ALIGN, now_tick);
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)"RET:ALIGN\r\n", 11U, 80);
#else
  (void)MazeTopo_StartReturnAfterAlign(now_tick);
#endif
}

static void MazeTopo_ClearReturn(uint32_t now_tick)
{
  MazeTopo_AbortPendingReturnAlign(now_tick);
  MazeTopo_ReturnResetState();
#if MAZE_ASTAR_ENABLE
  MazeAstar_ResetPlan();
#endif
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)"RET:CLEAR\r\n", 11U, 80);
}

static void MazeTopo_RecordDecision(uint8_t openings, uint8_t decision, uint32_t now_tick)
{
  uint16_t idx;
  maze_topo_node_t *node;
  uint8_t world_bit;

  MazeTopo_CancelPendingDecision();

  idx = MazeTopo_TouchNode(openings, now_tick, 1U);
  if (idx >= MAZE_TOPO_MAX_NODES)
  {
    return;
  }

  node = &maze_topo_nodes[idx];
  node->decision = decision;
  world_bit = MazeTopo_DecisionWorldBit(decision, node->heading_quadrant);
  if (world_bit != 0U)
  {
    node->tried |= world_bit;
  }

  MazeTopo_RecordPathNode(idx, decision, world_bit);
}

static void MazeTopo_PrintCommitDebug(uint16_t current_idx,
                                      float ds_m,
                                      uint8_t ok,
                                      char reason,
                                      uint32_t now_tick)
{
#if MAZE_TOPO_COMMIT_DEBUG
  char msg[112];
  int n;

  if (maze_topo_commit_active == 0U)
  {
    return;
  }

  n = snprintf(msg, sizeof(msg),
               "COMDBG:SRC=%u CUR=%u D=%c DS=%.2f OK=%u R=%c T=%lu\r\n",
               (unsigned)maze_topo_commit_idx,
               (unsigned)current_idx,
               MazeTopo_DecisionChar(maze_topo_commit_decision),
               ds_m,
               (unsigned)ok,
               reason,
               (unsigned long)now_tick);
  if (n > 0)
  {
    uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
  }
#else
  (void)current_idx;
  (void)ds_m;
  (void)ok;
  (void)reason;
  (void)now_tick;
#endif
}

static void MazeTopo_CommitPendingDecision(uint8_t decision, uint32_t now_tick)
{
  maze_topo_node_t *node;
  float dx;
  float dy;
  float ds_m;
  uint16_t current_idx;
  uint8_t side_decision;

  if (maze_topo_commit_active == 0U)
  {
    return;
  }

  if (decision != MAZE_TOPO_DECISION_NONE &&
      maze_topo_commit_decision != decision)
  {
    return;
  }

  if ((now_tick - maze_topo_commit_tick) < MAZE_TOPO_COMMIT_MIN_MS)
  {
    return;
  }

  dx = OGM_GetMapX() - maze_topo_commit_x_m;
  dy = OGM_GetMapY() - maze_topo_commit_y_m;
  ds_m = sqrtf((dx * dx) + (dy * dy));
  current_idx = MazeTopo_FindMergeIndex(OGM_GetMapX(), OGM_GetMapY());
  side_decision = (maze_topo_commit_decision == MAZE_TOPO_DECISION_RIGHT ||
                   maze_topo_commit_decision == MAZE_TOPO_DECISION_LEFT) ? 1U : 0U;

  if (side_decision != 0U)
  {
    if (current_idx == maze_topo_commit_idx ||
        ds_m < MAZE_TOPO_COMMIT_SIDE_MIN_LEAVE_M)
    {
      if ((now_tick - maze_topo_commit_tick) < MAZE_TOPO_COMMIT_SIDE_LEAVE_WAIT_MS)
      {
        return;
      }

      if (maze_topo_commit_idx < MAZE_TOPO_MAX_NODES)
      {
        MazeTopo_MarkFailedWorldNear(maze_topo_commit_idx, maze_topo_commit_world_bit);
      }
      MazeTopo_PrintCommitDebug(current_idx, ds_m, 0U, 'S', now_tick);
      MazeTopo_CancelPendingDecision();
      return;
    }

    if ((now_tick - maze_topo_commit_tick) >= MAZE_TOPO_COMMIT_SIDE_QUALITY_MIN_MS &&
        maze_simple_side_entry_quality_ok(maze_topo_commit_decision, now_tick) == 0U)
    {
      if (maze_topo_commit_idx < MAZE_TOPO_MAX_NODES)
      {
        MazeTopo_MarkFailedWorldNear(maze_topo_commit_idx, maze_topo_commit_world_bit);
      }
      MazeTopo_PrintCommitDebug(current_idx, ds_m, 0U, 'Q', now_tick);
      MazeTopo_CancelPendingDecision();
      return;
    }
  }
  else if (current_idx == maze_topo_commit_idx &&
           ds_m < MAZE_TOPO_COMMIT_SIDE_MIN_LEAVE_M)
  {
    return;
  }

  if (maze_topo_commit_idx < MAZE_TOPO_MAX_NODES &&
      maze_topo_commit_world_bit != 0U)
  {
    node = &maze_topo_nodes[maze_topo_commit_idx];
    node->decision = maze_topo_commit_decision;
    node->tried |= maze_topo_commit_world_bit;
    MazeTopo_ClearFailedWorldNear(maze_topo_commit_idx, maze_topo_commit_world_bit);
    node->openings |= maze_topo_commit_openings;
    node->open_world |= MazeTopo_WorldOpenMask(maze_topo_commit_openings, maze_topo_commit_heading);
    node->heading_quadrant = maze_topo_commit_heading;
    node->kind = MazeTopo_ClassifyWorld(node->open_world);
    MazeTopo_RecordPathNode(maze_topo_commit_idx, maze_topo_commit_decision, maze_topo_commit_world_bit);
    maze_topo_arrival_back_world_bit = MazeTopo_WorldBitOpposite(maze_topo_commit_world_bit);
    maze_topo_arrival_source_idx = maze_topo_commit_idx;
    MazeTopo_PrintCommitDebug(current_idx, ds_m, 1U, 'C', now_tick);
  }

  MazeTopo_CancelPendingDecision();
}

static uint8_t MazeTopo_AbortPendingSideIfBlocked(uint32_t now_tick)
{
  float dx;
  float dy;
  float ds_m;
  uint16_t current_idx;

  if (maze_topo_commit_active == 0U ||
      (maze_topo_commit_decision != MAZE_TOPO_DECISION_RIGHT &&
       maze_topo_commit_decision != MAZE_TOPO_DECISION_LEFT))
  {
    return 0U;
  }

  if (maze_simple_side_entry_panic(maze_topo_commit_decision, now_tick) == 0U)
  {
    return 0U;
  }

  dx = OGM_GetMapX() - maze_topo_commit_x_m;
  dy = OGM_GetMapY() - maze_topo_commit_y_m;
  ds_m = sqrtf((dx * dx) + (dy * dy));
  current_idx = MazeTopo_FindMergeIndex(OGM_GetMapX(), OGM_GetMapY());

  if (maze_topo_commit_idx < MAZE_TOPO_MAX_NODES)
  {
    MazeTopo_MarkFailedWorldNear(maze_topo_commit_idx, maze_topo_commit_world_bit);
  }
  MazeTopo_PrintCommitDebug(current_idx, ds_m, 0U, 'P', now_tick);
  MazeTopo_CancelPendingDecision();
  return 1U;
}

static void MazeTopo_CancelPendingDecision(void)
{
  maze_topo_commit_active = 0U;
  maze_topo_commit_idx = MAZE_TOPO_MAX_NODES;
  maze_topo_commit_decision = MAZE_TOPO_DECISION_NONE;
  maze_topo_commit_world_bit = 0U;
  maze_topo_commit_openings = 0U;
  maze_topo_commit_heading = 0;
  maze_topo_commit_tick = 0U;
  maze_topo_commit_x_m = 0.0f;
  maze_topo_commit_y_m = 0.0f;
}

static uint8_t MazeTopo_SelectExploreDecision(uint8_t openings, uint32_t now_tick, uint8_t from_backup)
{
  uint16_t idx = MazeTopo_TouchNode(openings, now_tick, 0U);
  const maze_topo_node_t *node;
  uint8_t first_visit;

  if (idx >= MAZE_TOPO_MAX_NODES)
  {
    return MAZE_TOPO_DECISION_NONE;
  }

  node = &maze_topo_nodes[idx];
  first_visit = (node->visits == 0U) ? 1U : 0U;
  return MazeTopo_PickUntriedDecision(openings,
                                      node->tried,
                                      node->heading_quadrant,
                                      first_visit,
                                      from_backup);
}

static uint8_t MazeTopo_CurrentNodeRevisited(void)
{
  uint16_t idx = MazeTopo_FindMergeIndex(OGM_GetMapX(), OGM_GetMapY());

  if (idx >= maze_topo_count)
  {
    return 0U;
  }

  if (maze_topo_nodes[idx].visits > 0U ||
      maze_topo_nodes[idx].tried != 0U ||
      maze_topo_failed_world[idx] != 0U)
  {
    return 1U;
  }

  return 0U;
}

static void MazeTopo_PrintDecisionDebug(uint8_t openings,
                                        uint8_t decision,
                                        uint8_t from_backup,
                                        int8_t pending_dir,
                                        uint32_t now_tick)
{
  uint16_t idx;
  const maze_topo_node_t *node = NULL;
  char msg[224];
  int n;
  float x_m = OGM_GetMapX();
  float y_m = OGM_GetMapY();
  int8_t heading_quadrant = MazeTopo_HeadingQuadrant();
  uint8_t world_available =
    (uint8_t)(MazeTopo_WorldOpenMask((uint8_t)(openings & (MAZE_TOPO_OPEN_FRONT |
                                                           MAZE_TOPO_OPEN_RIGHT |
                                                           MAZE_TOPO_OPEN_LEFT)),
                                     heading_quadrant) &
              (uint8_t)(MAZE_TOPO_WORLD_N |
                        MAZE_TOPO_WORLD_E |
                        MAZE_TOPO_WORLD_S |
                        MAZE_TOPO_WORLD_W));
  uint8_t tried = 0U;
  uint8_t failed = 0U;
  uint8_t untried;
  uint16_t node_id = 0U;
  uint8_t visits = 0U;
  char of = ((openings & MAZE_TOPO_OPEN_FRONT) != 0U) ? 'F' : '-';
  char oright = ((openings & MAZE_TOPO_OPEN_RIGHT) != 0U) ? 'R' : '-';
  char ob = ((openings & MAZE_TOPO_OPEN_BACK) != 0U) ? 'B' : '-';
  char ol = ((openings & MAZE_TOPO_OPEN_LEFT) != 0U) ? 'L' : '-';
  char an = ((world_available & MAZE_TOPO_WORLD_N) != 0U) ? 'N' : '-';
  char ae = ((world_available & MAZE_TOPO_WORLD_E) != 0U) ? 'E' : '-';
  char as = ((world_available & MAZE_TOPO_WORLD_S) != 0U) ? 'S' : '-';
  char aw = ((world_available & MAZE_TOPO_WORLD_W) != 0U) ? 'W' : '-';
  char tn = '-';
  char te = '-';
  char ts = '-';
  char tw = '-';
  char fn = '-';
  char fe = '-';
  char fs = '-';
  char fw = '-';
  char un = '-';
  char ue = '-';
  char us = '-';
  char uw = '-';

  idx = MazeTopo_FindMergeIndex(x_m, y_m);
  if (idx < maze_topo_count)
  {
    node = &maze_topo_nodes[idx];
    failed = maze_topo_failed_world[idx];
    tried = (uint8_t)(node->tried | failed);
    node_id = node->id;
    visits = node->visits;
    tn = ((tried & MAZE_TOPO_WORLD_N) != 0U) ? 'N' : '-';
    te = ((tried & MAZE_TOPO_WORLD_E) != 0U) ? 'E' : '-';
    ts = ((tried & MAZE_TOPO_WORLD_S) != 0U) ? 'S' : '-';
    tw = ((tried & MAZE_TOPO_WORLD_W) != 0U) ? 'W' : '-';
    fn = ((failed & MAZE_TOPO_WORLD_N) != 0U) ? 'N' : '-';
    fe = ((failed & MAZE_TOPO_WORLD_E) != 0U) ? 'E' : '-';
    fs = ((failed & MAZE_TOPO_WORLD_S) != 0U) ? 'S' : '-';
    fw = ((failed & MAZE_TOPO_WORLD_W) != 0U) ? 'W' : '-';
  }
  untried = (uint8_t)(world_available & (uint8_t)(~tried));
  un = ((untried & MAZE_TOPO_WORLD_N) != 0U) ? 'N' : '-';
  ue = ((untried & MAZE_TOPO_WORLD_E) != 0U) ? 'E' : '-';
  us = ((untried & MAZE_TOPO_WORLD_S) != 0U) ? 'S' : '-';
  uw = ((untried & MAZE_TOPO_WORLD_W) != 0U) ? 'W' : '-';

  n = snprintf(msg, sizeof(msg),
               "TOPODBG:ID=%u V=%u H=%d O=%c%c%c%c A=%c%c%c%c TR=%c%c%c%c FA=%c%c%c%c U=%c%c%c%c D=%c FB=%u P=%d PATH=%u X=%.2f Y=%.2f T=%lu\r\n",
               (unsigned)node_id,
               (unsigned)visits,
               (int)heading_quadrant,
               of,
               oright,
               ob,
               ol,
               an,
               ae,
               as,
               aw,
               tn,
               te,
               ts,
               tw,
               fn,
               fe,
               fs,
               fw,
               un,
               ue,
               us,
               uw,
               MazeTopo_DecisionChar(decision),
               (unsigned)from_backup,
               (int)pending_dir,
               (unsigned)maze_topo_path_count,
               (double)x_m,
               (double)y_m,
               (unsigned long)now_tick);
  if (n > 0)
  {
    uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
  }
}

static char MazeTopo_DecisionChar(uint8_t decision)
{
  switch (decision)
  {
    case MAZE_TOPO_DECISION_STRAIGHT:
      return 'F';
    case MAZE_TOPO_DECISION_RIGHT:
      return 'R';
    case MAZE_TOPO_DECISION_LEFT:
      return 'L';
    case MAZE_TOPO_DECISION_BACKUP:
      return 'B';
    case MAZE_TOPO_DECISION_NONE:
    default:
      return 'N';
  }
}

static int8_t MazeTopo_DecisionDebugValue(uint8_t decision)
{
  switch (decision)
  {
    case MAZE_TOPO_DECISION_STRAIGHT:
      return 2;
    case MAZE_TOPO_DECISION_RIGHT:
      return 1;
    case MAZE_TOPO_DECISION_LEFT:
      return -1;
    case MAZE_TOPO_DECISION_BACKUP:
      return -2;
    case MAZE_TOPO_DECISION_NONE:
    default:
      return 0;
  }
}

static char MazeTopo_KindChar(uint8_t kind)
{
  switch (kind)
  {
    case MAZE_TOPO_KIND_JUNCTION:
      return 'J';
    case MAZE_TOPO_KIND_CORNER:
      return 'C';
    case MAZE_TOPO_KIND_DEADEND:
      return 'E';
    case MAZE_TOPO_KIND_PASS:
    default:
      return 'P';
  }
}
#endif

static int OGM_WorldToCell(float x_m, float y_m, int *cx, int *cy)
{
  int ix;
  int iy;

  if (cx == NULL || cy == NULL)
  {
    return 0;
  }

  ix = (int)floorf(((x_m - OGM_MAP_CENTER_X_M) / OGM_RESOLUTION_M) + (OGM_WIDTH * 0.5f));
  iy = (int)floorf(((y_m - OGM_MAP_CENTER_Y_M) / OGM_RESOLUTION_M) + (OGM_HEIGHT * 0.5f));

  if (ix < 0 || ix >= OGM_WIDTH || iy < 0 || iy >= OGM_HEIGHT)
  {
    return 0;
  }

  *cx = ix;
  *cy = iy;
  return 1;
}

static float OGM_CellCenterX(int cx)
{
  return OGM_MAP_CENTER_X_M +
         (((((float)cx) + 0.5f) - (((float)OGM_WIDTH) * 0.5f)) * OGM_RESOLUTION_M);
}

static float OGM_CellCenterY(int cy)
{
  return OGM_MAP_CENTER_Y_M +
         (((((float)cy) + 0.5f) - (((float)OGM_HEIGHT) * 0.5f)) * OGM_RESOLUTION_M);
}

static uint8_t OGM_FindNearestOccCell(float x_m, float y_m, int *best_cx, int *best_cy)
{
  int cx;
  int cy;
  int best_d2 = 32767;
  uint8_t found = 0U;

  if (best_cx == NULL || best_cy == NULL)
  {
    return 0U;
  }
  if (OGM_WorldToCell(x_m, y_m, &cx, &cy) == 0)
  {
    return 0U;
  }

  for (int dy = -OGM_POSE_CORRECT_SEARCH_CELLS; dy <= OGM_POSE_CORRECT_SEARCH_CELLS; dy++)
  {
    for (int dx = -OGM_POSE_CORRECT_SEARCH_CELLS; dx <= OGM_POSE_CORRECT_SEARCH_CELLS; dx++)
    {
      int nx = cx + dx;
      int ny = cy + dy;
      int d2 = (dx * dx) + (dy * dy);

      if (nx < 0 || nx >= OGM_WIDTH || ny < 0 || ny >= OGM_HEIGHT)
      {
        continue;
      }
      if (ogm_grid[ny][nx] < OGM_POSE_CORRECT_OCC_MIN)
      {
        continue;
      }
      if (d2 < best_d2)
      {
        best_d2 = d2;
        *best_cx = nx;
        *best_cy = ny;
        found = 1U;
      }
    }
  }

  return found;
}

static void OGM_CorrectMapPoseFromWall(float rel_angle_rad, float dist_m, uint32_t age_ms)
{
#if OGM_POSE_CORRECT_ENABLE
  float map_theta_rad;
  float dir_angle_rad;
  float dir_x;
  float dir_y;
  float lidar_x;
  float lidar_y;
  float end_x;
  float end_y;
  int occ_cx;
  int occ_cy;
  float err_x;
  float err_y;
  float along_err;
  float step_m;

  if (OGM_MazeGridModeActive() == 0U || ogm_map_pose_valid == 0U)
  {
    return;
  }
  if (maze_simple_state == MAZE_SIMPLE_TURN_RIGHT ||
      maze_simple_state == MAZE_SIMPLE_TURN_LEFT ||
      maze_simple_state == MAZE_SIMPLE_TURN_BACK ||
      maze_simple_state == MAZE_SIMPLE_POST_TURN_LOCK ||
      maze_simple_state == MAZE_SIMPLE_POST_FORWARD ||
      maze_simple_state == MAZE_SIMPLE_BACKUP)
  {
    return;
  }
  if (age_ms > OGM_POSE_CORRECT_FRESH_MS ||
      dist_m < OGM_POSE_CORRECT_MIN_RANGE_M ||
      dist_m > OGM_POSE_CORRECT_MAX_RANGE_M)
  {
    return;
  }

  map_theta_rad = OGM_GetMapThetaRad();
  dir_angle_rad = wrap_pi(map_theta_rad + rel_angle_rad);
  dir_x = cosf(dir_angle_rad);
  dir_y = sinf(dir_angle_rad);
  lidar_x = (OGM_LIDAR_ORIGIN_X_M * cosf(map_theta_rad)) -
            (OGM_LIDAR_ORIGIN_Y_M * sinf(map_theta_rad));
  lidar_y = (OGM_LIDAR_ORIGIN_X_M * sinf(map_theta_rad)) +
            (OGM_LIDAR_ORIGIN_Y_M * cosf(map_theta_rad));
  end_x = ogm_map_x_m + lidar_x + (dir_x * dist_m);
  end_y = ogm_map_y_m + lidar_y + (dir_y * dist_m);

  if (OGM_FindNearestOccCell(end_x, end_y, &occ_cx, &occ_cy) == 0U)
  {
    return;
  }

  err_x = OGM_CellCenterX(occ_cx) - end_x;
  err_y = OGM_CellCenterY(occ_cy) - end_y;
  along_err = (err_x * dir_x) + (err_y * dir_y);
  step_m = along_err * OGM_POSE_CORRECT_GAIN;
  if (step_m > OGM_POSE_CORRECT_MAX_STEP_M)
  {
    step_m = OGM_POSE_CORRECT_MAX_STEP_M;
  }
  else if (step_m < -OGM_POSE_CORRECT_MAX_STEP_M)
  {
    step_m = -OGM_POSE_CORRECT_MAX_STEP_M;
  }

  ogm_map_x_m += step_m * dir_x;
  ogm_map_y_m += step_m * dir_y;
#else
  (void)rel_angle_rad;
  (void)dist_m;
  (void)age_ms;
#endif
}

static void OGM_CorrectMapPoseFromCardinalWalls(uint32_t now_tick)
{
#if OGM_POSE_CORRECT_ENABLE
  OGM_CorrectMapPoseFromWall(0.0f,
                             fordist,
                             now_tick - fordist_last_valid_tick);
  OGM_CorrectMapPoseFromWall(-PI_F * 0.5f,
                             rightdist,
                             now_tick - rightdist_last_valid_tick);
  OGM_CorrectMapPoseFromWall(PI_F,
                             reardist,
                             now_tick - reardist_last_valid_tick);
  OGM_CorrectMapPoseFromWall(PI_F * 0.5f,
                             leftdist,
                             now_tick - leftdist_last_valid_tick);
#else
  (void)now_tick;
#endif
}

static void OGM_AddCell(int cx, int cy, int delta)
{
  int v;

  if (cx < 0 || cx >= OGM_WIDTH || cy < 0 || cy >= OGM_HEIGHT)
  {
    return;
  }

  v = (int)ogm_grid[cy][cx] + delta;
  if (v > OGM_CELL_MAX)
  {
    v = OGM_CELL_MAX;
  }
  else if (v < OGM_CELL_MIN)
  {
    v = OGM_CELL_MIN;
  }
  ogm_grid[cy][cx] = (int8_t)v;
}

static void OGM_AddWorldOcc(float x_m, float y_m, int occ_delta, int neighbor_delta)
{
  int cx;
  int cy;

  if (OGM_WorldToCell(x_m, y_m, &cx, &cy) == 0)
  {
    return;
  }

  OGM_AddCell(cx, cy, occ_delta);
  if (neighbor_delta != 0)
  {
    OGM_AddCell(cx + 1, cy, neighbor_delta);
    OGM_AddCell(cx - 1, cy, neighbor_delta);
    OGM_AddCell(cx, cy + 1, neighbor_delta);
    OGM_AddCell(cx, cy - 1, neighbor_delta);
  }
}

static void OGM_SeedStartRearWall(void)
{
#if OGM_START_REAR_WALL_ENABLE
  float y_m;

  for (y_m = -OGM_START_REAR_WALL_HALF_WIDTH_M;
       y_m <= (OGM_START_REAR_WALL_HALF_WIDTH_M + (OGM_RESOLUTION_M * 0.5f));
       y_m += OGM_RESOLUTION_M)
  {
    OGM_AddWorldOcc(OGM_MAP_START_X_M - OGM_START_REAR_WALL_BACK_M,
                    y_m,
                    OGM_START_REAR_WALL_DELTA,
                    OGM_OCC_NEIGHBOR_DELTA);
  }
#endif
}

static void OGM_TraceFreeCells(int x0, int y0, int x1, int y1, int free_delta)
{
  int x = x0;
  int y = y0;
  int dx = absi(x1 - x0);
  int sx = (x0 < x1) ? 1 : -1;
  int dy = -absi(y1 - y0);
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  while ((x != x1) || (y != y1))
  {
    if (x < 0 || x >= OGM_WIDTH || y < 0 || y >= OGM_HEIGHT)
    {
      break;
    }

    if (free_delta != 0)
    {
      if (free_delta < 0 && ogm_grid[y][x] >= OGM_FREE_PROTECT_OCC_MIN)
      {
        /* Keep repeated free rays from erasing a wall that already has solid support. */
      }
      else
      {
        OGM_AddCell(x, y, free_delta);
      }
    }

    {
      int e2 = 2 * err;
      if (e2 >= dy)
      {
        err += dy;
        x += sx;
      }
      if (e2 <= dx)
      {
        err += dx;
        y += sy;
      }
    }
  }
}

static float OGM_RayMapLimitDist(float x_m, float y_m, float dir_x, float dir_y)
{
  const float map_min_x =
    OGM_MAP_CENTER_X_M - (((float)OGM_WIDTH) * OGM_RESOLUTION_M * 0.5f);
  const float map_max_x =
    OGM_MAP_CENTER_X_M + (((((float)OGM_WIDTH) * 0.5f) - 0.5f) * OGM_RESOLUTION_M);
  const float map_min_y =
    OGM_MAP_CENTER_Y_M - (((float)OGM_HEIGHT) * OGM_RESOLUTION_M * 0.5f);
  const float map_max_y =
    OGM_MAP_CENTER_Y_M + (((((float)OGM_HEIGHT) * 0.5f) - 0.5f) * OGM_RESOLUTION_M);
  float limit = DIST_VALID_MAX_M;
  float t;

  if (dir_x > 0.0001f)
  {
    t = (map_max_x - x_m) / dir_x;
    if (t > 0.0f && t < limit)
    {
      limit = t;
    }
  }
  else if (dir_x < -0.0001f)
  {
    t = (map_min_x - x_m) / dir_x;
    if (t > 0.0f && t < limit)
    {
      limit = t;
    }
  }

  if (dir_y > 0.0001f)
  {
    t = (map_max_y - y_m) / dir_y;
    if (t > 0.0f && t < limit)
    {
      limit = t;
    }
  }
  else if (dir_y < -0.0001f)
  {
    t = (map_min_y - y_m) / dir_y;
    if (t > 0.0f && t < limit)
    {
      limit = t;
    }
  }

  return limit;
}

static void OGM_UpdateRayWeightedAtPose(float map_x_m,
                                        float map_y_m,
                                        float map_theta_rad,
                                        float rel_angle_rad,
                                        float dist_m,
                                        float sensor_x_m,
                                        float sensor_y_m,
                                        int free_delta,
                                        int occ_delta,
                                        int neighbor_delta)
{
  int sx;
  int sy;
  int ex;
  int ey;
  float sensor_world_x_m;
  float sensor_world_y_m;
  float world_angle;
  float hit_x_m;
  float hit_y_m;
  float dir_x;
  float dir_y;
  float trace_dist_m;
  float map_limit_dist_m;
  uint8_t hit_inside_map = 1U;

  if (dist_m < OGM_RANGE_MIN_M || dist_m > DIST_VALID_MAX_M)
  {
    return;
  }

  world_angle = wrap_pi(map_theta_rad + rel_angle_rad);
  dir_x = cosf(world_angle);
  dir_y = sinf(world_angle);
  sensor_world_x_m = map_x_m +
                     (sensor_x_m * cosf(map_theta_rad)) -
                     (sensor_y_m * sinf(map_theta_rad));
  sensor_world_y_m = map_y_m +
                     (sensor_x_m * sinf(map_theta_rad)) +
                     (sensor_y_m * cosf(map_theta_rad));
  map_limit_dist_m = OGM_RayMapLimitDist(sensor_world_x_m,
                                         sensor_world_y_m,
                                         dir_x,
                                         dir_y);
  trace_dist_m = dist_m;
  if (trace_dist_m > map_limit_dist_m)
  {
    trace_dist_m = map_limit_dist_m;
    hit_inside_map = 0U;
  }
  if (trace_dist_m < OGM_RANGE_MIN_M)
  {
    return;
  }
  hit_x_m = sensor_world_x_m + (trace_dist_m * dir_x);
  hit_y_m = sensor_world_y_m + (trace_dist_m * dir_y);

  if (OGM_WorldToCell(sensor_world_x_m, sensor_world_y_m, &sx, &sy) == 0)
  {
    return;
  }
  if (OGM_WorldToCell(hit_x_m, hit_y_m, &ex, &ey) == 0)
  {
    return;
  }

  OGM_TraceFreeCells(sx, sy, ex, ey, free_delta);
  if (hit_inside_map != 0U && occ_delta != 0)
  {
    OGM_AddCell(ex, ey, occ_delta);
  }

  if (hit_inside_map != 0U && neighbor_delta != 0)
  {
    /* Slightly thicken obstacle hits to reduce jagged single-cell spikes. */
    OGM_AddCell(ex + 1, ey, neighbor_delta);
    OGM_AddCell(ex - 1, ey, neighbor_delta);
    OGM_AddCell(ex, ey + 1, neighbor_delta);
    OGM_AddCell(ex, ey - 1, neighbor_delta);
  }
}

static void OGM_UpdateRayWeighted(float rel_angle_rad,
                                  float dist_m,
                                  float sensor_x_m,
                                  float sensor_y_m,
                                  int free_delta,
                                  int occ_delta,
                                  int neighbor_delta)
{
  OGM_UpdateRayWeightedAtPose(OGM_GetMapX(),
                              OGM_GetMapY(),
                              OGM_GetMapThetaRad(),
                              rel_angle_rad,
                              dist_m,
                              sensor_x_m,
                              sensor_y_m,
                              free_delta,
                              occ_delta,
                              neighbor_delta);
}

static void OGM_UpdateRay(float rel_angle_rad,
                          float dist_m,
                          float sensor_x_m,
                          float sensor_y_m)
{
  OGM_UpdateRayWeighted(rel_angle_rad,
                        dist_m,
                        sensor_x_m,
                        sensor_y_m,
                        OGM_CELL_FREE_DELTA,
                        OGM_CELL_OCC_DELTA,
                        OGM_OCC_NEIGHBOR_DELTA);
}

static float OGM_FilterRangeForMapping(float raw_dist,
                                       uint32_t age_ms,
                                       float *filtered_dist)
{
  float alpha;
  float delta;

  if (filtered_dist == NULL)
  {
    return 0.0f;
  }

  if (age_ms > OGM_RANGE_STALE_MS ||
      raw_dist < OGM_RANGE_MIN_M ||
      raw_dist > DIST_VALID_MAX_M)
  {
    return 0.0f;
  }

  if (*filtered_dist <= 0.0f)
  {
    *filtered_dist = raw_dist;
    return *filtered_dist;
  }

  delta = raw_dist - *filtered_dist;
  if (fabsf(delta) > OGM_FILTER_HARD_JUMP_M)
  {
    return 0.0f;
  }
  alpha = (fabsf(delta) > OGM_FILTER_JUMP_M) ? OGM_FILTER_ALPHA_JUMP : OGM_FILTER_ALPHA_STABLE;
  *filtered_dist += alpha * delta;
  return *filtered_dist;
}

static uint8_t OGM_ScanBinValidMm(uint16_t dist_mm)
{
  float dist_m = (float)dist_mm / 1000.0f;

  return (dist_m >= OGM_SCAN_RANGE_MIN_M && dist_m <= OGM_SCAN_FREE_MAX_M) ? 1U : 0U;
}

static uint16_t OGM_FilterScanBinMm(uint16_t index, uint16_t count)
{
  uint16_t center_mm = ogm_scan_bins_mm[index];
  float center_m = (float)center_mm / 1000.0f;
  float support_diff_m = OGM_SCAN_SUPPORT_DIFF_BASE_M +
                         (center_m * OGM_SCAN_SUPPORT_DIFF_RATIO);
  uint16_t min_support_count = (center_m <= OGM_SCAN_NEAR_SUPPORT_MAX_M) ?
                               OGM_SCAN_NEAR_SUPPORT_MIN_COUNT :
                               OGM_SCAN_SUPPORT_MIN_COUNT;
  uint16_t samples[(OGM_SCAN_SUPPORT_WINDOW * 2U) + 1U];
  uint16_t sample_count = 0U;
  int offset;

  if (count == 0U || OGM_ScanBinValidMm(center_mm) == 0U)
  {
    return 0U;
  }

  for (offset = -(int)OGM_SCAN_SUPPORT_WINDOW;
       offset <= (int)OGM_SCAN_SUPPORT_WINDOW;
       offset++)
  {
    int neighbor = (int)index + offset;
    uint16_t neighbor_mm;
    float neighbor_m;

    while (neighbor < 0)
    {
      neighbor += (int)count;
    }
    while (neighbor >= (int)count)
    {
      neighbor -= (int)count;
    }

    neighbor_mm = ogm_scan_bins_mm[(uint16_t)neighbor];
    if (OGM_ScanBinValidMm(neighbor_mm) == 0U)
    {
      continue;
    }

    neighbor_m = (float)neighbor_mm / 1000.0f;
    if (fabsf(neighbor_m - center_m) <= support_diff_m)
    {
      samples[sample_count] = neighbor_mm;
      sample_count++;
    }
  }

  if (sample_count < min_support_count)
  {
    return 0U;
  }

  for (uint16_t i = 1U; i < sample_count; i++)
  {
    uint16_t key = samples[i];
    uint16_t j = i;
    while (j > 0U && samples[j - 1U] > key)
    {
      samples[j] = samples[j - 1U];
      j--;
    }
    samples[j] = key;
  }

  return samples[sample_count / 2U];
}

static void OGM_CorrectGridHeadingFromSideWalls(uint16_t count)
{
#if OGM_WALL_HEADING_CORRECT_ENABLE && OGM_GRID_HEADING_ENABLE && MAZE_SIMPLE_RULE_ENABLE
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_xx = 0.0f;
  float sum_xy = 0.0f;
  uint16_t n = 0U;
  float den;
  float slope;
  float wall_angle_rad;
  float step_rad;

  if (OGM_MazeGridModeActive() == 0U || count == 0U)
  {
    return;
  }

  if (maze_simple_state == MAZE_SIMPLE_TURN_RIGHT ||
      maze_simple_state == MAZE_SIMPLE_TURN_LEFT ||
      maze_simple_state == MAZE_SIMPLE_TURN_BACK ||
      maze_simple_state == MAZE_SIMPLE_POST_TURN_LOCK ||
      maze_simple_state == MAZE_SIMPLE_POST_FORWARD)
  {
    return;
  }

  for (uint16_t i = 0U; i < count; i++)
  {
    float dist_m;
    float lidar_angle_deg;
    float rel_angle_rad;
    float x_m;
    float y_m;

    if (ogm_scan_filtered_bins_mm[i] == 0U)
    {
      continue;
    }

    dist_m = (float)ogm_scan_filtered_bins_mm[i] / 1000.0f;
    if (dist_m < OGM_WALL_HEADING_MIN_SIDE_M ||
        dist_m > OGM_WALL_HEADING_MAX_SIDE_M)
    {
      continue;
    }

    lidar_angle_deg = ((((float)i) + 0.5f) * OGM_SCAN_BIN_DEG) + OGM_LIDAR_YAW_OFFSET_DEG;
    rel_angle_rad = wrap_pi(-deg_to_rad(lidar_angle_deg));
    x_m = dist_m * cosf(rel_angle_rad);
    y_m = dist_m * sinf(rel_angle_rad);

    if (fabsf(x_m) > OGM_WALL_HEADING_MAX_ABS_X_M ||
        fabsf(y_m) < OGM_WALL_HEADING_MIN_SIDE_M)
    {
      continue;
    }

    sum_x += x_m;
    sum_y += y_m;
    sum_xx += x_m * x_m;
    sum_xy += x_m * y_m;
    n++;
  }

  if (n < OGM_WALL_HEADING_MIN_POINTS)
  {
    return;
  }

  den = ((float)n * sum_xx) - (sum_x * sum_x);
  if (fabsf(den) < 0.0001f)
  {
    return;
  }

  slope = (((float)n * sum_xy) - (sum_x * sum_y)) / den;
  wall_angle_rad = atanf(slope);
  if (fabsf(wall_angle_rad) > 0.20f)
  {
    return;
  }

  step_rad = wall_angle_rad * OGM_WALL_HEADING_GAIN;
  if (step_rad > OGM_WALL_HEADING_MAX_STEP_RAD)
  {
    step_rad = OGM_WALL_HEADING_MAX_STEP_RAD;
  }
  else if (step_rad < -OGM_WALL_HEADING_MAX_STEP_RAD)
  {
    step_rad = -OGM_WALL_HEADING_MAX_STEP_RAD;
  }

  if (fabsf(step_rad) >= 0.0002f)
  {
    OGM_EnsureGridHeading(odom_theta_rad);
    ogm_grid_heading_rad = wrap_pi(ogm_grid_heading_rad + step_rad);
  }
#else
  (void)count;
#endif
}

static uint8_t OGM_UpdateFromScanBins(void)
{
  uint32_t now_tick = HAL_GetTick();
  uint32_t scan_tick = Lidar_GetScanTick();
  uint16_t count;
  uint16_t i;
  uint16_t raw_valid_bins = 0U;
  uint16_t valid_bins = 0U;
  uint16_t far_bins = 0U;
  int free_delta = OGM_SCAN_FREE_DELTA;
  int occ_delta = OGM_SCAN_OCC_DELTA;
  uint32_t scan_pose_tick;
  float scan_map_x_m;
  float scan_map_y_m;
  float scan_map_theta_rad;

#if OGM_MAP_REVERSE_BACKUP_ENABLE && MAZE_SIMPLE_RULE_ENABLE
  if (maze_simple_state == MAZE_SIMPLE_BACKUP)
  {
    free_delta = OGM_SCAN_BACKUP_FREE_DELTA;
    occ_delta = OGM_SCAN_BACKUP_OCC_DELTA;
  }
#endif

  ogm_dbg_scan_used = 0U;
  ogm_dbg_scan_tick = scan_tick;
  ogm_dbg_scan_masked_count = Lidar_GetScanMaskedCount();

  if (scan_tick == 0U)
  {
    ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_NO_SCAN;
    return 0U;
  }
  if (scan_tick == ogm_last_scan_tick_mapped)
  {
    ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_DUPLICATE;
    return 0U;
  }
  if ((now_tick - scan_tick) > OGM_SCAN_STALE_MS)
  {
    ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_STALE;
    return 0U;
  }

  scan_map_x_m = OGM_GetMapX();
  scan_map_y_m = OGM_GetMapY();
  scan_map_theta_rad = OGM_GetMapThetaRad();
  if (OGM_MazeGridModeActive() == 0U)
  {
    scan_pose_tick = (scan_tick > OGM_SCAN_POSE_DELAY_MS) ?
                     (scan_tick - OGM_SCAN_POSE_DELAY_MS) :
                     scan_tick;
    (void)OGM_GetPoseAtTick(scan_pose_tick,
                            &scan_map_x_m,
                            &scan_map_y_m,
                            &scan_map_theta_rad);
  }

  count = Lidar_CopyScanBinsMm(ogm_scan_bins_mm,
                               (uint16_t)LIDAR_SCAN_BIN_COUNT);
  if (count == 0U)
  {
    ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_COPY_EMPTY;
    return 0U;
  }

  for (i = 0U; i < count; i++)
  {
    if (OGM_ScanBinValidMm(ogm_scan_bins_mm[i]) != 0U)
    {
      raw_valid_bins++;
    }

    ogm_scan_filtered_bins_mm[i] = OGM_FilterScanBinMm(i, count);

    if (ogm_scan_filtered_bins_mm[i] != 0U)
    {
      valid_bins++;
    }
  }
  ogm_dbg_scan_raw_valid_bins = raw_valid_bins;
  ogm_dbg_scan_filtered_bins = valid_bins;
  ogm_dbg_scan_far_bins = 0U;

  if (valid_bins < OGM_SCAN_MIN_VALID_BINS)
  {
    ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_TOO_FEW_BINS;
    return 0U;
  }

  for (i = 0U; i < count; i++)
  {
    float dist_m = (float)ogm_scan_filtered_bins_mm[i] / 1000.0f;

    if (ogm_scan_filtered_bins_mm[i] != 0U)
    {
      float lidar_angle_deg = ((((float)i) + 0.5f) * OGM_SCAN_BIN_DEG) + OGM_LIDAR_YAW_OFFSET_DEG;
      float rel_angle_rad = wrap_pi(-deg_to_rad(lidar_angle_deg));

      if (dist_m <= OGM_SCAN_OCC_MAX_M)
      {
        OGM_UpdateRayWeightedAtPose(scan_map_x_m,
                                    scan_map_y_m,
                                    scan_map_theta_rad,
                                    rel_angle_rad,
                                    dist_m,
                                    OGM_LIDAR_ORIGIN_X_M,
                                    OGM_LIDAR_ORIGIN_Y_M,
                                    free_delta,
                                    occ_delta,
                                    OGM_SCAN_OCC_NEIGHBOR_DELTA);
      }
      else
      {
        far_bins++;
        OGM_UpdateRayWeightedAtPose(scan_map_x_m,
                                    scan_map_y_m,
                                    scan_map_theta_rad,
                                    rel_angle_rad,
                                    dist_m,
                                    OGM_LIDAR_ORIGIN_X_M,
                                    OGM_LIDAR_ORIGIN_Y_M,
                                    free_delta,
                                    0,
                                    0);
      }
    }
  }
  ogm_dbg_scan_far_bins = far_bins;
  OGM_CorrectGridHeadingFromSideWalls(count);
  OGM_CorrectMapPoseFromCardinalWalls(now_tick);

  ogm_last_scan_tick_mapped = scan_tick;
  ogm_dbg_scan_used = 1U;
  ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_NONE;
  ogm_dbg_scan_update_count++;
  return 1U;
}

static void OGM_UpdateFromRanges(void)
{
  uint32_t now_tick = HAL_GetTick();

  if (OGM_UpdateFromScanBins() != 0U)
  {
    return;
  }

  float map_front = OGM_FilterRangeForMapping(fordist,
                                              now_tick - fordist_last_valid_tick,
                                              &ogm_fordist_f);
  float map_right = OGM_FilterRangeForMapping(rightdist,
                                              now_tick - rightdist_last_valid_tick,
                                              &ogm_rightdist_f);
  float map_right_front = OGM_FilterRangeForMapping(rightfrontdist,
                                                    now_tick - rightfrontdist_last_valid_tick,
                                                    &ogm_rightfrontdist_f);
  float map_right_rear = OGM_FilterRangeForMapping(rightreardist,
                                                   now_tick - rightreardist_last_valid_tick,
                                                   &ogm_rightreardist_f);
  float map_rear = OGM_FilterRangeForMapping(reardist,
                                             now_tick - reardist_last_valid_tick,
                                             &ogm_reardist_f);
  float map_left_rear = OGM_FilterRangeForMapping(leftreardist,
                                                  now_tick - leftreardist_last_valid_tick,
                                                  &ogm_leftreardist_f);
  float map_left = OGM_FilterRangeForMapping(leftdist,
                                             now_tick - leftdist_last_valid_tick,
                                             &ogm_leftdist_f);
  float map_left_front = OGM_FilterRangeForMapping(leftfrontdist,
                                                   now_tick - leftfrontdist_last_valid_tick,
                                                   &ogm_leftfrontdist_f);

  OGM_UpdateRay(0.0f,
                map_front,
                OGM_LIDAR_ORIGIN_X_M,
                OGM_LIDAR_ORIGIN_Y_M);
  OGM_UpdateRay(-PI_F * 0.5f,
                map_right,
                OGM_LIDAR_ORIGIN_X_M,
                OGM_LIDAR_ORIGIN_Y_M);
#if OGM_MAP_RIGHT_FRONT_ENABLE
  if (map_right_front >= OGM_DIAGONAL_MAP_MIN_M)
  {
    OGM_UpdateRayWeighted(-PI_F * 0.25f,
                          map_right_front,
                          OGM_LIDAR_ORIGIN_X_M,
                          OGM_LIDAR_ORIGIN_Y_M,
                          OGM_DIAGONAL_FREE_DELTA,
                          OGM_DIAGONAL_OCC_DELTA,
                          0);
  }
  if (map_right_rear >= OGM_DIAGONAL_MAP_MIN_M)
  {
    OGM_UpdateRayWeighted(-PI_F * 0.75f,
                          map_right_rear,
                          OGM_LIDAR_ORIGIN_X_M,
                          OGM_LIDAR_ORIGIN_Y_M,
                          OGM_DIAGONAL_FREE_DELTA,
                          OGM_DIAGONAL_OCC_DELTA,
                          0);
  }
  if (map_left_rear >= OGM_DIAGONAL_MAP_MIN_M)
  {
    OGM_UpdateRayWeighted(PI_F * 0.75f,
                          map_left_rear,
                          OGM_LIDAR_ORIGIN_X_M,
                          OGM_LIDAR_ORIGIN_Y_M,
                          OGM_DIAGONAL_FREE_DELTA,
                          OGM_DIAGONAL_OCC_DELTA,
                          0);
  }
  if (map_left_front >= OGM_DIAGONAL_MAP_MIN_M)
  {
    OGM_UpdateRayWeighted(PI_F * 0.25f,
                          map_left_front,
                          OGM_LIDAR_ORIGIN_X_M,
                          OGM_LIDAR_ORIGIN_Y_M,
                          OGM_DIAGONAL_FREE_DELTA,
                          OGM_DIAGONAL_OCC_DELTA,
                          0);
  }
#else
  (void)map_right_front;
  (void)map_right_rear;
  (void)map_left_rear;
  (void)map_left_front;
#endif
  OGM_UpdateRay(PI_F,
                map_rear,
                OGM_LIDAR_ORIGIN_X_M,
                OGM_LIDAR_ORIGIN_Y_M);
  OGM_UpdateRay(PI_F * 0.5f,
                map_left,
                OGM_LIDAR_ORIGIN_X_M,
                OGM_LIDAR_ORIGIN_Y_M);
  OGM_CorrectMapPoseFromCardinalWalls(now_tick);
}

static uint8_t OGM_CommandIsStraightMotion(uint8_t *reverse_motion)
{
  if (reverse_motion != NULL)
  {
    *reverse_motion = 0U;
  }

  if (fabsf(maze_prev_left_cmd) <= OGM_STOP_CMD_EPS_PWM ||
      fabsf(maze_prev_right_cmd) <= OGM_STOP_CMD_EPS_PWM ||
      fabsf(maze_prev_left_cmd - maze_prev_right_cmd) > 90.0f)
  {
    return 0U;
  }

  if (maze_prev_left_cmd > OGM_STOP_CMD_EPS_PWM &&
      maze_prev_right_cmd > OGM_STOP_CMD_EPS_PWM)
  {
    return 1U;
  }

  if (maze_prev_left_cmd < -OGM_STOP_CMD_EPS_PWM &&
      maze_prev_right_cmd < -OGM_STOP_CMD_EPS_PWM)
  {
    if (reverse_motion != NULL)
    {
      *reverse_motion = 1U;
    }
    return 1U;
  }

  return 0U;
}

static uint8_t maze_simple_state_allows_mapping(void)
{
  if (maze_centerline_phase != MAZE_CENTERLINE_IDLE)
  {
    return 0U;
  }

  switch (maze_simple_state)
  {
    case MAZE_SIMPLE_FOLLOW:
    case MAZE_SIMPLE_START_STRAIGHT:
    case MAZE_SIMPLE_APPROACH_JUNCTION:
    case MAZE_SIMPLE_CENTER_ENTRY:
    case MAZE_SIMPLE_PRE_TURN_FORWARD:
      return 1U;

    case MAZE_SIMPLE_POST_FORWARD:
    case MAZE_SIMPLE_STOP_SCAN:
    case MAZE_SIMPLE_POST_TURN_LOCK:
    case MAZE_SIMPLE_START_ALIGN:
    case MAZE_SIMPLE_YAW_RECOVER:
    case MAZE_SIMPLE_TURN_RIGHT:
    case MAZE_SIMPLE_TURN_LEFT:
    case MAZE_SIMPLE_TURN_BACK:
    case MAZE_SIMPLE_BACKUP:
#if OGM_MAP_REVERSE_BACKUP_ENABLE
      return (maze_simple_state == MAZE_SIMPLE_BACKUP) ? 1U : 0U;
#else
      return 0U;
#endif
    default:
      return 0U;
  }
}

static uint8_t maze_simple_state_allows_stop_mapping(void)
{
  if (maze_centerline_phase != MAZE_CENTERLINE_IDLE)
  {
    return 0U;
  }

  switch (maze_simple_state)
  {
    case MAZE_SIMPLE_FOLLOW:
    case MAZE_SIMPLE_START_STRAIGHT:
    case MAZE_SIMPLE_APPROACH_JUNCTION:
    case MAZE_SIMPLE_CENTER_ENTRY:
    case MAZE_SIMPLE_STOP_SCAN:
      return 1U;

    case MAZE_SIMPLE_PRE_TURN_FORWARD:
    case MAZE_SIMPLE_POST_TURN_LOCK:
    case MAZE_SIMPLE_START_ALIGN:
    case MAZE_SIMPLE_YAW_RECOVER:
    case MAZE_SIMPLE_TURN_RIGHT:
    case MAZE_SIMPLE_TURN_LEFT:
    case MAZE_SIMPLE_TURN_BACK:
    case MAZE_SIMPLE_POST_FORWARD:
    case MAZE_SIMPLE_BACKUP:
    default:
      return 0U;
  }
}

static void OGM_UpdatePeriodic(void)
{
  static uint32_t last_update_tick = 0U;
  static uint32_t stop_start_tick = 0U;
  static uint32_t last_stop_scan_tick = 0U;
  static uint8_t pose_inited = 0U;
  static float last_x_m = 0.0f;
  static float last_y_m = 0.0f;
  static float last_theta_rad = 0.0f;
  uint32_t now_tick = HAL_GetTick();
  float map_theta_rad;
  float dx;
  float dy;
  float ds_m;
  float dtheta;
  uint8_t reverse_motion = 0U;
  uint8_t straight_motion;
  uint8_t motion_allows_mapping;

  if (lidar_is_ready_for_runtime() == 0U)
  {
    ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_NO_SCAN;
    pose_inited = 0U;
    stop_start_tick = 0U;
    ogm_fordist_f = 0.0f;
    ogm_rightdist_f = 0.0f;
    ogm_rightfrontdist_f = 0.0f;
    ogm_rightreardist_f = 0.0f;
    ogm_reardist_f = 0.0f;
    ogm_leftreardist_f = 0.0f;
    ogm_leftdist_f = 0.0f;
    ogm_leftfrontdist_f = 0.0f;
    return;
  }

  if (imu_is_ready_for_runtime() == 0U)
  {
    ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_IMU;
    pose_inited = 0U;
    stop_start_tick = 0U;
    ogm_heading_ref_valid = 0U;
    ogm_fordist_f = 0.0f;
    ogm_rightdist_f = 0.0f;
    ogm_rightfrontdist_f = 0.0f;
    ogm_rightreardist_f = 0.0f;
    ogm_reardist_f = 0.0f;
    ogm_leftreardist_f = 0.0f;
    ogm_leftdist_f = 0.0f;
    ogm_leftfrontdist_f = 0.0f;
    return;
  }

  /* The sensor is mounted with roll near +/-180 deg; yaw is still the best map heading. */
  map_theta_rad = OGM_GetMapThetaRad();
  OGM_EnsureMapPose();

  if (fabsf(maze_prev_left_cmd) <= OGM_STOP_CMD_EPS_PWM &&
      fabsf(maze_prev_right_cmd) <= OGM_STOP_CMD_EPS_PWM)
  {
    if (stop_start_tick == 0U)
    {
      stop_start_tick = now_tick;
    }

    last_update_tick = now_tick;
    last_x_m = odom_x_m;
    last_y_m = odom_y_m;
    last_theta_rad = map_theta_rad;
    pose_inited = 1U;
    OGM_RecordPoseSample(now_tick, OGM_GetMapX(), OGM_GetMapY(), map_theta_rad);

    if (maze_simple_state_allows_stop_mapping() != 0U &&
        OGM_AttitudeStableForMapping() != 0U &&
        (now_tick - stop_start_tick) >= OGM_STOP_SCAN_SETTLE_MS &&
        (now_tick - last_stop_scan_tick) >= OGM_STOP_SCAN_INTERVAL_MS)
    {
      last_stop_scan_tick = now_tick;
      OGM_UpdateFromRanges();
    }
    else if (maze_simple_state_allows_stop_mapping() == 0U)
    {
      ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_STATE;
    }
    else if (OGM_AttitudeStableForMapping() == 0U)
    {
      ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_IMU;
    }
    else
    {
      ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_STOP_WAIT;
    }
    return;
  }

  stop_start_tick = 0U;
  straight_motion = OGM_CommandIsStraightMotion(&reverse_motion);
  motion_allows_mapping = (straight_motion != 0U &&
                           OGM_AttitudeStableForMapping() != 0U &&
                           maze_simple_state_allows_mapping() != 0U &&
                           (reverse_motion == 0U
#if OGM_MAP_REVERSE_BACKUP_ENABLE && MAZE_SIMPLE_RULE_ENABLE
                            || maze_simple_state == MAZE_SIMPLE_BACKUP
#endif
                           )) ? 1U : 0U;

  if (pose_inited == 0U)
  {
    last_x_m = odom_x_m;
    last_y_m = odom_y_m;
    last_theta_rad = map_theta_rad;
    pose_inited = 1U;
  }

  if ((now_tick - last_update_tick) < OGM_UPDATE_INTERVAL_MS)
  {
    ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_INTERVAL;
    return;
  }

  dx = odom_x_m - last_x_m;
  dy = odom_y_m - last_y_m;
  ds_m = sqrtf((dx * dx) + (dy * dy));
  dtheta = fabsf(wrap_pi(map_theta_rad - last_theta_rad));
  if (ds_m < OGM_MIN_UPDATE_DS_M && dtheta < OGM_MIN_UPDATE_DTHETA_RAD)
  {
    ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_SMALL_DELTA;
    return;
  }

  if (dtheta > OGM_MAPPING_MAX_DTHETA_RAD)
  {
    motion_allows_mapping = 0U;
    ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_TURNING;
  }

  last_update_tick = now_tick;
  last_x_m = odom_x_m;
  last_y_m = odom_y_m;
  last_theta_rad = map_theta_rad;
  if (straight_motion != 0U)
  {
    OGM_AdvanceMapPose((reverse_motion != 0U) ? -ds_m : ds_m, map_theta_rad);
  }
  OGM_RecordPoseSample(now_tick, OGM_GetMapX(), OGM_GetMapY(), map_theta_rad);

  if (motion_allows_mapping != 0U)
  {
    OGM_UpdateFromRanges();
  }
  else if (dtheta <= OGM_MAPPING_MAX_DTHETA_RAD)
  {
    ogm_dbg_scan_skip_reason = OGM_SCAN_SKIP_STATE;
  }
}

static void Odom_UpdatePeriodic(void)
{
  static uint32_t last_update_tick = 0U;
  uint32_t now_tick = HAL_GetTick();
  int32_t pulse_left = 0;
  int32_t pulse_right = 0;

  if (lidar_is_ready_for_runtime() == 0U)
  {
    return;
  }

  if ((now_tick - last_update_tick) < 20U)
  {
    return;
  }
  last_update_tick = now_tick;

  __disable_irq();
  pulse_left = odom_pulse_accum_left;
  pulse_right = odom_pulse_accum_right;
  odom_pulse_accum_left = 0;
  odom_pulse_accum_right = 0;
  __enable_irq();

  Odom_UpdateFromPulses((float)pulse_left, (float)pulse_right);
}

static void OGM_GetStats(uint16_t *unknown_count, uint16_t *free_count, uint16_t *occ_count)
{
  uint16_t u = 0U;
  uint16_t f = 0U;
  uint16_t o = 0U;
  int x;
  int y;

  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      int8_t v = ogm_grid[y][x];
      if (v > 0)
      {
        o++;
      }
      else if (v < 0)
      {
        f++;
      }
      else
      {
        u++;
      }
    }
  }

  if (unknown_count != NULL)
  {
    *unknown_count = u;
  }
  if (free_count != NULL)
  {
    *free_count = f;
  }
  if (occ_count != NULL)
  {
    *occ_count = o;
  }
}

static char OGM_CellToChar(int8_t v)
{
  if (v >= OGM_DISPLAY_OCC_STRONG)
  {
    return '#';
  }
  if (v >= OGM_DISPLAY_OCC_MIN)
  {
    return '+';
  }
  if (v <= OGM_DISPLAY_FREE_STRONG)
  {
    return '.';
  }
  if (v <= OGM_DISPLAY_FREE_MIN)
  {
    return ',';
  }
  return ' ';
}

static uint8_t OGM_IsDisplayOccCell(int x, int y)
{
  if (x < 0 || x >= OGM_WIDTH || y < 0 || y >= OGM_HEIGHT)
  {
    return 0U;
  }

  return (ogm_grid[y][x] >= OGM_DISPLAY_OCC_MIN) ? 1U : 0U;
}

static uint8_t OGM_IsOccEvidenceCell(int x, int y)
{
  if (x < 0 || x >= OGM_WIDTH || y < 0 || y >= OGM_HEIGHT)
  {
    return 0U;
  }

  return (ogm_grid[y][x] >= OGM_DISPLAY_OCC_EVIDENCE_MIN) ? 1U : 0U;
}

static uint8_t OGM_IsFreeEvidenceCell(int x, int y)
{
  if (x < 0 || x >= OGM_WIDTH || y < 0 || y >= OGM_HEIGHT)
  {
    return 0U;
  }

  return (ogm_grid[y][x] <= OGM_DISPLAY_FREE_MIN) ? 1U : 0U;
}

static uint8_t OGM_CountOccNeighbors(int x, int y)
{
  uint8_t count = 0U;
  int dx;
  int dy;

  for (dy = -1; dy <= 1; dy++)
  {
    for (dx = -1; dx <= 1; dx++)
    {
      if (dx == 0 && dy == 0)
      {
        continue;
      }
      if (OGM_IsOccEvidenceCell(x + dx, y + dy) != 0U)
      {
        count++;
      }
    }
  }

  return count;
}

static uint8_t OGM_CountFreeNeighbors(int x, int y)
{
  uint8_t count = 0U;
  int dx;
  int dy;

  for (dy = -1; dy <= 1; dy++)
  {
    for (dx = -1; dx <= 1; dx++)
    {
      if (dx == 0 && dy == 0)
      {
        continue;
      }
      if (OGM_IsFreeEvidenceCell(x + dx, y + dy) != 0U)
      {
        count++;
      }
    }
  }

  return count;
}

static uint8_t OGM_HasLineOccEvidence(int x, int y)
{
  uint8_t bridge_h = (OGM_IsOccEvidenceCell(x - 1, y) != 0U &&
                      OGM_IsOccEvidenceCell(x + 1, y) != 0U) ? 1U : 0U;
  uint8_t bridge_v = (OGM_IsOccEvidenceCell(x, y - 1) != 0U &&
                      OGM_IsOccEvidenceCell(x, y + 1) != 0U) ? 1U : 0U;
#if OGM_DISPLAY_DIAGONAL_BRIDGE_ENABLE
  uint8_t bridge_d1 = (OGM_IsOccEvidenceCell(x - 1, y - 1) != 0U &&
                       OGM_IsOccEvidenceCell(x + 1, y + 1) != 0U) ? 1U : 0U;
  uint8_t bridge_d2 = (OGM_IsOccEvidenceCell(x - 1, y + 1) != 0U &&
                       OGM_IsOccEvidenceCell(x + 1, y - 1) != 0U) ? 1U : 0U;

  return (bridge_h != 0U || bridge_v != 0U || bridge_d1 != 0U || bridge_d2 != 0U) ? 1U : 0U;
#else
  return (bridge_h != 0U || bridge_v != 0U) ? 1U : 0U;
#endif
}

static char OGM_CellToDisplayChar(int x, int y)
{
  int8_t v;
  uint8_t occ_neighbors;
  uint8_t free_neighbors;

  if (x < 0 || x >= OGM_WIDTH || y < 0 || y >= OGM_HEIGHT)
  {
    return ' ';
  }

  v = ogm_grid[y][x];
  free_neighbors = OGM_CountFreeNeighbors(x, y);
  if (v >= OGM_DISPLAY_OCC_MIN)
  {
    occ_neighbors = OGM_CountOccNeighbors(x, y);
#if OGM_DISPLAY_OCC_SURFACE_ONLY
    if (free_neighbors == 0U)
    {
      return ' ';
    }
#endif
    if (occ_neighbors == 0U &&
        OGM_HasLineOccEvidence(x, y) == 0U)
    {
      return ' ';
    }
    if (v < OGM_DISPLAY_OCC_STRONG &&
        occ_neighbors < 2U &&
        OGM_HasLineOccEvidence(x, y) == 0U)
    {
      return ' ';
    }
#if OGM_DISPLAY_WEAK_OCC_REQUIRE_FREE
    if (v < OGM_DISPLAY_OCC_STRONG && free_neighbors == 0U)
    {
      return ' ';
    }
#endif
  }
  else if (v >= OGM_DISPLAY_OCC_EVIDENCE_MIN)
  {
    if (OGM_HasLineOccEvidence(x, y) != 0U
#if OGM_DISPLAY_WEAK_OCC_REQUIRE_FREE
        && free_neighbors != 0U
#endif
       )
    {
      return '+';
    }
    return ' ';
  }
  else
  {
    if (v > OGM_DISPLAY_FREE_MIN)
    {
      if (OGM_HasLineOccEvidence(x, y) != 0U)
      {
        return '+';
      }
    }
  }

  return OGM_CellToChar(v);
}

static uint8_t OGM_DisplayCharIsOcc(char c)
{
  return (c == '+' || c == '#') ? 1U : 0U;
}

static uint8_t OGM_DisplayCharIsFree(char c)
{
  return (c == ',' || c == '.') ? 1U : 0U;
}

static uint8_t OGM_DisplayHasOccNeighbor(char map[OGM_HEIGHT][OGM_WIDTH], int x, int y)
{
  int dx;
  int dy;

  for (dy = -1; dy <= 1; dy++)
  {
    for (dx = -1; dx <= 1; dx++)
    {
      if (dx == 0 && dy == 0)
      {
        continue;
      }
      if (x + dx < 0 || x + dx >= OGM_WIDTH || y + dy < 0 || y + dy >= OGM_HEIGHT)
      {
        continue;
      }
      if (OGM_DisplayCharIsOcc(map[y + dy][x + dx]) != 0U)
      {
        return 1U;
      }
    }
  }

  return 0U;
}

static uint8_t OGM_DisplayHasFreeNeighbor(char map[OGM_HEIGHT][OGM_WIDTH], int x, int y)
{
  int dx;
  int dy;

  for (dy = -1; dy <= 1; dy++)
  {
    for (dx = -1; dx <= 1; dx++)
    {
      if (dx == 0 && dy == 0)
      {
        continue;
      }
      if (x + dx < 0 || x + dx >= OGM_WIDTH || y + dy < 0 || y + dy >= OGM_HEIGHT)
      {
        continue;
      }
      if (OGM_DisplayCharIsFree(map[y + dy][x + dx]) != 0U)
      {
        return 1U;
      }
    }
  }

  return 0U;
}

static uint8_t OGM_DisplayCountFreeNeighbors(char map[OGM_HEIGHT][OGM_WIDTH], int x, int y)
{
  uint8_t count = 0U;
  int dx;
  int dy;

  for (dy = -1; dy <= 1; dy++)
  {
    for (dx = -1; dx <= 1; dx++)
    {
      if (dx == 0 && dy == 0)
      {
        continue;
      }
      if (x + dx < 0 || x + dx >= OGM_WIDTH || y + dy < 0 || y + dy >= OGM_HEIGHT)
      {
        continue;
      }
      if (OGM_DisplayCharIsFree(map[y + dy][x + dx]) != 0U)
      {
        count++;
      }
    }
  }

  return count;
}

static uint8_t OGM_DisplayShouldBridge(char map[OGM_HEIGHT][OGM_WIDTH], int x, int y)
{
  if (x <= 0 || x >= (OGM_WIDTH - 1) || y <= 0 || y >= (OGM_HEIGHT - 1))
  {
    return 0U;
  }

  if (OGM_DisplayCharIsOcc(map[y][x - 1]) != 0U &&
      OGM_DisplayCharIsOcc(map[y][x + 1]) != 0U)
  {
    return 1U;
  }
  if (OGM_DisplayCharIsOcc(map[y - 1][x]) != 0U &&
      OGM_DisplayCharIsOcc(map[y + 1][x]) != 0U)
  {
    return 1U;
  }
#if OGM_DISPLAY_DIAGONAL_BRIDGE_ENABLE
  if (OGM_DisplayCharIsOcc(map[y - 1][x - 1]) != 0U &&
      OGM_DisplayCharIsOcc(map[y + 1][x + 1]) != 0U)
  {
    return 1U;
  }
  if (OGM_DisplayCharIsOcc(map[y + 1][x - 1]) != 0U &&
      OGM_DisplayCharIsOcc(map[y - 1][x + 1]) != 0U)
  {
    return 1U;
  }
#endif

  return 0U;
}

static uint8_t OGM_DisplayHasOccInDir(char map[OGM_HEIGHT][OGM_WIDTH],
                                      int x,
                                      int y,
                                      int dx,
                                      int dy,
                                      int max_step)
{
  int step;

  for (step = 1; step <= max_step; step++)
  {
    int nx = x + (dx * step);
    int ny = y + (dy * step);
    if (nx < 0 || nx >= OGM_WIDTH || ny < 0 || ny >= OGM_HEIGHT)
    {
      return 0U;
    }
    if (OGM_DisplayCharIsOcc(map[ny][nx]) != 0U)
    {
      return 1U;
    }
  }

  return 0U;
}

static uint8_t OGM_DisplayShouldBridgeAxisWide(char map[OGM_HEIGHT][OGM_WIDTH], int x, int y)
{
  if (x <= 1 || x >= (OGM_WIDTH - 2) || y <= 1 || y >= (OGM_HEIGHT - 2))
  {
    return 0U;
  }

  if (OGM_DisplayHasOccInDir(map, x, y, -1, 0, 3) != 0U &&
      OGM_DisplayHasOccInDir(map, x, y, 1, 0, 3) != 0U)
  {
    return 1U;
  }

  if (OGM_DisplayHasOccInDir(map, x, y, 0, -1, 3) != 0U &&
      OGM_DisplayHasOccInDir(map, x, y, 0, 1, 3) != 0U)
  {
    return 1U;
  }

  return 0U;
}

static uint8_t OGM_DisplayShouldBridgeTightAxisGap(char map[OGM_HEIGHT][OGM_WIDTH], int x, int y)
{
  if (x <= 0 || x >= (OGM_WIDTH - 1) || y <= 0 || y >= (OGM_HEIGHT - 1))
  {
    return 0U;
  }

  if (OGM_DisplayCharIsOcc(map[y][x - 1]) != 0U &&
      OGM_DisplayCharIsOcc(map[y][x + 1]) != 0U)
  {
    return 1U;
  }
  if (OGM_DisplayCharIsOcc(map[y - 1][x]) != 0U &&
      OGM_DisplayCharIsOcc(map[y + 1][x]) != 0U)
  {
    return 1U;
  }

  if (x > 1 &&
      map[y][x - 1] == ' ' &&
      OGM_DisplayCharIsOcc(map[y][x - 2]) != 0U &&
      OGM_DisplayCharIsOcc(map[y][x + 1]) != 0U)
  {
    return 1U;
  }
  if (x < (OGM_WIDTH - 2) &&
      map[y][x + 1] == ' ' &&
      OGM_DisplayCharIsOcc(map[y][x - 1]) != 0U &&
      OGM_DisplayCharIsOcc(map[y][x + 2]) != 0U)
  {
    return 1U;
  }

  if (y > 1 &&
      map[y - 1][x] == ' ' &&
      OGM_DisplayCharIsOcc(map[y - 2][x]) != 0U &&
      OGM_DisplayCharIsOcc(map[y + 1][x]) != 0U)
  {
    return 1U;
  }
  if (y < (OGM_HEIGHT - 2) &&
      map[y + 1][x] == ' ' &&
      OGM_DisplayCharIsOcc(map[y - 1][x]) != 0U &&
      OGM_DisplayCharIsOcc(map[y + 2][x]) != 0U)
  {
    return 1U;
  }

  return 0U;
}

static void OGM_PostProcessDisplayMap(char map[OGM_HEIGHT][OGM_WIDTH])
{
  int x;
  int y;

  memcpy(ogm_display_src, map, sizeof(ogm_display_src));
  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (map[y][x] == ' ' &&
          OGM_DisplayShouldBridgeTightAxisGap(ogm_display_src, x, y) != 0U)
      {
        map[y][x] = '+';
      }
    }
  }

  memcpy(ogm_display_src, map, sizeof(ogm_display_src));
  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (map[y][x] == ' ' &&
          OGM_DisplayHasFreeNeighbor(ogm_display_src, x, y) != 0U &&
          OGM_DisplayShouldBridge(ogm_display_src, x, y) != 0U)
      {
        map[y][x] = '+';
      }
    }
  }

  memcpy(ogm_display_src, map, sizeof(ogm_display_src));
  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (map[y][x] == ' ' &&
          OGM_DisplayHasFreeNeighbor(ogm_display_src, x, y) != 0U &&
          OGM_DisplayShouldBridgeAxisWide(ogm_display_src, x, y) != 0U)
      {
        map[y][x] = '+';
      }
    }
  }

  memcpy(ogm_display_src, map, sizeof(ogm_display_src));
  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (OGM_DisplayCharIsOcc(map[y][x]) != 0U &&
          OGM_DisplayHasOccNeighbor(ogm_display_src, x, y) == 0U)
      {
        map[y][x] = ' ';
      }
    }
  }

  memcpy(ogm_display_src, map, sizeof(ogm_display_src));
  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (OGM_DisplayCharIsFree(map[y][x]) != 0U &&
          (OGM_DisplayHasFreeNeighbor(ogm_display_src, x, y) == 0U ||
           (OGM_DisplayHasOccNeighbor(ogm_display_src, x, y) == 0U &&
            OGM_DisplayCountFreeNeighbors(ogm_display_src, x, y) < 3U)))
      {
        map[y][x] = ' ';
      }
    }
  }

  memcpy(ogm_display_src, map, sizeof(ogm_display_src));
  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (OGM_DisplayCharIsOcc(map[y][x]) != 0U &&
          OGM_DisplayCountFreeNeighbors(ogm_display_src, x, y) == 0U)
      {
        map[y][x] = ' ';
      }
    }
  }

  memcpy(ogm_display_src, map, sizeof(ogm_display_src));
  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (OGM_DisplayCharIsFree(map[y][x]) != 0U &&
          OGM_DisplayCountFreeNeighbors(ogm_display_src, x, y) < 2U)
      {
        map[y][x] = ' ';
      }
    }
  }
}

static uint8_t OGM_NavCharIsWall(char c)
{
  return (c == '#') ? 1U : 0U;
}

static uint8_t OGM_NavCharIsFree(char c)
{
  return (c == '.') ? 1U : 0U;
}

static uint8_t OGM_NavWallAxisNeighborCount(char map[OGM_HEIGHT][OGM_WIDTH], int x, int y)
{
  uint8_t count = 0U;

  if (x > 0 && OGM_NavCharIsWall(map[y][x - 1]) != 0U)
  {
    count++;
  }
  if (x < (OGM_WIDTH - 1) && OGM_NavCharIsWall(map[y][x + 1]) != 0U)
  {
    count++;
  }
  if (y > 0 && OGM_NavCharIsWall(map[y - 1][x]) != 0U)
  {
    count++;
  }
  if (y < (OGM_HEIGHT - 1) && OGM_NavCharIsWall(map[y + 1][x]) != 0U)
  {
    count++;
  }

  return count;
}

static uint8_t OGM_NavWallDiagNeighborCount(char map[OGM_HEIGHT][OGM_WIDTH], int x, int y)
{
  uint8_t count = 0U;

  if (x > 0 && y > 0 && OGM_NavCharIsWall(map[y - 1][x - 1]) != 0U)
  {
    count++;
  }
  if (x < (OGM_WIDTH - 1) && y > 0 && OGM_NavCharIsWall(map[y - 1][x + 1]) != 0U)
  {
    count++;
  }
  if (x > 0 && y < (OGM_HEIGHT - 1) && OGM_NavCharIsWall(map[y + 1][x - 1]) != 0U)
  {
    count++;
  }
  if (x < (OGM_WIDTH - 1) && y < (OGM_HEIGHT - 1) && OGM_NavCharIsWall(map[y + 1][x + 1]) != 0U)
  {
    count++;
  }

  return count;
}

static uint8_t OGM_NavWallRunLength(char map[OGM_HEIGHT][OGM_WIDTH], int x, int y, int dx, int dy)
{
  uint8_t len = 1U;
  int nx = x + dx;
  int ny = y + dy;

  while (nx >= 0 && nx < OGM_WIDTH && ny >= 0 && ny < OGM_HEIGHT &&
         OGM_NavCharIsWall(map[ny][nx]) != 0U)
  {
    if (len < 250U)
    {
      len++;
    }
    nx += dx;
    ny += dy;
  }

  nx = x - dx;
  ny = y - dy;
  while (nx >= 0 && nx < OGM_WIDTH && ny >= 0 && ny < OGM_HEIGHT &&
         OGM_NavCharIsWall(map[ny][nx]) != 0U)
  {
    if (len < 250U)
    {
      len++;
    }
    nx -= dx;
    ny -= dy;
  }

  return len;
}

static uint8_t OGM_NavWallHasCornerSupport(char map[OGM_HEIGHT][OGM_WIDTH], int x, int y)
{
  uint8_t has_h = 0U;
  uint8_t has_v = 0U;

  if ((x > 0 && OGM_NavCharIsWall(map[y][x - 1]) != 0U) ||
      (x < (OGM_WIDTH - 1) && OGM_NavCharIsWall(map[y][x + 1]) != 0U))
  {
    has_h = 1U;
  }

  if ((y > 0 && OGM_NavCharIsWall(map[y - 1][x]) != 0U) ||
      (y < (OGM_HEIGHT - 1) && OGM_NavCharIsWall(map[y + 1][x]) != 0U))
  {
    has_v = 1U;
  }

  return (has_h != 0U && has_v != 0U) ? 1U : 0U;
}

static uint8_t OGM_NavWallLooksHorizontal(char map[OGM_HEIGHT][OGM_WIDTH], int x, int y)
{
  uint8_t h_run = OGM_NavWallRunLength(map, x, y, 1, 0);
  uint8_t v_run = OGM_NavWallRunLength(map, x, y, 0, 1);

  return (h_run >= OGM_NAV_MIN_WALL_RUN && h_run >= v_run) ? 1U : 0U;
}

static uint8_t OGM_NavWallLooksVertical(char map[OGM_HEIGHT][OGM_WIDTH], int x, int y)
{
  uint8_t h_run = OGM_NavWallRunLength(map, x, y, 1, 0);
  uint8_t v_run = OGM_NavWallRunLength(map, x, y, 0, 1);

  return (v_run >= OGM_NAV_MIN_WALL_RUN && v_run >= h_run) ? 1U : 0U;
}

static uint8_t OGM_NavEstimateWallThickness(char map[OGM_HEIGHT][OGM_WIDTH])
{
  int x;
  int y;
  uint16_t samples = 0U;
  uint16_t sum = 0U;

  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      uint8_t h_run;
      uint8_t v_run;
      uint8_t thickness = 0U;

      if (OGM_NavCharIsWall(map[y][x]) == 0U)
      {
        continue;
      }

      h_run = OGM_NavWallRunLength(map, x, y, 1, 0);
      v_run = OGM_NavWallRunLength(map, x, y, 0, 1);
      if (h_run >= OGM_NAV_MIN_WALL_RUN && h_run >= v_run)
      {
        thickness = v_run;
      }
      else if (v_run >= OGM_NAV_MIN_WALL_RUN)
      {
        thickness = h_run;
      }

      if (thickness == 0U)
      {
        continue;
      }
      if (thickness > OGM_NAV_THICKNESS_SAMPLE_MAX)
      {
        thickness = OGM_NAV_THICKNESS_SAMPLE_MAX;
      }

      sum = (uint16_t)(sum + thickness);
      samples++;
    }
  }

  if (samples == 0U)
  {
    return OGM_NAV_THICKNESS_MIN;
  }

  {
    uint8_t avg = (uint8_t)((sum + (samples / 2U)) / samples);
    if (avg < OGM_NAV_THICKNESS_MIN)
    {
      avg = OGM_NAV_THICKNESS_MIN;
    }
    if (avg > OGM_NAV_THICKNESS_MAX)
    {
      avg = OGM_NAV_THICKNESS_MAX;
    }
    return avg;
  }
}

static void OGM_BuildProcessedDisplayMap(char map[OGM_HEIGHT][OGM_WIDTH])
{
  int x;
  int y;

  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      map[y][x] = OGM_CellToDisplayChar(x, y);
    }
  }

  OGM_PostProcessDisplayMap(map);
}

static void OGM_BuildNavMapFromDisplay(char nav[OGM_HEIGHT][OGM_WIDTH],
                                       char display[OGM_HEIGHT][OGM_WIDTH])
{
  int x;
  int y;

  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (OGM_DisplayCharIsOcc(display[y][x]) != 0U)
      {
        nav[y][x] = '#';
      }
      else if (OGM_DisplayCharIsFree(display[y][x]) != 0U)
      {
        nav[y][x] = '.';
      }
      else
      {
        nav[y][x] = ' ';
      }
    }
  }
}

static uint8_t OGM_DisplayHasOccNear(char display[OGM_HEIGHT][OGM_WIDTH],
                                     int x,
                                     int y,
                                     int radius)
{
  int dx;
  int dy;

  for (dy = -radius; dy <= radius; dy++)
  {
    for (dx = -radius; dx <= radius; dx++)
    {
      int nx = x + dx;
      int ny = y + dy;

      if (nx < 0 || nx >= OGM_WIDTH || ny < 0 || ny >= OGM_HEIGHT)
      {
        continue;
      }
      if (OGM_DisplayCharIsOcc(display[ny][nx]) != 0U)
      {
        return 1U;
      }
    }
  }

  return 0U;
}

static void OGM_NavConstrainToDisplayMap(char nav[OGM_HEIGHT][OGM_WIDTH],
                                         char display[OGM_HEIGHT][OGM_WIDTH])
{
  int x;
  int y;

  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (OGM_NavCharIsWall(nav[y][x]) != 0U &&
          OGM_DisplayHasOccNear(display, x, y, OGM_NAV_CONSTRAIN_OCC_RADIUS) == 0U)
      {
        nav[y][x] = (OGM_DisplayCharIsFree(display[y][x]) != 0U) ? '.' : ' ';
      }
    }
  }
}

static void OGM_NavBridgeAxisGaps(char map[OGM_HEIGHT][OGM_WIDTH])
{
  int x;
  int y;
  uint8_t gap;

  memcpy(ogm_display_src, map, sizeof(ogm_display_src));
  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (OGM_NavCharIsWall(ogm_display_src[y][x]) == 0U)
      {
        continue;
      }

      for (gap = 1U; gap <= OGM_NAV_BRIDGE_MAX_GAP; gap++)
      {
        int nx = x + (int)gap + 1;
        uint8_t clear_gap = 1U;

        if (nx >= OGM_WIDTH)
        {
          break;
        }
        if (OGM_NavCharIsWall(ogm_display_src[y][nx]) == 0U)
        {
          continue;
        }

        for (uint8_t k = 1U; k <= gap; k++)
        {
          if (OGM_NavCharIsFree(ogm_display_src[y][x + (int)k]) != 0U)
          {
            clear_gap = 0U;
            break;
          }
        }

        if (clear_gap != 0U)
        {
          for (uint8_t k = 1U; k <= gap; k++)
          {
            map[y][x + (int)k] = '#';
          }
        }
        break;
      }
    }
  }

  memcpy(ogm_display_src, map, sizeof(ogm_display_src));
  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (OGM_NavCharIsWall(ogm_display_src[y][x]) == 0U)
      {
        continue;
      }

      for (gap = 1U; gap <= OGM_NAV_BRIDGE_MAX_GAP; gap++)
      {
        int ny = y + (int)gap + 1;
        uint8_t clear_gap = 1U;

        if (ny >= OGM_HEIGHT)
        {
          break;
        }
        if (OGM_NavCharIsWall(ogm_display_src[ny][x]) == 0U)
        {
          continue;
        }

        for (uint8_t k = 1U; k <= gap; k++)
        {
          if (OGM_NavCharIsFree(ogm_display_src[y + (int)k][x]) != 0U)
          {
            clear_gap = 0U;
            break;
          }
        }

        if (clear_gap != 0U)
        {
          for (uint8_t k = 1U; k <= gap; k++)
          {
            map[y + (int)k][x] = '#';
          }
        }
        break;
      }
    }
  }
}

static void OGM_NavPruneDiagonalNoise(char map[OGM_HEIGHT][OGM_WIDTH])
{
  int x;
  int y;

  memcpy(ogm_display_src, map, sizeof(ogm_display_src));
  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (OGM_NavCharIsWall(ogm_display_src[y][x]) != 0U &&
          OGM_NavWallAxisNeighborCount(ogm_display_src, x, y) == 0U)
      {
        map[y][x] = (ogm_grid[y][x] <= OGM_NAV_FREE_MIN) ? '.' : ' ';
      }
    }
  }

  memcpy(ogm_display_src, map, sizeof(ogm_display_src));
  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (OGM_NavCharIsWall(ogm_display_src[y][x]) != 0U &&
          OGM_NavWallAxisNeighborCount(ogm_display_src, x, y) == 0U &&
          OGM_NavWallDiagNeighborCount(ogm_display_src, x, y) <= 1U)
      {
        map[y][x] = (ogm_grid[y][x] <= OGM_NAV_FREE_MIN) ? '.' : ' ';
      }
    }
  }
}

static void OGM_NavCopyFreeUnknown(char dst[OGM_HEIGHT][OGM_WIDTH],
                                   char src[OGM_HEIGHT][OGM_WIDTH])
{
  int x;
  int y;

  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (OGM_NavCharIsWall(src[y][x]) != 0U)
      {
        dst[y][x] = (ogm_grid[y][x] <= OGM_NAV_FREE_MIN) ? '.' : ' ';
      }
      else
      {
        dst[y][x] = src[y][x];
      }
    }
  }
}

static void OGM_NavClearMap(char map[OGM_HEIGHT][OGM_WIDTH])
{
  int x;
  int y;

  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      map[y][x] = ' ';
    }
  }
}

static void OGM_NavBridgeSupportLine(uint8_t support[], uint8_t len)
{
  uint8_t i;

  for (i = 0U; i < len; i++)
  {
    if (support[i] == 0U)
    {
      continue;
    }

    for (uint8_t gap = 1U; gap <= OGM_NAV_AXIS_BAND_GAP; gap++)
    {
      uint8_t next = (uint8_t)(i + gap + 1U);

      if (next >= len)
      {
        break;
      }
      if (support[next] == 0U)
      {
        continue;
      }

      for (uint8_t k = 1U; k <= gap; k++)
      {
        support[i + k] = 1U;
      }
      break;
    }
  }
}

static void OGM_NavBuildCenterMask(char src[OGM_HEIGHT][OGM_WIDTH],
                                   char center[OGM_HEIGHT][OGM_WIDTH])
{
  int x;
  int y;

  OGM_NavClearMap(center);

  for (x = 0; x < OGM_WIDTH; x++)
  {
    y = 0;
    while (y < OGM_HEIGHT)
    {
      int y0;
      int y1;

      while (y < OGM_HEIGHT &&
             (OGM_NavCharIsWall(src[y][x]) == 0U ||
              OGM_NavWallLooksHorizontal(src, x, y) == 0U))
      {
        y++;
      }

      if (y >= OGM_HEIGHT)
      {
        break;
      }

      y0 = y;
      while (y < OGM_HEIGHT &&
             OGM_NavCharIsWall(src[y][x]) != 0U &&
             OGM_NavWallLooksHorizontal(src, x, y) != 0U)
      {
        y++;
      }
      y1 = y - 1;
      center[(y0 + y1) / 2][x] = '#';
    }
  }

  for (y = 0; y < OGM_HEIGHT; y++)
  {
    x = 0;
    while (x < OGM_WIDTH)
    {
      int x0;
      int x1;

      while (x < OGM_WIDTH &&
             (OGM_NavCharIsWall(src[y][x]) == 0U ||
              OGM_NavWallLooksVertical(src, x, y) == 0U))
      {
        x++;
      }

      if (x >= OGM_WIDTH)
      {
        break;
      }

      x0 = x;
      while (x < OGM_WIDTH &&
             OGM_NavCharIsWall(src[y][x]) != 0U &&
             OGM_NavWallLooksVertical(src, x, y) != 0U)
      {
        x++;
      }
      x1 = x - 1;
      center[y][(x0 + x1) / 2] = '#';
    }
  }
}

static void OGM_NavDrawHorizontalSupportRuns(char dst[OGM_HEIGHT][OGM_WIDTH],
                                             uint8_t support[],
                                             uint8_t len,
                                             int y)
{
  uint8_t x = 0U;

  while (x < len)
  {
    uint8_t x0;
    uint8_t x1;

    while (x < len && support[x] == 0U)
    {
      x++;
    }
    if (x >= len)
    {
      break;
    }

    x0 = x;
    while (x < len && support[x] != 0U)
    {
      x++;
    }
    x1 = (uint8_t)(x - 1U);

    if ((uint8_t)(x1 - x0 + 1U) >= OGM_NAV_AXIS_BAND_MIN_POINTS)
    {
      for (uint8_t xx = x0; xx <= x1; xx++)
      {
        dst[y][xx] = '#';
      }
    }
  }
}

static void OGM_NavDrawVerticalSupportRuns(char dst[OGM_HEIGHT][OGM_WIDTH],
                                           uint8_t support[],
                                           uint8_t len,
                                           int x)
{
  uint8_t y = 0U;

  while (y < len)
  {
    uint8_t y0;
    uint8_t y1;

    while (y < len && support[y] == 0U)
    {
      y++;
    }
    if (y >= len)
    {
      break;
    }

    y0 = y;
    while (y < len && support[y] != 0U)
    {
      y++;
    }
    y1 = (uint8_t)(y - 1U);

    if ((uint8_t)(y1 - y0 + 1U) >= OGM_NAV_AXIS_BAND_MIN_POINTS)
    {
      for (uint8_t yy = y0; yy <= y1; yy++)
      {
        dst[yy][x] = '#';
      }
    }
  }
}

static void OGM_NavSnapCenterLines(char center[OGM_HEIGHT][OGM_WIDTH],
                                   char snapped[OGM_HEIGHT][OGM_WIDTH])
{
  int x;
  int y;

  OGM_NavClearMap(snapped);
  memset(ogm_nav_row_support, 0, sizeof(ogm_nav_row_support));
  memset(ogm_nav_col_support, 0, sizeof(ogm_nav_col_support));

  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (OGM_NavCharIsWall(center[y][x]) != 0U)
      {
        if (ogm_nav_row_support[y] < 250U)
        {
          ogm_nav_row_support[y]++;
        }
        if (ogm_nav_col_support[x] < 250U)
        {
          ogm_nav_col_support[x]++;
        }
      }
    }
  }

  y = 0;
  while (y < OGM_HEIGHT)
  {
    int y0;
    int y1;
    int probe;
    int last;
    uint32_t sum = 0U;
    uint32_t count = 0U;
    int avg_y;

    while (y < OGM_HEIGHT && ogm_nav_row_support[y] == 0U)
    {
      y++;
    }
    if (y >= OGM_HEIGHT)
    {
      break;
    }

    y0 = y;
    last = y;
    probe = y;
    while (probe < OGM_HEIGHT && (probe - y0) <= (int)OGM_NAV_AXIS_BAND_MAX_SPAN)
    {
      if (ogm_nav_row_support[probe] != 0U)
      {
        sum += (uint32_t)probe * (uint32_t)ogm_nav_row_support[probe];
        count += ogm_nav_row_support[probe];
        last = probe;
      }
      else if ((probe - last) > (int)OGM_NAV_AXIS_BAND_GAP)
      {
        break;
      }
      probe++;
    }
    y1 = last;
    y = y1 + 1;

    if (count < OGM_NAV_AXIS_BAND_MIN_POINTS)
    {
      continue;
    }

    avg_y = (int)((sum + (count / 2U)) / count);
    memset(ogm_nav_axis_support, 0, OGM_WIDTH);
    for (int yy = y0; yy <= y1; yy++)
    {
      for (x = 0; x < OGM_WIDTH; x++)
      {
        if (OGM_NavCharIsWall(center[yy][x]) != 0U)
        {
          ogm_nav_axis_support[x] = 1U;
        }
      }
    }
    OGM_NavBridgeSupportLine(ogm_nav_axis_support, OGM_WIDTH);
    OGM_NavDrawHorizontalSupportRuns(snapped, ogm_nav_axis_support, OGM_WIDTH, avg_y);
  }

  x = 0;
  while (x < OGM_WIDTH)
  {
    int x0;
    int x1;
    int probe;
    int last;
    uint32_t sum = 0U;
    uint32_t count = 0U;
    int avg_x;

    while (x < OGM_WIDTH && ogm_nav_col_support[x] == 0U)
    {
      x++;
    }
    if (x >= OGM_WIDTH)
    {
      break;
    }

    x0 = x;
    last = x;
    probe = x;
    while (probe < OGM_WIDTH && (probe - x0) <= (int)OGM_NAV_AXIS_BAND_MAX_SPAN)
    {
      if (ogm_nav_col_support[probe] != 0U)
      {
        sum += (uint32_t)probe * (uint32_t)ogm_nav_col_support[probe];
        count += ogm_nav_col_support[probe];
        last = probe;
      }
      else if ((probe - last) > (int)OGM_NAV_AXIS_BAND_GAP)
      {
        break;
      }
      probe++;
    }
    x1 = last;
    x = x1 + 1;

    if (count < OGM_NAV_AXIS_BAND_MIN_POINTS)
    {
      continue;
    }

    avg_x = (int)((sum + (count / 2U)) / count);
    memset(ogm_nav_axis_support, 0, OGM_HEIGHT);
    for (int xx = x0; xx <= x1; xx++)
    {
      for (y = 0; y < OGM_HEIGHT; y++)
      {
        if (OGM_NavCharIsWall(center[y][xx]) != 0U)
        {
          ogm_nav_axis_support[y] = 1U;
        }
      }
    }
    OGM_NavBridgeSupportLine(ogm_nav_axis_support, OGM_HEIGHT);
    OGM_NavDrawVerticalSupportRuns(snapped, ogm_nav_axis_support, OGM_HEIGHT, avg_x);
  }
}

static void OGM_NavDrawHorizontalWall(char map[OGM_HEIGHT][OGM_WIDTH],
                                      int x,
                                      int y,
                                      uint8_t thickness)
{
  int start_y = y - ((int)thickness / 2);

  for (uint8_t i = 0U; i < thickness; i++)
  {
    int yy = start_y + (int)i;
    if (yy >= 0 && yy < OGM_HEIGHT)
    {
      map[yy][x] = '#';
    }
  }
}

static void OGM_NavDrawVerticalWall(char map[OGM_HEIGHT][OGM_WIDTH],
                                    int x,
                                    int y,
                                    uint8_t thickness)
{
  int start_x = x - ((int)thickness / 2);

  for (uint8_t i = 0U; i < thickness; i++)
  {
    int xx = start_x + (int)i;
    if (xx >= 0 && xx < OGM_WIDTH)
    {
      map[y][xx] = '#';
    }
  }
}

static void OGM_NavPaintNormalizedWalls(char map[OGM_HEIGHT][OGM_WIDTH],
                                        char center[OGM_HEIGHT][OGM_WIDTH],
                                        uint8_t thickness)
{
  int x;
  int y;

  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      uint8_t h_run;
      uint8_t v_run;

      if (OGM_NavCharIsWall(center[y][x]) == 0U)
      {
        continue;
      }

      h_run = OGM_NavWallRunLength(center, x, y, 1, 0);
      v_run = OGM_NavWallRunLength(center, x, y, 0, 1);
      if (h_run >= v_run)
      {
        OGM_NavDrawHorizontalWall(map, x, y, thickness);
      }
      if (v_run >= h_run)
      {
        OGM_NavDrawVerticalWall(map, x, y, thickness);
      }
    }
  }
}

static void OGM_NavNormalizeWallBands(char map[OGM_HEIGHT][OGM_WIDTH])
{
#if OGM_NAV_NORMALIZE_ENABLE
  uint8_t thickness;

  memcpy(ogm_display_src, map, sizeof(ogm_display_src));
  thickness = OGM_NavEstimateWallThickness(ogm_display_src);
  OGM_NavBuildCenterMask(ogm_display_src, ogm_nav_center_map);
  OGM_NavBridgeAxisGaps(ogm_nav_center_map);
  OGM_NavSnapCenterLines(ogm_nav_center_map, map);
  memcpy(ogm_nav_center_map, map, sizeof(ogm_nav_center_map));
  OGM_NavCopyFreeUnknown(map, ogm_display_src);
  OGM_NavPaintNormalizedWalls(map, ogm_nav_center_map, thickness);
#else
  (void)map;
#endif
}

static void OGM_NavPruneShortRuns(char map[OGM_HEIGHT][OGM_WIDTH])
{
  int x;
  int y;

  memcpy(ogm_display_src, map, sizeof(ogm_display_src));
  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      uint8_t h_run;
      uint8_t v_run;

      if (OGM_NavCharIsWall(ogm_display_src[y][x]) == 0U)
      {
        continue;
      }

      h_run = OGM_NavWallRunLength(ogm_display_src, x, y, 1, 0);
      v_run = OGM_NavWallRunLength(ogm_display_src, x, y, 0, 1);
      if (h_run < OGM_NAV_MIN_WALL_RUN &&
          v_run < OGM_NAV_MIN_WALL_RUN &&
          OGM_NavWallHasCornerSupport(ogm_display_src, x, y) == 0U)
      {
        map[y][x] = (ogm_grid[y][x] <= OGM_NAV_FREE_MIN) ? '.' : ' ';
      }
    }
  }
}

static void OGM_PostProcessNavMap(char map[OGM_HEIGHT][OGM_WIDTH])
{
  if (OGM_NAV_DIRECT_FROM_MAP_ENABLE != 0U)
  {
#if OGM_NAV_DIRECT_CLEAN_ENABLE
    for (uint8_t i = 0U; i < OGM_NAV_PRUNE_PASSES; i++)
    {
      OGM_NavBridgeAxisGaps(map);
      OGM_NavPruneDiagonalNoise(map);
    }

    OGM_NavPruneShortRuns(map);
    OGM_NavBridgeAxisGaps(map);
#if OGM_NAV_DIRECT_MANHATTAN_ENABLE
    OGM_NavNormalizeWallBands(map);
    OGM_NavBridgeAxisGaps(map);
    OGM_NavPruneDiagonalNoise(map);
#endif
#else
    (void)map;
#endif
    return;
  }

  for (uint8_t i = 0U; i < OGM_NAV_PRUNE_PASSES; i++)
  {
    OGM_NavBridgeAxisGaps(map);
    OGM_NavPruneDiagonalNoise(map);
    OGM_NavPruneShortRuns(map);
  }

  OGM_NavNormalizeWallBands(map);

  for (uint8_t i = 0U; i < OGM_NAV_PRUNE_PASSES; i++)
  {
    OGM_NavBridgeAxisGaps(map);
    OGM_NavPruneDiagonalNoise(map);
    OGM_NavPruneShortRuns(map);
  }
}

#if MAZE_TOPO_ENABLE && MAZE_ASTAR_ENABLE
static void MazeAstar_ResetPlan(void)
{
  maze_return_astar_active = 0U;
  maze_return_astar_fallback = 0U;
  maze_astar_last_fail_pose = 0U;
  maze_astar_path_count = 0U;
  maze_astar_path_valid = 0U;
  maze_astar_plan_count = 0U;
  maze_astar_final_segment_m = 0.0f;
  maze_astar_return_progress_i = 0U;
  maze_astar_last_dynamic_decision = MAZE_TOPO_DECISION_NONE;
  maze_astar_last_dynamic_goal = 0U;
  maze_astar_return_replan_tick = 0U;
  maze_astar_segment_count = 0U;
  maze_astar_active_segment_i = 0U;
  maze_astar_segment_turn_pending = 0U;
  memset(maze_astar_plan_decisions, 0, sizeof(maze_astar_plan_decisions));
  memset(maze_astar_plan_turn_cells, 0, sizeof(maze_astar_plan_turn_cells));
  memset(maze_astar_path_cells, 0, sizeof(maze_astar_path_cells));
  memset(maze_astar_segments, 0, sizeof(maze_astar_segments));
}

static void MazeAstar_ClearTempBlocks(void)
{
  maze_astar_temp_block_count = 0U;
  memset(maze_astar_temp_block_cells, 0, sizeof(maze_astar_temp_block_cells));
}

static uint16_t MazeAstar_CellIndex(int x, int y)
{
  return (uint16_t)(((uint16_t)y * (uint16_t)OGM_WIDTH) + (uint16_t)x);
}

static uint8_t MazeAstar_CellTempBlocked(uint16_t idx)
{
  uint8_t i;

  for (i = 0U; i < maze_astar_temp_block_count; i++)
  {
    if (maze_astar_temp_block_cells[i] == idx)
    {
      return 1U;
    }
  }

  return 0U;
}

static uint8_t MazeAstar_AddTempBlockedCell(int x, int y)
{
  uint16_t idx;

  if (x < 0 || x >= OGM_WIDTH || y < 0 || y >= OGM_HEIGHT)
  {
    return 0U;
  }

  idx = MazeAstar_CellIndex(x, y);
  if (MazeAstar_CellTempBlocked(idx) != 0U)
  {
    return 0U;
  }

  if (maze_astar_temp_block_count >= MAZE_ASTAR_BLOCK_MAX_CELLS)
  {
    memmove(&maze_astar_temp_block_cells[0],
            &maze_astar_temp_block_cells[1],
            sizeof(maze_astar_temp_block_cells[0]) *
            (MAZE_ASTAR_BLOCK_MAX_CELLS - 1U));
    maze_astar_temp_block_count = (uint8_t)(MAZE_ASTAR_BLOCK_MAX_CELLS - 1U);
  }

  maze_astar_temp_block_cells[maze_astar_temp_block_count] = idx;
  maze_astar_temp_block_count++;
  return 1U;
}

static int MazeAstar_CellX(uint16_t idx)
{
  return (int)(idx % (uint16_t)OGM_WIDTH);
}

static int MazeAstar_CellY(uint16_t idx)
{
  return (int)(idx / (uint16_t)OGM_WIDTH);
}

static uint16_t MazeAstar_Heuristic(int x, int y, int gx, int gy)
{
  int dx = x - gx;
  int dy = y - gy;
  if (dx < 0)
  {
    dx = -dx;
  }
  if (dy < 0)
  {
    dy = -dy;
  }
  return (uint16_t)((dx + dy) * (int)MAZE_ASTAR_STEP_COST);
}

static uint8_t MazeAstar_MapPassable(char map[OGM_HEIGHT][OGM_WIDTH],
                                     int x,
                                     int y,
                                     uint16_t start_idx,
                                     uint16_t goal_idx)
{
  uint16_t idx;

  if (x < 0 || x >= OGM_WIDTH || y < 0 || y >= OGM_HEIGHT)
  {
    return 0U;
  }

  idx = MazeAstar_CellIndex(x, y);
  if (idx == start_idx || idx == goal_idx)
  {
    return 1U;
  }
  if (MazeAstar_CellTempBlocked(idx) != 0U)
  {
    return 0U;
  }

  return (map[y][x] == '.') ? 1U : 0U;
}

static uint16_t MazeAstar_WallClearanceCost(char map[OGM_HEIGHT][OGM_WIDTH],
                                            int x,
                                            int y,
                                            uint16_t start_idx,
                                            uint16_t goal_idx)
{
#if MAZE_ASTAR_CENTER_BIAS_ENABLE
  uint16_t idx;
  int dx;
  int dy;
  uint8_t adj_wall = 0U;
  uint8_t near_wall = 0U;

  if (x < 0 || x >= OGM_WIDTH || y < 0 || y >= OGM_HEIGHT)
  {
    return MAZE_ASTAR_WALL_ADJ_COST;
  }

  idx = MazeAstar_CellIndex(x, y);
  if (idx == start_idx || idx == goal_idx)
  {
    return 0U;
  }

  for (dy = -MAZE_ASTAR_CENTER_BIAS_RADIUS_CELLS; dy <= MAZE_ASTAR_CENTER_BIAS_RADIUS_CELLS; dy++)
  {
    for (dx = -MAZE_ASTAR_CENTER_BIAS_RADIUS_CELLS; dx <= MAZE_ASTAR_CENTER_BIAS_RADIUS_CELLS; dx++)
    {
      int nx;
      int ny;
      int manhattan;

      if (dx == 0 && dy == 0)
      {
        continue;
      }

      manhattan = (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy);
      if (manhattan > MAZE_ASTAR_CENTER_BIAS_RADIUS_CELLS)
      {
        continue;
      }

      nx = x + dx;
      ny = y + dy;
      if (nx < 0 || nx >= OGM_WIDTH || ny < 0 || ny >= OGM_HEIGHT)
      {
        if (manhattan <= 1)
        {
          adj_wall = 1U;
        }
        else
        {
          near_wall = 1U;
        }
        continue;
      }

      if (OGM_NavCharIsWall(map[ny][nx]) != 0U)
      {
        if (manhattan <= 1)
        {
          adj_wall = 1U;
        }
        else
        {
          near_wall = 1U;
        }
      }
    }
  }

  if (adj_wall != 0U)
  {
    return MAZE_ASTAR_WALL_ADJ_COST;
  }
  if (near_wall != 0U)
  {
    return MAZE_ASTAR_WALL_NEAR_COST;
  }
#else
  (void)map;
  (void)x;
  (void)y;
  (void)start_idx;
  (void)goal_idx;
#endif
  return 0U;
}

static uint8_t MazeAstar_FindNearestPassable(char map[OGM_HEIGHT][OGM_WIDTH],
                                             int in_x,
                                             int in_y,
                                             int *out_x,
                                             int *out_y)
{
  uint16_t dummy_idx = MAZE_ASTAR_PARENT_NONE;
  int radius;

  if (in_x < 0 || in_x >= OGM_WIDTH || in_y < 0 || in_y >= OGM_HEIGHT)
  {
    return 0U;
  }

  for (radius = 0; radius <= MAZE_ASTAR_NEAREST_FREE_RADIUS; radius++)
  {
    int dy;
    for (dy = -radius; dy <= radius; dy++)
    {
      int dx;
      for (dx = -radius; dx <= radius; dx++)
      {
        int nx;
        int ny;
        if ((dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy) != radius)
        {
          continue;
        }
        nx = in_x + dx;
        ny = in_y + dy;
        if (MazeAstar_MapPassable(map, nx, ny, dummy_idx, dummy_idx) != 0U)
        {
          *out_x = nx;
          *out_y = ny;
          return 1U;
        }
      }
    }
  }

  return 0U;
}

static uint8_t MazeAstar_DirBetween(uint16_t from_idx, uint16_t to_idx)
{
  int fx = MazeAstar_CellX(from_idx);
  int fy = MazeAstar_CellY(from_idx);
  int tx = MazeAstar_CellX(to_idx);
  int ty = MazeAstar_CellY(to_idx);

  if (tx == (fx + 1) && ty == fy)
  {
    return 0U;
  }
  if (tx == fx && ty == (fy + 1))
  {
    return 1U;
  }
  if (tx == (fx - 1) && ty == fy)
  {
    return 2U;
  }
  if (tx == fx && ty == (fy - 1))
  {
    return 3U;
  }
  return 4U;
}

static uint8_t MazeAstar_DirTurnDecision(uint8_t from_dir, uint8_t to_dir)
{
  uint8_t delta = (uint8_t)((to_dir + 4U - from_dir) & 0x03U);

  if (delta == 0U)
  {
    return MAZE_TOPO_DECISION_STRAIGHT;
  }
  if (delta == 1U)
  {
    return MAZE_TOPO_DECISION_LEFT;
  }
  if (delta == 3U)
  {
    return MAZE_TOPO_DECISION_RIGHT;
  }
  return MAZE_TOPO_DECISION_BACKUP;
}

static uint8_t MazeAstar_DirWorldBit(uint8_t dir)
{
  switch (dir)
  {
    case 0U:
      return MAZE_TOPO_WORLD_E;
    case 1U:
      return MAZE_TOPO_WORLD_N;
    case 2U:
      return MAZE_TOPO_WORLD_W;
    case 3U:
      return MAZE_TOPO_WORLD_S;
    default:
      return 0U;
  }
}

static uint8_t MazeAstar_AddSegment(uint16_t start_i, uint16_t end_i, uint8_t dir)
{
  if (dir > 3U || end_i <= start_i)
  {
    return 0U;
  }
  if (maze_astar_segment_count >= MAZE_ASTAR_MAX_SEGMENTS)
  {
    return 0U;
  }

  maze_astar_segments[maze_astar_segment_count].start_i = start_i;
  maze_astar_segments[maze_astar_segment_count].end_i = end_i;
  maze_astar_segments[maze_astar_segment_count].dir = dir;
  maze_astar_segment_count++;
  return 1U;
}

static uint8_t MazeAstar_BuildSegments(void)
{
  uint16_t i;
  uint16_t seg_start;
  uint8_t seg_dir;

  maze_astar_segment_count = 0U;
  maze_astar_active_segment_i = 0U;
  maze_astar_segment_turn_pending = 0U;
  memset(maze_astar_segments, 0, sizeof(maze_astar_segments));

  if (maze_astar_path_valid == 0U || maze_astar_path_count < 2U)
  {
    return 0U;
  }

  seg_start = 0U;
  seg_dir = MazeAstar_DirBetween(maze_astar_path_cells[0],
                                 maze_astar_path_cells[1]);
  if (seg_dir > 3U)
  {
    return 0U;
  }

  for (i = 2U; i < maze_astar_path_count; i++)
  {
    uint8_t dir = MazeAstar_DirBetween(maze_astar_path_cells[i - 1U],
                                       maze_astar_path_cells[i]);
    if (dir > 3U)
    {
      return 0U;
    }
    if (dir != seg_dir)
    {
      if (MazeAstar_AddSegment(seg_start, (uint16_t)(i - 1U), seg_dir) == 0U)
      {
        return 0U;
      }
      seg_start = (uint16_t)(i - 1U);
      seg_dir = dir;
    }
  }

  return MazeAstar_AddSegment(seg_start,
                              (uint16_t)(maze_astar_path_count - 1U),
                              seg_dir);
}

static uint8_t MazeAstar_DebugSnapshot(maze_astar_debug_t *dbg)
{
  uint16_t i;
  uint16_t nearest_i = 0U;
  uint16_t nearest_d2 = 0xFFFFU;
  uint16_t base_i;
  uint16_t target_i;
  uint16_t next_i;
  uint16_t gate_i = MAZE_ASTAR_PARENT_NONE;
  int cur_x;
  int cur_y;
  int dx;
  int dy;

  if (dbg == 0)
  {
    return 0U;
  }
  memset(dbg, 0, sizeof(*dbg));
  dbg->next_dir = 4U;
  dbg->next_decision = MAZE_TOPO_DECISION_NONE;

  if (maze_return_active == 0U ||
      maze_return_astar_active == 0U ||
      maze_astar_path_valid == 0U ||
      maze_astar_path_count == 0U)
  {
    return 0U;
  }

  if (OGM_WorldToCell(OGM_GetMapX(), OGM_GetMapY(), &cur_x, &cur_y) == 0)
  {
    return 0U;
  }

  dbg->cur_x = cur_x;
  dbg->cur_y = cur_y;
  dbg->progress_i = maze_astar_return_progress_i;

  for (i = 0U; i < maze_astar_path_count; i++)
  {
    int path_x = MazeAstar_CellX(maze_astar_path_cells[i]);
    int path_y = MazeAstar_CellY(maze_astar_path_cells[i]);
    int ddx = cur_x - path_x;
    int ddy = cur_y - path_y;
    uint16_t d2;

    if (ddx < 0)
    {
      ddx = -ddx;
    }
    if (ddy < 0)
    {
      ddy = -ddy;
    }
    d2 = (uint16_t)((ddx * ddx) + (ddy * ddy));
    if (d2 < nearest_d2)
    {
      nearest_d2 = d2;
      nearest_i = i;
    }
  }

  if (maze_astar_segment_count != 0U &&
      maze_astar_active_segment_i < maze_astar_segment_count)
  {
    const maze_astar_segment_t *seg = &maze_astar_segments[maze_astar_active_segment_i];
    nearest_i = seg->start_i;
    nearest_d2 = 0xFFFFU;
    for (i = seg->start_i; i <= seg->end_i; i++)
    {
      int path_x = MazeAstar_CellX(maze_astar_path_cells[i]);
      int path_y = MazeAstar_CellY(maze_astar_path_cells[i]);
      int ddx = cur_x - path_x;
      int ddy = cur_y - path_y;
      uint16_t d2;

      if (ddx < 0)
      {
        ddx = -ddx;
      }
      if (ddy < 0)
      {
        ddy = -ddy;
      }
      d2 = (uint16_t)((ddx * ddx) + (ddy * ddy));
      if (d2 < nearest_d2)
      {
        nearest_d2 = d2;
        nearest_i = i;
      }
    }
  }

  dbg->nearest_i = nearest_i;
  dbg->nearest_x = MazeAstar_CellX(maze_astar_path_cells[nearest_i]);
  dbg->nearest_y = MazeAstar_CellY(maze_astar_path_cells[nearest_i]);
  dbg->nearest_d2 = nearest_d2;

  base_i = maze_astar_return_progress_i;
  if (nearest_i > base_i)
  {
    base_i = nearest_i;
  }
  if (maze_astar_segment_count != 0U &&
      maze_astar_active_segment_i < maze_astar_segment_count)
  {
    const maze_astar_segment_t *seg = &maze_astar_segments[maze_astar_active_segment_i];
    if (base_i < seg->start_i)
    {
      base_i = seg->start_i;
    }
    if (base_i > seg->end_i)
    {
      base_i = seg->end_i;
    }
  }
  if (base_i >= maze_astar_path_count)
  {
    base_i = (uint16_t)(maze_astar_path_count - 1U);
  }

  target_i = (uint16_t)(base_i + MAZE_ASTAR_DBG_LOOKAHEAD_CELLS);
  if (maze_astar_segment_count != 0U &&
      maze_astar_active_segment_i < maze_astar_segment_count &&
      target_i > maze_astar_segments[maze_astar_active_segment_i].end_i)
  {
    target_i = maze_astar_segments[maze_astar_active_segment_i].end_i;
  }
  if (target_i >= maze_astar_path_count)
  {
    target_i = (uint16_t)(maze_astar_path_count - 1U);
  }
  if (target_i == base_i && (base_i + 1U) < maze_astar_path_count)
  {
    target_i = (uint16_t)(base_i + 1U);
  }

  dbg->target_i = target_i;
  dbg->target_x = MazeAstar_CellX(maze_astar_path_cells[target_i]);
  dbg->target_y = MazeAstar_CellY(maze_astar_path_cells[target_i]);
  dx = cur_x - dbg->target_x;
  dy = cur_y - dbg->target_y;
  if (dx < 0)
  {
    dx = -dx;
  }
  if (dy < 0)
  {
    dy = -dy;
  }
  dbg->target_d2 = (uint16_t)((dx * dx) + (dy * dy));

  next_i = base_i;
  if ((base_i + 1U) < maze_astar_path_count)
  {
    next_i = (uint16_t)(base_i + 1U);
  }
  if (next_i != base_i)
  {
    dbg->next_dir = MazeAstar_DirBetween(maze_astar_path_cells[base_i],
                                         maze_astar_path_cells[next_i]);
  }
  if (maze_astar_segment_count != 0U &&
      maze_astar_active_segment_i < maze_astar_segment_count)
  {
    dbg->next_dir = maze_astar_segments[maze_astar_active_segment_i].dir;
  }
  dbg->astar_heading = (uint8_t)(MazeTopo_HeadingQuadrant() & 0x03);
  if (dbg->next_dir <= 3U)
  {
    dbg->next_decision = MazeAstar_DirTurnDecision(dbg->astar_heading,
                                                   dbg->next_dir);
  }

  if (maze_astar_plan_count != 0U)
  {
    uint16_t best_path_i = MAZE_ASTAR_PARENT_NONE;
    uint16_t plan_i;

    for (plan_i = 0U; plan_i < maze_astar_plan_count; plan_i++)
    {
      uint16_t plan_cell = maze_astar_plan_turn_cells[plan_i];
      uint16_t path_i;
      for (path_i = maze_astar_return_progress_i; path_i < maze_astar_path_count; path_i++)
      {
        if (maze_astar_path_cells[path_i] == plan_cell)
        {
          if (path_i < best_path_i)
          {
            best_path_i = path_i;
            gate_i = plan_cell;
          }
          break;
        }
      }
    }
  }

  dbg->gate_i = gate_i;
  if (gate_i < (uint16_t)(OGM_WIDTH * OGM_HEIGHT))
  {
    dbg->gate_valid = 1U;
    dbg->gate_x = MazeAstar_CellX(gate_i);
    dbg->gate_y = MazeAstar_CellY(gate_i);
    dx = cur_x - dbg->gate_x;
    dy = cur_y - dbg->gate_y;
    if (dx < 0)
    {
      dx = -dx;
    }
    if (dy < 0)
    {
      dy = -dy;
    }
    dbg->gate_d2 = (uint16_t)((dx * dx) + (dy * dy));
    dbg->gate_ready = (dbg->gate_d2 <= (MAZE_ASTAR_TURN_GATE_CELLS * MAZE_ASTAR_TURN_GATE_CELLS)) ? 1U : 0U;
  }

  return 1U;
}

static void MazeAstar_ReturnBeginHome(uint32_t now_tick)
{
  maze_return_remaining = 0U;
  maze_astar_segment_turn_pending = 0U;
  maze_return_home_pending = 1U;
  maze_return_home_tick = now_tick;
  maze_return_home_drive_started = 0U;
  maze_return_home_drive_tick = 0U;
  maze_return_home_start_x_m = OGM_GetMapX();
  maze_return_home_start_y_m = OGM_GetMapY();
  maze_return_final_target_m = MAZE_RETURN_FINAL_MIN_M;
  MazeReturn_SetPhase(MAZE_RETURN_PHASE_HOME_APPROACH, now_tick);
}

static uint8_t MazeAstar_ReturnFirstDecision(uint8_t *decision, uint8_t *path_dir)
{
  uint8_t dir;
  uint8_t world_bit;

  if (decision == 0 || path_dir == 0)
  {
    return 0U;
  }

  *decision = MAZE_TOPO_DECISION_NONE;
  *path_dir = 4U;

  if (maze_astar_path_valid == 0U || maze_astar_path_count < 2U)
  {
    return 0U;
  }

  if (maze_astar_segment_count != 0U)
  {
    dir = maze_astar_segments[0].dir;
  }
  else
  {
    dir = MazeAstar_DirBetween(maze_astar_path_cells[0],
                               maze_astar_path_cells[1]);
  }
  if (dir > 3U)
  {
    return 0U;
  }

  world_bit = MazeAstar_DirWorldBit(dir);
  *decision = MazeTopo_WorldBitToDecision(world_bit, MazeTopo_HeadingQuadrant());
  *path_dir = dir;

  return (*decision != MAZE_TOPO_DECISION_NONE) ? 1U : 0U;
}

static uint8_t MazeAstar_ReturnReplanForDeviation(const maze_astar_debug_t *dbg,
                                                  uint32_t now_tick,
                                                  uint8_t *replanned)
{
  char msg[128];
  int n;

  if (replanned != 0)
  {
    *replanned = 0U;
  }

  if (dbg == 0)
  {
    return 0U;
  }

  if (maze_astar_return_replan_tick != 0U &&
      (now_tick - maze_astar_return_replan_tick) < MAZE_ASTAR_REPLAN_COOLDOWN_MS)
  {
    return 1U;
  }

  n = snprintf(msg, sizeof(msg),
               "ASTAR:REPLAN DEV ND=%u PR=%u C=%d,%d N=%u,%d,%d\r\n",
               (unsigned)dbg->nearest_d2,
               (unsigned)maze_astar_return_progress_i,
               dbg->cur_x,
               dbg->cur_y,
               (unsigned)dbg->nearest_i,
               dbg->nearest_x,
               dbg->nearest_y);
  if (n > 0)
  {
    uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
  }
  MazeReturn_SetPhase(MAZE_RETURN_PHASE_REPLAN, now_tick);

  if (MazeAstar_BuildReturnPlan(now_tick) == 0U)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"ASTAR:REPLAN CLRBLK\r\n", 21U, 80);
    MazeAstar_ClearTempBlocks();
    if (MazeAstar_BuildReturnPlan(now_tick) == 0U)
    {
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)"ASTAR:REPLAY FALLBACK\r\n", 23U, 80);
      maze_return_astar_active = 0U;
      maze_return_astar_fallback = 1U;
      maze_astar_segment_turn_pending = 0U;
      MazeReturn_SetPlan(MAZE_RETURN_PLAN_ASTAR_FALLBACK);
      MazeReturn_SetPhase(MAZE_RETURN_PHASE_FOLLOW_REPLAY, now_tick);
      maze_return_remaining = maze_topo_path_count;
      maze_return_home_pending = 0U;
      maze_return_home_drive_started = 0U;
      maze_return_home_drive_tick = 0U;
      maze_astar_return_replan_tick = now_tick;
      return 1U;
    }
  }

  maze_return_astar_active = 1U;
  maze_return_astar_fallback = 0U;
  MazeReturn_SetPlan(MAZE_RETURN_PLAN_ASTAR);
  maze_return_remaining = (maze_astar_path_count > 0U) ?
                          (uint16_t)(maze_astar_path_count - 1U) :
                          0U;
  maze_return_home_pending = 0U;
  maze_return_home_drive_started = 0U;
  maze_return_home_drive_tick = 0U;
  if (maze_return_remaining == 0U)
  {
    MazeAstar_ReturnBeginHome(now_tick);
  }
  else
  {
    MazeReturn_BeginAstarPathAlign(now_tick, "REPLAN");
  }
  maze_astar_return_replan_tick = now_tick;
  if (replanned != 0)
  {
    *replanned = 1U;
  }

  return 1U;
}

static uint8_t MazeAstar_AddReturnBlockCells(int cur_x, int cur_y)
{
  static const int8_t dx[4] = { 1, 0, -1, 0 };
  static const int8_t dy[4] = { 0, 1, 0, -1 };
  maze_astar_debug_t dbg;
  uint8_t added = 0U;
  uint8_t heading_dir = (uint8_t)(MazeTopo_HeadingQuadrant() & 0x03);
  uint8_t step;

  if (cur_x >= 0 && cur_x < OGM_WIDTH && cur_y >= 0 && cur_y < OGM_HEIGHT)
  {
    for (step = 1U; step <= MAZE_ASTAR_BLOCK_AHEAD_CELLS; step++)
    {
      added = (uint8_t)(added +
        MazeAstar_AddTempBlockedCell(cur_x + ((int)dx[heading_dir] * (int)step),
                                     cur_y + ((int)dy[heading_dir] * (int)step)));
    }
  }

  if (MazeAstar_DebugSnapshot(&dbg) != 0U)
  {
    uint16_t base_i = dbg.nearest_i;
    if (maze_astar_return_progress_i > base_i)
    {
      base_i = maze_astar_return_progress_i;
    }

    for (step = 1U; step <= MAZE_ASTAR_BLOCK_AHEAD_CELLS; step++)
    {
      uint16_t path_i = (uint16_t)(base_i + step);
      if (path_i >= maze_astar_path_count)
      {
        break;
      }
      added = (uint8_t)(added +
        MazeAstar_AddTempBlockedCell(MazeAstar_CellX(maze_astar_path_cells[path_i]),
                                     MazeAstar_CellY(maze_astar_path_cells[path_i])));
    }
  }

  return added;
}

static uint8_t MazeAstar_ReturnReplanForBlock(uint32_t now_tick,
                                              float front_m,
                                              uint32_t blocked_ms,
                                              uint8_t decision)
{
  char msg[128];
  int cur_x = -1;
  int cur_y = -1;
  uint8_t added;
  int n;

  if (maze_astar_return_replan_tick != 0U &&
      (now_tick - maze_astar_return_replan_tick) < MAZE_ASTAR_REPLAN_COOLDOWN_MS)
  {
    return 1U;
  }

  (void)OGM_WorldToCell(OGM_GetMapX(), OGM_GetMapY(), &cur_x, &cur_y);
  added = MazeAstar_AddReturnBlockCells(cur_x, cur_y);
  n = snprintf(msg, sizeof(msg),
               "ASTAR:REPLAN BLOCK D=%c F=%.2f AGE=%lu C=%d,%d PR=%u ADD=%u BLK=%u\r\n",
               MazeTopo_DecisionChar(decision),
               (double)front_m,
               (unsigned long)blocked_ms,
               cur_x,
               cur_y,
               (unsigned)maze_astar_return_progress_i,
               (unsigned)added,
               (unsigned)maze_astar_temp_block_count);
  if (n > 0)
  {
    uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
  }
  MazeReturn_SetPhase(MAZE_RETURN_PHASE_REPLAN, now_tick);

  if (MazeAstar_BuildReturnPlan(now_tick) == 0U)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"ASTAR:REPLAN CLRBLK\r\n", 21U, 80);
    MazeAstar_ClearTempBlocks();
    if (MazeAstar_BuildReturnPlan(now_tick) == 0U)
    {
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)"ASTAR:REPLAY FALLBACK\r\n", 23U, 80);
      maze_return_astar_active = 0U;
      maze_return_astar_fallback = 1U;
      maze_astar_segment_turn_pending = 0U;
      MazeReturn_SetPlan(MAZE_RETURN_PLAN_ASTAR_FALLBACK);
      MazeReturn_SetPhase(MAZE_RETURN_PHASE_FOLLOW_REPLAY, now_tick);
      maze_return_remaining = maze_topo_path_count;
      maze_return_home_pending = 0U;
      maze_return_home_drive_started = 0U;
      maze_return_home_drive_tick = 0U;
      maze_astar_return_replan_tick = now_tick;
      return 1U;
    }
  }

  maze_return_astar_active = 1U;
  maze_return_astar_fallback = 0U;
  MazeReturn_SetPlan(MAZE_RETURN_PLAN_ASTAR);
  maze_return_remaining = (maze_astar_path_count > 0U) ?
                          (uint16_t)(maze_astar_path_count - 1U) :
                          0U;
  maze_return_home_pending = 0U;
  maze_return_home_drive_started = 0U;
  maze_return_home_drive_tick = 0U;
  if (maze_return_remaining == 0U)
  {
    MazeAstar_ReturnBeginHome(now_tick);
  }
  else
  {
    MazeReturn_BeginAstarPathAlign(now_tick, "REPLAN");
  }
  maze_astar_return_replan_tick = now_tick;

  return 1U;
}

static uint8_t MazeAstar_ReturnDynamicDecision(uint8_t *decision, uint8_t *goal_ready)
{
  maze_astar_debug_t dbg;
  maze_astar_debug_t local_dbg;
  const maze_astar_segment_t *seg;
  uint16_t goal_i;
  uint16_t i;
  uint16_t nearest_i;
  uint16_t nearest_d2;
  uint16_t dev_limit_d2 =
    (uint16_t)(MAZE_ASTAR_REPLAN_DEVIATION_CELLS * MAZE_ASTAR_REPLAN_DEVIATION_CELLS);
  int goal_x;
  int goal_y;
  int dx;
  int dy;
  int start_x;
  int start_y;
  int end_x;
  int end_y;
  int seg_len;
  int along;
  int lateral;
  uint8_t arrived_segment_end;
  uint8_t world_bit;
  uint8_t replanned = 0U;

  if (decision == 0 || goal_ready == 0)
  {
    return 0U;
  }

  *decision = MAZE_TOPO_DECISION_NONE;
  *goal_ready = 0U;
  maze_astar_last_dynamic_decision = MAZE_TOPO_DECISION_NONE;
  maze_astar_last_dynamic_goal = 0U;

  if (MazeAstar_DebugSnapshot(&dbg) == 0U)
  {
    return 0U;
  }

  if (maze_astar_segment_count == 0U)
  {
    if (MazeAstar_BuildSegments() == 0U)
    {
      return 0U;
    }
  }

  if (MazeReturn_PostAlignReplanGuardActive(HAL_GetTick()) != 0U &&
      maze_return_latched_decision != MAZE_TOPO_DECISION_NONE)
  {
    *decision = maze_return_latched_decision;
    maze_return_target_dir = (maze_return_target_dir <= 3U) ?
                             maze_return_target_dir :
                             dbg.next_dir;
    maze_astar_last_dynamic_decision = *decision;
    return 1U;
  }

choose_segment:
  if (maze_astar_active_segment_i >= maze_astar_segment_count)
  {
    *goal_ready = 1U;
    maze_astar_last_dynamic_goal = 1U;
    return 1U;
  }

  goal_i = (uint16_t)(maze_astar_path_count - 1U);
  seg = &maze_astar_segments[maze_astar_active_segment_i];
  if (seg->end_i > goal_i || seg->start_i >= seg->end_i || seg->dir > 3U)
  {
    return 0U;
  }

  nearest_i = seg->start_i;
  nearest_d2 = 0xFFFFU;
  for (i = seg->start_i; i <= seg->end_i; i++)
  {
    int path_x = MazeAstar_CellX(maze_astar_path_cells[i]);
    int path_y = MazeAstar_CellY(maze_astar_path_cells[i]);
    int ddx = dbg.cur_x - path_x;
    int ddy = dbg.cur_y - path_y;
    uint16_t d2;

    if (ddx < 0)
    {
      ddx = -ddx;
    }
    if (ddy < 0)
    {
      ddy = -ddy;
    }
    d2 = (uint16_t)((ddx * ddx) + (ddy * ddy));
    if (d2 < nearest_d2)
    {
      nearest_d2 = d2;
      nearest_i = i;
    }
  }

  local_dbg = dbg;
  local_dbg.nearest_i = nearest_i;
  local_dbg.nearest_x = MazeAstar_CellX(maze_astar_path_cells[nearest_i]);
  local_dbg.nearest_y = MazeAstar_CellY(maze_astar_path_cells[nearest_i]);
  local_dbg.nearest_d2 = nearest_d2;

  if (nearest_d2 > dev_limit_d2)
  {
    if (MazeReturn_PostAlignReplanGuardActive(HAL_GetTick()) != 0U &&
        maze_return_latched_decision != MAZE_TOPO_DECISION_NONE)
    {
      *decision = maze_return_latched_decision;
      maze_astar_last_dynamic_decision = *decision;
      return 1U;
    }

    if (MazeAstar_ReturnReplanForDeviation(&local_dbg, HAL_GetTick(), &replanned) == 0U)
    {
      return 0U;
    }
    if (maze_return_astar_active == 0U)
    {
      *decision = MazeTopo_ReturnPeekDecision();
      maze_return_latched_decision = *decision;
      maze_return_target_dir = 4U;
      return (*decision != MAZE_TOPO_DECISION_NONE) ? 1U : 0U;
    }
    if (maze_return_phase == MAZE_RETURN_PHASE_ALIGN_TO_PATH)
    {
      *decision = maze_return_latched_decision;
      maze_astar_last_dynamic_decision = *decision;
      return (*decision != MAZE_TOPO_DECISION_NONE) ? 1U : 0U;
    }
    if (replanned != 0U &&
        MazeAstar_DebugSnapshot(&dbg) == 0U)
    {
      return 0U;
    }
    if (replanned != 0U)
    {
      goto choose_segment;
    }
  }

  if (maze_astar_return_progress_i < seg->start_i)
  {
    maze_astar_return_progress_i = seg->start_i;
  }
  if (nearest_i > maze_astar_return_progress_i)
  {
    maze_astar_return_progress_i = nearest_i;
  }

  goal_x = MazeAstar_CellX(maze_astar_path_cells[goal_i]);
  goal_y = MazeAstar_CellY(maze_astar_path_cells[goal_i]);
  dx = dbg.cur_x - goal_x;
  dy = dbg.cur_y - goal_y;
  if (dx < 0)
  {
    dx = -dx;
  }
  if (dy < 0)
  {
    dy = -dy;
  }

  if (maze_astar_return_progress_i < goal_i)
  {
    maze_return_remaining = (uint16_t)(goal_i - maze_astar_return_progress_i);
  }
  else
  {
    maze_return_remaining = 0U;
  }

  start_x = MazeAstar_CellX(maze_astar_path_cells[seg->start_i]);
  start_y = MazeAstar_CellY(maze_astar_path_cells[seg->start_i]);
  end_x = MazeAstar_CellX(maze_astar_path_cells[seg->end_i]);
  end_y = MazeAstar_CellY(maze_astar_path_cells[seg->end_i]);
  seg_len = (int)(seg->end_i - seg->start_i);
  along = 0;
  lateral = 0;
  if (seg->dir == 0U)
  {
    along = dbg.cur_x - start_x;
    lateral = dbg.cur_y - start_y;
  }
  else if (seg->dir == 1U)
  {
    along = dbg.cur_y - start_y;
    lateral = dbg.cur_x - start_x;
  }
  else if (seg->dir == 2U)
  {
    along = start_x - dbg.cur_x;
    lateral = dbg.cur_y - start_y;
  }
  else
  {
    along = start_y - dbg.cur_y;
    lateral = dbg.cur_x - start_x;
  }
  if (lateral < 0)
  {
    lateral = -lateral;
  }

  arrived_segment_end =
    (along >= (seg_len - (int)MAZE_ASTAR_SEGMENT_ARRIVE_CELLS) &&
     lateral <= (int)MAZE_ASTAR_SEGMENT_LATERAL_CELLS) ? 1U : 0U;

  if (seg->end_i >= goal_i &&
      (arrived_segment_end != 0U ||
       ((dx * dx) + (dy * dy)) <= (MAZE_ASTAR_DYNAMIC_GOAL_CELLS * MAZE_ASTAR_DYNAMIC_GOAL_CELLS)))
  {
    *goal_ready = 1U;
    maze_astar_last_dynamic_goal = 1U;
    maze_astar_segment_turn_pending = 0U;
    return 1U;
  }

  if (arrived_segment_end != 0U &&
      (maze_astar_active_segment_i + 1U) < maze_astar_segment_count)
  {
    maze_astar_active_segment_i++;
    seg = &maze_astar_segments[maze_astar_active_segment_i];
    maze_astar_return_progress_i = seg->start_i;
  }

  if (seg->dir > 3U)
  {
    return 0U;
  }

  (void)end_x;
  (void)end_y;

  world_bit = MazeAstar_DirWorldBit(seg->dir);
  *decision = MazeTopo_WorldBitToDecision(world_bit, MazeTopo_HeadingQuadrant());
  if (*decision == MAZE_TOPO_DECISION_NONE)
  {
    return 0U;
  }

  MazeReturn_SetPhase(MAZE_RETURN_PHASE_FOLLOW_ASTAR, HAL_GetTick());
  maze_return_target_dir = seg->dir;
  maze_return_latched_decision = *decision;
  maze_astar_segment_turn_pending =
    (*decision != MAZE_TOPO_DECISION_STRAIGHT) ? 1U : 0U;
  maze_astar_last_dynamic_decision = *decision;
  return 1U;
}

static uint8_t MazeAstar_AddPlanDecision(uint8_t decision, uint16_t turn_cell)
{
  if (decision == MAZE_TOPO_DECISION_STRAIGHT)
  {
    return 1U;
  }
  if (maze_astar_plan_count >= MAZE_TOPO_MAX_NODES)
  {
    return 0U;
  }
  maze_astar_plan_decisions[maze_astar_plan_count] = decision;
  maze_astar_plan_turn_cells[maze_astar_plan_count] = turn_cell;
  maze_astar_plan_count++;
  return 1U;
}

static uint8_t MazeAstar_ReconstructPlan(uint16_t start_idx,
                                         uint16_t goal_idx,
                                         uint8_t initial_heading)
{
  uint16_t cur = goal_idx;
  uint16_t count = 0U;
  uint16_t i;
  uint8_t current_dir = initial_heading;
  uint16_t segment_steps = 0U;

  maze_astar_path_count = 0U;
  maze_astar_path_valid = 0U;
  maze_astar_plan_count = 0U;
  maze_astar_final_segment_m = 0.0f;
  memset(maze_astar_plan_decisions, 0, sizeof(maze_astar_plan_decisions));
  memset(maze_astar_plan_turn_cells, 0, sizeof(maze_astar_plan_turn_cells));

  while (1)
  {
    if (count >= MAZE_ASTAR_MAX_PATH_CELLS)
    {
      return 0U;
    }
    maze_astar_path_cells[count] = cur;
    count++;
    if (cur == start_idx)
    {
      break;
    }
    if (cur >= (uint16_t)(OGM_WIDTH * OGM_HEIGHT) ||
        maze_astar_parent[cur] == MAZE_ASTAR_PARENT_NONE)
    {
      return 0U;
    }
    cur = maze_astar_parent[cur];
  }

  for (i = 0U; i < (count / 2U); i++)
  {
    uint16_t tmp = maze_astar_path_cells[i];
    maze_astar_path_cells[i] = maze_astar_path_cells[count - 1U - i];
    maze_astar_path_cells[count - 1U - i] = tmp;
  }

  maze_astar_path_count = count;
  maze_astar_path_valid = 1U;

  for (i = 1U; i < count; i++)
  {
    uint8_t dir = MazeAstar_DirBetween(maze_astar_path_cells[i - 1U],
                                       maze_astar_path_cells[i]);
    if (dir > 3U)
    {
      continue;
    }
    if (dir != current_dir)
    {
      if (MazeAstar_AddPlanDecision(MazeAstar_DirTurnDecision(current_dir, dir),
                                    maze_astar_path_cells[i - 1U]) == 0U)
      {
        return 0U;
      }
      current_dir = dir;
      segment_steps = 0U;
    }
    segment_steps++;
  }

  maze_astar_final_segment_m = ((float)segment_steps) * OGM_RESOLUTION_M;

  return MazeAstar_BuildSegments();
}

static uint8_t MazeAstar_Run(char map[OGM_HEIGHT][OGM_WIDTH],
                             int start_x,
                             int start_y,
                             int goal_x,
                             int goal_y,
                             uint8_t initial_heading)
{
  static const int8_t dx[4] = { 1, 0, -1, 0 };
  static const int8_t dy[4] = { 0, 1, 0, -1 };
  const uint16_t cell_count = (uint16_t)(OGM_WIDTH * OGM_HEIGHT);
  uint16_t start_idx = MazeAstar_CellIndex(start_x, start_y);
  uint16_t goal_idx = MazeAstar_CellIndex(goal_x, goal_y);
  uint16_t i;

  for (i = 0U; i < cell_count; i++)
  {
    maze_astar_flags[i] = 0U;
    maze_astar_g_cost[i] = MAZE_ASTAR_COST_INF;
    maze_astar_parent[i] = MAZE_ASTAR_PARENT_NONE;
  }

  maze_astar_g_cost[start_idx] = 0U;
  maze_astar_flags[start_idx] = MAZE_ASTAR_FLAG_OPEN;

  while (1)
  {
    uint16_t best_idx = MAZE_ASTAR_PARENT_NONE;
    uint16_t best_f = MAZE_ASTAR_COST_INF;
    uint16_t best_h = MAZE_ASTAR_COST_INF;

    for (i = 0U; i < cell_count; i++)
    {
      if ((maze_astar_flags[i] & MAZE_ASTAR_FLAG_OPEN) != 0U &&
          (maze_astar_flags[i] & MAZE_ASTAR_FLAG_CLOSED) == 0U)
      {
        int x = MazeAstar_CellX(i);
        int y = MazeAstar_CellY(i);
        uint16_t h = MazeAstar_Heuristic(x, y, goal_x, goal_y);
        uint16_t f = (maze_astar_g_cost[i] >= (MAZE_ASTAR_COST_INF - h)) ?
                     MAZE_ASTAR_COST_INF :
                     (uint16_t)(maze_astar_g_cost[i] + h);
        if (f < best_f || (f == best_f && h < best_h))
        {
          best_idx = i;
          best_f = f;
          best_h = h;
        }
      }
    }

    if (best_idx == MAZE_ASTAR_PARENT_NONE)
    {
      return 0U;
    }
    if (best_idx == goal_idx)
    {
      return MazeAstar_ReconstructPlan(start_idx, goal_idx, initial_heading);
    }

    maze_astar_flags[best_idx] |= MAZE_ASTAR_FLAG_CLOSED;

    {
      int bx = MazeAstar_CellX(best_idx);
      int by = MazeAstar_CellY(best_idx);
      uint8_t base_dir = initial_heading;
      uint8_t dir;

      if (best_idx != start_idx &&
          maze_astar_parent[best_idx] != MAZE_ASTAR_PARENT_NONE)
      {
        uint8_t parent_dir = MazeAstar_DirBetween(maze_astar_parent[best_idx], best_idx);
        if (parent_dir <= 3U)
        {
          base_dir = parent_dir;
        }
      }

      for (dir = 0U; dir < 4U; dir++)
      {
        int nx = bx + dx[dir];
        int ny = by + dy[dir];
        uint16_t nidx;
        uint16_t turn_cost;
        uint16_t center_cost;
        uint16_t tentative;

        if (MazeAstar_MapPassable(map, nx, ny, start_idx, goal_idx) == 0U)
        {
          continue;
        }

        nidx = MazeAstar_CellIndex(nx, ny);
        if ((maze_astar_flags[nidx] & MAZE_ASTAR_FLAG_CLOSED) != 0U)
        {
          continue;
        }

        turn_cost = (dir == base_dir) ? 0U : MAZE_ASTAR_TURN_COST;
        center_cost = MazeAstar_WallClearanceCost(map, nx, ny, start_idx, goal_idx);
        tentative = (uint16_t)(maze_astar_g_cost[best_idx] +
                               MAZE_ASTAR_STEP_COST +
                               turn_cost +
                               center_cost);

        if ((maze_astar_flags[nidx] & MAZE_ASTAR_FLAG_OPEN) == 0U ||
            tentative < maze_astar_g_cost[nidx])
        {
          maze_astar_flags[nidx] |= MAZE_ASTAR_FLAG_OPEN;
          maze_astar_g_cost[nidx] = tentative;
          maze_astar_parent[nidx] = best_idx;
        }
      }
    }
  }
}

static uint8_t MazeAstar_BuildReturnPlan(uint32_t now_tick)
{
  char msg[128];
  int sx;
  int sy;
  int gx;
  int gy;
  int start_x;
  int start_y;
  int goal_x;
  int goal_y;
  uint8_t initial_heading;
  uint8_t ok = 0U;
  int n;

  (void)now_tick;
  MazeAstar_ResetPlan();

  OGM_BuildProcessedDisplayMap(ogm_nav_source_map);
  OGM_BuildNavMapFromDisplay(ogm_display_map, ogm_nav_source_map);
  OGM_PostProcessNavMap(ogm_display_map);
  OGM_NavConstrainToDisplayMap(ogm_display_map, ogm_nav_source_map);

  if (OGM_WorldToCell(OGM_GetMapX(), OGM_GetMapY(), &sx, &sy) == 0 ||
      OGM_WorldToCell(OGM_MAP_START_X_M, OGM_MAP_START_Y_M, &gx, &gy) == 0)
  {
    maze_astar_last_fail_pose = 1U;
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"ASTAR:FAIL POSE\r\n", 17U, 80);
    return 0U;
  }

  if (MazeAstar_FindNearestPassable(ogm_display_map, sx, sy, &start_x, &start_y) == 0U ||
      MazeAstar_FindNearestPassable(ogm_display_map, gx, gy, &goal_x, &goal_y) == 0U)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"ASTAR:FAIL FREE\r\n", 17U, 80);
    return 0U;
  }

  initial_heading = (uint8_t)(MazeTopo_HeadingQuadrant() & 0x03);
  ok = MazeAstar_Run(ogm_display_map,
                     start_x,
                     start_y,
                     goal_x,
                     goal_y,
                     initial_heading);

  n = snprintf(msg, sizeof(msg),
               "ASTAR:%s C=%u D=%u F=%.2f S=%d,%d G=%d,%d H=%u\r\n",
               (ok != 0U) ? "OK" : "FAIL",
               (unsigned)maze_astar_path_count,
               (unsigned)maze_astar_plan_count,
               (double)maze_astar_final_segment_m,
               start_x,
               start_y,
               goal_x,
               goal_y,
               (unsigned)initial_heading);
  if (n > 0)
  {
    uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
  }

  if (ok != 0U)
  {
    OGM_PrintViewMap(ogm_display_map);
  }

  return ok;
}

static uint8_t MazeAstar_CellOnPath(int x, int y)
{
  uint16_t idx;
  uint16_t i;

  if (maze_astar_path_valid == 0U ||
      x < 0 || x >= OGM_WIDTH ||
      y < 0 || y >= OGM_HEIGHT)
  {
    return 0U;
  }

  idx = MazeAstar_CellIndex(x, y);
  for (i = 0U; i < maze_astar_path_count; i++)
  {
    if (maze_astar_path_cells[i] == idx)
    {
      return 1U;
    }
  }
  return 0U;
}
#endif

static void OGM_PrintAsciiMap(void)
{
  char msg[96];
  int y;
  int x;
  int n = snprintf(msg, sizeof(msg),
                   "MAP:BEGIN W=%u H=%u RES=%.3f\r\n",
                   (unsigned)OGM_WIDTH,
                   (unsigned)OGM_HEIGHT,
                   (double)OGM_RESOLUTION_M);
  if (n > 0)
  {
    uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 120);
  }

  OGM_BuildProcessedDisplayMap(ogm_display_map);

  for (y = OGM_HEIGHT - 1; y >= 0; y--)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      ogm_map_row[x] = ogm_display_map[y][x];
    }
    ogm_map_row[OGM_WIDTH] = '\r';
    ogm_map_row[OGM_WIDTH + 1] = '\n';
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)ogm_map_row, (uint16_t)(OGM_WIDTH + 2), 120);
  }

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)"MAP:END\r\n", 9U, 120);
}

static void OGM_ViewIncludeCell(int cx,
                                int cy,
                                int *min_x,
                                int *max_x,
                                int *min_y,
                                int *max_y,
                                uint8_t *found)
{
  if (cx < 0 || cx >= OGM_WIDTH || cy < 0 || cy >= OGM_HEIGHT)
  {
    return;
  }

  if (*found == 0U)
  {
    *min_x = cx;
    *max_x = cx;
    *min_y = cy;
    *max_y = cy;
    *found = 1U;
    return;
  }

  if (cx < *min_x)
  {
    *min_x = cx;
  }
  if (cx > *max_x)
  {
    *max_x = cx;
  }
  if (cy < *min_y)
  {
    *min_y = cy;
  }
  if (cy > *max_y)
  {
    *max_y = cy;
  }
}

static char OGM_ViewHeadingChar(float theta_rad)
{
  float q = wrap_pi(theta_rad) / (PI_F * 0.5f);
  int h = (q >= 0.0f) ? (int)(q + 0.5f) : (int)(q - 0.5f);

  h %= 4;
  if (h < 0)
  {
    h += 4;
  }

  if (h == 0)
  {
    return '>';
  }
  if (h == 1)
  {
    return '^';
  }
  if (h == 2)
  {
    return '<';
  }
  return 'v';
}

static void OGM_PrintViewMap(char map[OGM_HEIGHT][OGM_WIDTH])
{
  char msg[160];
  int start_x = 0;
  int start_y = 0;
  int robot_x = 0;
  int robot_y = 0;
  uint8_t start_valid;
  uint8_t robot_valid;
  char robot_heading;
  uint8_t found = 0U;
  int min_x = 0;
  int max_x = 0;
  int min_y = 0;
  int max_y = 0;
  int width;
  int height;
  int y;
  int x;
  int n;

  start_valid = OGM_WorldToCell(OGM_MAP_START_X_M, OGM_MAP_START_Y_M, &start_x, &start_y);
  robot_valid = OGM_WorldToCell(OGM_GetMapX(), OGM_GetMapY(), &robot_x, &robot_y);
  robot_heading = OGM_ViewHeadingChar(OGM_GetMapThetaRad());

  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (map[y][x] != ' ')
      {
        OGM_ViewIncludeCell(x, y, &min_x, &max_x, &min_y, &max_y, &found);
      }
    }
  }

  if (start_valid != 0U)
  {
    OGM_ViewIncludeCell(start_x, start_y, &min_x, &max_x, &min_y, &max_y, &found);
  }
  if (robot_valid != 0U)
  {
    OGM_ViewIncludeCell(robot_x, robot_y, &min_x, &max_x, &min_y, &max_y, &found);
  }

  if (found == 0U)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"VIEW:EMPTY\r\n", 12U, 120);
    return;
  }

  min_x -= (int)OGM_VIEW_MARGIN_CELLS;
  max_x += (int)OGM_VIEW_MARGIN_CELLS;
  min_y -= (int)OGM_VIEW_MARGIN_CELLS;
  max_y += (int)OGM_VIEW_MARGIN_CELLS;

  if (min_x < 0)
  {
    min_x = 0;
  }
  if (min_y < 0)
  {
    min_y = 0;
  }
  if (max_x >= OGM_WIDTH)
  {
    max_x = OGM_WIDTH - 1;
  }
  if (max_y >= OGM_HEIGHT)
  {
    max_y = OGM_HEIGHT - 1;
  }

  width = (max_x - min_x) + 1;
  height = (max_y - min_y) + 1;

  n = snprintf(msg, sizeof(msg),
               "VIEW:STAT X0=%d Y0=%d W=%u H=%u RES=%.3f PATH=%u APATH=%u RET=%u DONE=%u\r\n",
               min_x,
               min_y,
               (unsigned)width,
               (unsigned)height,
               (double)OGM_RESOLUTION_M,
#if MAZE_TOPO_ENABLE
               (unsigned)maze_topo_path_count,
#if MAZE_ASTAR_ENABLE
               (unsigned)maze_astar_path_count,
#else
               0U,
#endif
               (unsigned)maze_return_active,
               (unsigned)maze_return_done
#else
               0U,
               0U,
               0U,
               0U
#endif
               );
  if (n > 0)
  {
    uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 120);
  }

  n = snprintf(msg, sizeof(msg),
               "VIEW:POSE S=%d,%d R=%d,%d DIR=%c X=%.2f Y=%.2f TH=%.2f\r\n",
               start_valid != 0U ? start_x : -1,
               start_valid != 0U ? start_y : -1,
               robot_valid != 0U ? robot_x : -1,
               robot_valid != 0U ? robot_y : -1,
               robot_heading,
               (double)OGM_GetMapX(),
               (double)OGM_GetMapY(),
               (double)OGM_GetMapThetaRad());
  if (n > 0)
  {
    uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 120);
  }

  (void)HAL_UART_Transmit(&huart1,
                          (uint8_t *)"VIEW:LEGEND #=wall .=free *=Astar S=start ^/>/v/<=robot @=robot_on_start\r\n",
                          (uint16_t)(sizeof("VIEW:LEGEND #=wall .=free *=Astar S=start ^/>/v/<=robot @=robot_on_start\r\n") - 1U),
                          120);
  (void)HAL_UART_Transmit(&huart1,
                          (uint8_t *)"VIEW:BEGIN\r\n",
                          (uint16_t)(sizeof("VIEW:BEGIN\r\n") - 1U),
                          120);

  for (y = max_y; y >= min_y; y--)
  {
    for (x = min_x; x <= max_x; x++)
    {
      char c = map[y][x];

#if MAZE_TOPO_ENABLE && MAZE_ASTAR_ENABLE
      if (MazeAstar_CellOnPath(x, y) != 0U && c != '#')
      {
        c = '*';
      }
#endif

      if (start_valid != 0U && robot_valid != 0U &&
          x == start_x && y == start_y &&
          x == robot_x && y == robot_y)
      {
        c = '@';
      }
      else if (robot_valid != 0U && x == robot_x && y == robot_y)
      {
        c = robot_heading;
      }
      else if (start_valid != 0U && x == start_x && y == start_y)
      {
        c = 'S';
      }

      ogm_map_row[x - min_x] = c;
    }
    ogm_map_row[width] = '\r';
    ogm_map_row[width + 1] = '\n';
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)ogm_map_row, (uint16_t)(width + 2), 120);
  }

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)"VIEW:END\r\n", 10U, 120);
}

static void OGM_PrintNavMap(void)
{
  char msg[96];
  int y;
  int x;
  uint16_t nav_unknown = 0U;
  uint16_t nav_free = 0U;
  uint16_t nav_wall = 0U;

  OGM_BuildProcessedDisplayMap(ogm_nav_source_map);
  OGM_BuildNavMapFromDisplay(ogm_display_map, ogm_nav_source_map);
  OGM_PostProcessNavMap(ogm_display_map);
  OGM_NavConstrainToDisplayMap(ogm_display_map, ogm_nav_source_map);

  for (y = 0; y < OGM_HEIGHT; y++)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      if (ogm_display_map[y][x] == '#')
      {
        nav_wall++;
      }
      else if (ogm_display_map[y][x] == '.')
      {
        nav_free++;
      }
      else
      {
        nav_unknown++;
      }
    }
  }

  {
    int n = snprintf(msg, sizeof(msg),
                     "NAV:STAT U=%u F=%u W=%u\r\n"
                     "NAV:BEGIN W=%u H=%u RES=%.3f\r\n",
                     (unsigned)nav_unknown,
                     (unsigned)nav_free,
                     (unsigned)nav_wall,
                     (unsigned)OGM_WIDTH,
                     (unsigned)OGM_HEIGHT,
                     (double)OGM_RESOLUTION_M);
    if (n > 0)
    {
      uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 120);
    }
  }

  for (y = OGM_HEIGHT - 1; y >= 0; y--)
  {
    for (x = 0; x < OGM_WIDTH; x++)
    {
      ogm_map_row[x] = ogm_display_map[y][x];
    }
    ogm_map_row[OGM_WIDTH] = '\r';
    ogm_map_row[OGM_WIDTH + 1] = '\n';
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)ogm_map_row, (uint16_t)(OGM_WIDTH + 2), 120);
  }

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)"NAV:END\r\n", 9U, 120);
  OGM_PrintViewMap(ogm_display_map);
}
#endif

static void OGM_HandleCommand(void)
{
#if ODOM_OGM_ENABLE && OGM_RUNTIME_ENABLE
  if (Map_Clear_Request != 0U)
  {
    Map_Clear_Request = 0U;
    Odom_Reset();
    __disable_irq();
    odom_pulse_accum_left = 0;
    odom_pulse_accum_right = 0;
    __enable_irq();
    OGM_Reset();
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"MAP:CLEAR\r\n", 11U, 80);
  }

  if (Map_Dump_Request != 0U)
  {
    uint16_t u = 0U;
    uint16_t f = 0U;
    uint16_t o = 0U;
    char msg[96];
    int n;

    Map_Dump_Request = 0U;
    Load(0, 0);
#if ODOM_OGM_ENABLE
    if (maze_simple_state_allows_stop_mapping() != 0U &&
        lidar_is_ready_for_runtime() != 0U &&
        imu_is_ready_for_runtime() != 0U &&
        OGM_AttitudeStableForMapping() != 0U)
    {
      uint32_t now_tick = HAL_GetTick();
      float map_theta_rad = OGM_GetMapThetaRad();
      OGM_EnsureMapPose();
      OGM_RecordPoseSample(now_tick, OGM_GetMapX(), OGM_GetMapY(), map_theta_rad);
      OGM_UpdateFromRanges();
    }
#endif
    OGM_GetStats(&u, &f, &o);
    n = snprintf(msg, sizeof(msg),
                 "MAP:STAT U=%u F=%u O=%u\r\n",
                 (unsigned)u,
                 (unsigned)f,
                 (unsigned)o);
    if (n > 0)
    {
      uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
    }
    OGM_PrintAsciiMap();
    OGM_PrintNavMap();
  }
#else
  if (Map_Clear_Request != 0U || Map_Dump_Request != 0U)
  {
    Map_Clear_Request = 0U;
    Map_Dump_Request = 0U;
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"MAP:DISABLED\r\n", 14U, 80);
  }
#endif
}

static void MazeTopo_HandleCommand(void)
{
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
  if (Topo_Return_Clear_Request != 0U)
  {
    Topo_Return_Clear_Request = 0U;
    Topo_Return_Request = 0U;
    MazeTopo_ClearReturn(HAL_GetTick());
  }

  if (Topo_Return_Request != 0U)
  {
    Topo_Return_Request = 0U;
#if MAZE_RETURN_ENABLE
    MazeTopo_StartReturn(HAL_GetTick());
#else
    MazeTopo_ClearReturn(HAL_GetTick());
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"RET:DISABLED\r\n", 14U, 80);
#endif
  }

  if (Topo_Dump_Request != 0U)
  {
    char msg[160];
    uint16_t i;
    int n;

    Topo_Dump_Request = 0U;
    Load(0, 0);

    n = snprintf(msg, sizeof(msg),
                 "TOPO:STAT N=%u NEXT=%u PATH=%u RET=%u REM=%u DONE=%u MERGE=%.2f\r\n",
                 (unsigned)maze_topo_count,
                 (unsigned)maze_topo_next_id,
                 (unsigned)maze_topo_path_count,
                 (unsigned)maze_return_active,
                 (unsigned)maze_return_remaining,
                 (unsigned)maze_return_done,
                 (double)MAZE_TOPO_MERGE_DIST_M);
    if (n > 0)
    {
      uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
    }

    (void)HAL_UART_Transmit(&huart1,
                            (uint8_t *)"TOPO:BEGIN O=FRBL A=NESW D=NFRLB TR=NESW K=PJCE R=FRBLmm\r\n",
                            (uint16_t)(sizeof("TOPO:BEGIN O=FRBL A=NESW D=NFRLB TR=NESW K=PJCE R=FRBLmm\r\n") - 1U),
                            80);

    for (i = 0U; i < maze_topo_count; i++)
    {
      const maze_topo_node_t *node = &maze_topo_nodes[i];
      char of = ((node->openings & MAZE_TOPO_OPEN_FRONT) != 0U) ? 'F' : '-';
      char oright = ((node->openings & MAZE_TOPO_OPEN_RIGHT) != 0U) ? 'R' : '-';
      char ob = ((node->openings & MAZE_TOPO_OPEN_BACK) != 0U) ? 'B' : '-';
      char ol = ((node->openings & MAZE_TOPO_OPEN_LEFT) != 0U) ? 'L' : '-';
      char an = ((node->open_world & MAZE_TOPO_WORLD_N) != 0U) ? 'N' : '-';
      char ae = ((node->open_world & MAZE_TOPO_WORLD_E) != 0U) ? 'E' : '-';
      char as = ((node->open_world & MAZE_TOPO_WORLD_S) != 0U) ? 'S' : '-';
      char aw = ((node->open_world & MAZE_TOPO_WORLD_W) != 0U) ? 'W' : '-';
      uint8_t tried = (uint8_t)(node->tried | maze_topo_failed_world[i]);
      char yn = ((tried & MAZE_TOPO_WORLD_N) != 0U) ? 'N' : '-';
      char ye = ((tried & MAZE_TOPO_WORLD_E) != 0U) ? 'E' : '-';
      char ys = ((tried & MAZE_TOPO_WORLD_S) != 0U) ? 'S' : '-';
      char yw = ((tried & MAZE_TOPO_WORLD_W) != 0U) ? 'W' : '-';

      if (node->id == 0U)
      {
        continue;
      }

      n = snprintf(msg, sizeof(msg),
                   "TOPO:N ID=%u X=%.2f Y=%.2f H=%d O=%c%c%c%c A=%c%c%c%c D=%c TR=%c%c%c%c K=%c V=%u R=%u,%u,%u,%u T=%lu\r\n",
                   (unsigned)node->id,
                   (double)node->x_m,
                   (double)node->y_m,
                   (int)node->heading_quadrant,
                   of,
                   oright,
                   ob,
                   ol,
                   an,
                   ae,
                   as,
                   aw,
                   MazeTopo_DecisionChar(node->decision),
                   yn,
                   ye,
                   ys,
                   yw,
                   MazeTopo_KindChar(node->kind),
                   (unsigned)node->visits,
                   (unsigned)node->front_mm,
                   (unsigned)node->right_mm,
                   (unsigned)node->rear_mm,
                   (unsigned)node->left_mm,
                   (unsigned long)node->tick);
      if (n > 0)
      {
        uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
      }
    }

    (void)HAL_UART_Transmit(&huart1,
                            (uint8_t *)"TOPO:PATH BEGIN\r\n",
                            (uint16_t)(sizeof("TOPO:PATH BEGIN\r\n") - 1U),
                            80);
    for (i = 0U; i < maze_topo_path_count; i++)
    {
      uint16_t node_idx = maze_topo_path_nodes[i];
      uint16_t node_id = (node_idx < maze_topo_count) ? maze_topo_nodes[node_idx].id : 0U;
      uint8_t return_decision = MazeTopo_PathReturnDecision(i);
      uint8_t return_world =
        MazeTopo_WorldBitOpposite(maze_topo_path_world_bits[i]);

      n = snprintf(msg, sizeof(msg),
                   "TOPO:P I=%u NODE=%u D=%c W=%c%c%c%c RW=%c%c%c%c RD=%c\r\n",
                   (unsigned)i,
                   (unsigned)node_id,
                   MazeTopo_DecisionChar(maze_topo_path_decisions[i]),
                   ((maze_topo_path_world_bits[i] & MAZE_TOPO_WORLD_N) != 0U) ? 'N' : '-',
                   ((maze_topo_path_world_bits[i] & MAZE_TOPO_WORLD_E) != 0U) ? 'E' : '-',
                   ((maze_topo_path_world_bits[i] & MAZE_TOPO_WORLD_S) != 0U) ? 'S' : '-',
                   ((maze_topo_path_world_bits[i] & MAZE_TOPO_WORLD_W) != 0U) ? 'W' : '-',
                   ((return_world & MAZE_TOPO_WORLD_N) != 0U) ? 'N' : '-',
                   ((return_world & MAZE_TOPO_WORLD_E) != 0U) ? 'E' : '-',
                   ((return_world & MAZE_TOPO_WORLD_S) != 0U) ? 'S' : '-',
                   ((return_world & MAZE_TOPO_WORLD_W) != 0U) ? 'W' : '-',
                   MazeTopo_DecisionChar(return_decision));
      if (n > 0)
      {
        uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
      }
    }
    (void)HAL_UART_Transmit(&huart1,
                            (uint8_t *)"TOPO:PATH END\r\n",
                            (uint16_t)(sizeof("TOPO:PATH END\r\n") - 1U),
                            80);

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"TOPO:END\r\n", 10U, 80);
  }
#else
  if (Topo_Dump_Request != 0U ||
      Topo_Return_Request != 0U ||
      Topo_Return_Clear_Request != 0U)
  {
    Topo_Dump_Request = 0U;
    Topo_Return_Request = 0U;
    Topo_Return_Clear_Request = 0U;
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"TOPO:DISABLED\r\n", 15U, 80);
  }
#endif
}

static void Lidar_HandleScanDumpCommand(void)
{
  if (Lidar_Scan_Dump_Request != 0U)
  {
    uint16_t count;
    uint16_t i;
    char msg[128];
    int n;

    Lidar_Scan_Dump_Request = 0U;
    Load(0, 0);

    count = Lidar_CopyScanBinsMm(lidar_scan_dump_bins,
                                 (uint16_t)LIDAR_SCAN_BIN_COUNT);

    n = snprintf(msg, sizeof(msg),
                 "SCAN:BEGIN B=%u DEG=5 T=%lu\r\n",
                 (unsigned)count,
                 (unsigned long)Lidar_GetScanTick());
    if (n > 0)
    {
      uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
    }

    for (i = 0U; i < count; i += 12U)
    {
      uint16_t end = (uint16_t)(i + 12U);
      uint16_t j;
      if (end > count)
      {
        end = count;
      }

      n = snprintf(msg, sizeof(msg),
                   "SCAN:%03u", (unsigned)i);
      for (j = i; j < end && n > 0 && n < (int)(sizeof(msg) - 8U); j++)
      {
        int m = snprintf(&msg[n],
                         sizeof(msg) - (uint16_t)n,
                         "%c%u",
                         (j == i) ? ' ' : ',',
                         (unsigned)lidar_scan_dump_bins[j]);
        if (m <= 0)
        {
          break;
        }
        n += m;
      }

      if (n > 0 && n < (int)(sizeof(msg) - 2U))
      {
        msg[n++] = '\r';
        msg[n++] = '\n';
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)n, 120);
      }
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"SCAN:END\r\n", 10U, 80);
  }
}

static uint8_t lidar_ranges_all_zero(void)
{
  if (fordist > 0.0f ||
      rightfrontdist > 0.0f ||
      rightdist > 0.0f ||
      rightreardist > 0.0f ||
      reardist > 0.0f ||
      leftreardist > 0.0f ||
      leftdist > 0.0f ||
      leftfrontdist > 0.0f)
  {
    return 0U;
  }
  return 1U;
}

static void Lidar_HandleReinitRequest(void)
{
  static uint32_t last_auto_reinit_tick = 0U;
  uint32_t now_tick = HAL_GetTick();
  uint8_t reinit_requested = 0U;
  uint8_t need_reinit = 0U;

  if (Lidar_Reinit_Request != 0U)
  {
    reinit_requested = 1U;
    __disable_irq();
    Lidar_Reinit_Request = 0U;
    __enable_irq();
  }

#if LIDAR_IDLE_REINIT_ENABLE
  if (reinit_requested == 0U &&
      lidar_is_ready_for_runtime() == 0U &&
      (now_tick - lidar_last_nonzero_tick) >= LIDAR_IDLE_REINIT_MS &&
      (now_tick - last_auto_reinit_tick) >= LIDAR_IDLE_REINIT_COOLDOWN_MS)
  {
    reinit_requested = 1U;
    last_auto_reinit_tick = now_tick;
  }
#endif

  if (reinit_requested == 0U)
  {
    return;
  }

  if (lidar_is_ready_for_runtime() == 0U || lidar_ranges_all_zero() != 0U)
  {
    need_reinit = 1U;
  }

  if (need_reinit == 0U)
  {
    return;
  }

  Load(0, 0);
  Lidar_InitSequence();
  lidar_ready = 0U;
  lidar_ready_stable_count = 0U;
  lidar_last_nonzero_tick = HAL_GetTick();
}

static float update_lidar_distance_filter(float raw_dist,
                                          float *filtered_dist,
                                          uint32_t *last_valid_tick,
                                          uint32_t now_tick)
{
  if (filtered_dist == NULL || last_valid_tick == NULL)
  {
    return 0.0f;
  }

  if (raw_dist >= DIST_VALID_MIN_M && raw_dist <= DIST_VALID_MAX_M)
  {
    if (*last_valid_tick == 0U || *filtered_dist <= 0.0f)
    {
      *filtered_dist = raw_dist;
    }
    else
    {
      float alpha = (raw_dist > *filtered_dist) ? DIST_LPF_ALPHA_RISE : DIST_LPF_ALPHA_FALL;
      *filtered_dist = *filtered_dist + alpha * (raw_dist - *filtered_dist);
    }
    *last_valid_tick = now_tick;
  }
  else if ((now_tick - *last_valid_tick) > DIST_HOLD_MS)
  {
    *filtered_dist = 0.0f;
  }

  return *filtered_dist;
}

static float clampf(float v, float vmin, float vmax)
{
  if (v < vmin)
  {
    return vmin;
  }
  if (v > vmax)
  {
    return vmax;
  }
  return v;
}

static uint8_t imu_yaw_usable_for_heading(void)
{
  float abs_roll;

  if (imu_is_ready_for_runtime() == 0U)
  {
    return 0U;
  }

  if (fabsf(pitch) > 25.0f)
  {
    return 0U;
  }

  abs_roll = fabsf(roll);
  if (abs_roll < 35.0f || abs_roll > 145.0f)
  {
    return 1U;
  }

  return 0U;
}

static void pid_reset_runtime(pid_t *ctrl)
{
  if (ctrl == NULL)
  {
    return;
  }

  ctrl->set = 0.0f;
  ctrl->fdb = 0.0f;
  ctrl->out = 0.0f;
  ctrl->Pout = 0.0f;
  ctrl->Iout = 0.0f;
  ctrl->Dout = 0.0f;
  ctrl->Dbuf[0] = 0.0f;
  ctrl->Dbuf[1] = 0.0f;
  ctrl->Dbuf[2] = 0.0f;
  ctrl->error[0] = 0.0f;
  ctrl->error[1] = 0.0f;
  ctrl->error[2] = 0.0f;
}

static void drive_pid_reset_all(void)
{
  pid_cmd_acc_A = 0.0f;
  pid_cmd_acc_B = 0.0f;
  pid_fdb_acc_A = 0.0f;
  pid_fdb_acc_B = 0.0f;
  pid_window_samples = 0U;
  pid_corr_pwm_A = 0.0f;
  pid_corr_pwm_B = 0.0f;
  set_A = 0.0f;
  set_B = 0.0f;
  pid_last_sign_A = 0;
  pid_last_sign_B = 0;
  pid_reset_runtime(&pid_A);
  pid_reset_runtime(&pid_B);
}

static int8_t pwm_sign(float v)
{
  if (v > 1.0f)
  {
    return 1;
  }
  if (v < -1.0f)
  {
    return -1;
  }
  return 0;
}

static void MazeLoad(float left_pwm, float right_pwm, uint32_t now_tick)
{
#if DRIVE_PID_ENABLE
  int8_t sign_l = pwm_sign(left_pwm);
  int8_t sign_r = pwm_sign(right_pwm);
  uint8_t moving = (sign_l != 0 || sign_r != 0) ? 1U : 0U;
  uint8_t need_reset = 0U;

  if (moving != 0U)
  {
    if (pid_last_move_tick == 0U || (now_tick - pid_last_move_tick) > DRIVE_PID_IDLE_RESET_MS)
    {
      need_reset = 1U;
    }
    pid_last_move_tick = now_tick;
  }

  if (sign_l != pid_last_sign_A || sign_r != pid_last_sign_B)
  {
    need_reset = 1U;
  }

  if (need_reset != 0U)
  {
    drive_pid_reset_all();
  }

  pid_last_sign_A = sign_l;
  pid_last_sign_B = sign_r;

  if (moving == 0U)
  {
    drive_pid_reset_all();
  }
  else
  {
    pid_cmd_acc_A += fabsf(left_pwm);
    pid_cmd_acc_B += fabsf(right_pwm);
    pid_fdb_acc_A += fabsf(fdb_A);
    pid_fdb_acc_B += fabsf(fdb_B);
    pid_window_samples++;

    if (pid_window_samples >= DRIVE_PID_WINDOW_TICKS)
    {
      set_A = pid_cmd_acc_A * DRIVE_PID_TARGET_PULSE_PER_PWM_TICK;
      set_B = pid_cmd_acc_B * DRIVE_PID_TARGET_PULSE_PER_PWM_TICK;
      pid_corr_pwm_A = pid_compute(&pid_A, pid_fdb_acc_A, set_A);
      pid_corr_pwm_B = pid_compute(&pid_B, pid_fdb_acc_B, set_B);

      pid_cmd_acc_A = 0.0f;
      pid_cmd_acc_B = 0.0f;
      pid_fdb_acc_A = 0.0f;
      pid_fdb_acc_B = 0.0f;
      pid_window_samples = 0U;
    }

    left_pwm = (float)sign_l * clampf(fabsf(left_pwm) + pid_corr_pwm_A, 0.0f, MAZE_PWM_MAX);
    right_pwm = (float)sign_r * clampf(fabsf(right_pwm) + pid_corr_pwm_B, 0.0f, MAZE_PWM_MAX);
  }
#else
  (void)now_tick;
#endif

  drive_dbg_load_left_pwm = left_pwm;
  drive_dbg_load_right_pwm = right_pwm;

#if MAZE_RIGHT_MOTOR_REVERSED
  Load(left_pwm, -right_pwm);
#else
  Load(left_pwm, right_pwm);
#endif
}

static float signed_min_mag(float v, float min_abs)
{
  if (v > 0.0f)
  {
    return (v < min_abs) ? min_abs : v;
  }
  if (v < 0.0f)
  {
    return (-v < min_abs) ? -min_abs : v;
  }
  return 0.0f;
}

static void MazeDrive(float left_pwm, float right_pwm, uint32_t now_tick)
{
  float l = left_pwm;
  float r = right_pwm;
  int8_t straight_sign = 0;
  uint8_t suppress_kick = 0U;

  if ((l * r) > 0.0f && fabsf(l - r) < DRIVE_YAW_HOLD_CMD_DIFF_MAX_PWM)
  {
    straight_sign = (l > 0.0f) ? 1 : -1;
  }
  drive_dbg_straight_sign = straight_sign;
  drive_dbg_yaw_err_deg = 0.0f;
  drive_dbg_yaw_corr_pwm = 0.0f;
  drive_dbg_straight_trim_pwm = 0.0f;

#if DRIVE_YAW_HOLD_ENABLE
  {
    static uint8_t yaw_hold_active = 0U;
    static int8_t yaw_hold_sign = 0;
    static float yaw_hold_ref_rad = 0.0f;

    if (straight_sign != 0 &&
        imu_yaw_usable_for_heading() != 0U)
    {
      if (yaw_hold_active == 0U || yaw_hold_sign != straight_sign)
      {
        float yaw_now_rad = deg_to_rad(yaw);
        yaw_hold_ref_rad = yaw_now_rad;
#if MAZE_SIMPLE_RULE_ENABLE
        if (Maze_Enable != 0U &&
            straight_sign > 0 &&
            maze_simple_cardinal_ref_valid != 0U)
        {
          yaw_hold_ref_rad = maze_simple_nearest_cardinal_yaw_rad(yaw_now_rad);
        }
#endif
        yaw_hold_active = 1U;
        yaw_hold_sign = straight_sign;
      }
      else
      {
        float yaw_now_rad = deg_to_rad(yaw);
        float yaw_err_deg = wrap_pi(yaw_hold_ref_rad - yaw_now_rad) * (180.0f / PI_F);
        float yaw_kp = (straight_sign < 0) ?
                       DRIVE_YAW_HOLD_REVERSE_KP_PWM_PER_DEG :
                       DRIVE_YAW_HOLD_KP_PWM_PER_DEG;
        float yaw_max = (straight_sign < 0) ?
                        DRIVE_YAW_HOLD_REVERSE_CORR_MAX_PWM :
                        DRIVE_YAW_HOLD_CORR_MAX_PWM;
        float yaw_corr = clampf(yaw_err_deg * yaw_kp,
                                -yaw_max,
                                yaw_max);
        drive_dbg_yaw_err_deg = yaw_err_deg;
        drive_dbg_yaw_corr_pwm = yaw_corr;

        if (straight_sign > 0)
        {
          l += yaw_corr;
          r -= yaw_corr;
        }
        else
        {
          l += yaw_corr;
          r -= yaw_corr;
        }
      }
    }
    else
    {
      yaw_hold_active = 0U;
      yaw_hold_sign = 0;
    }
  }
#endif

  if (straight_sign != 0)
  {
    float straight_trim_pwm = (straight_sign > 0) ?
                              DRIVE_STRAIGHT_TRIM_FORWARD_PWM :
                              DRIVE_STRAIGHT_TRIM_REVERSE_PWM;
    drive_dbg_straight_trim_pwm = straight_trim_pwm;
    l += straight_trim_pwm;
    r -= straight_trim_pwm;
  }

#if !DRIVE_YAW_HOLD_ENABLE
  if (straight_sign != 0)
  {
    /* Reserved for builds without IMU yaw hold; trim above still applies. */
  }
#endif

  uint8_t moving = ((fabsf(l) > 1.0f) || (fabsf(r) > 1.0f)) ? 1U : 0U;
  uint8_t dir_changed = 0U;

#if DRIVE_STRAIGHT_SUPPRESS_KICK
  if (straight_sign != 0)
  {
    suppress_kick = 1U;
  }
#endif

#if MAZE_SIMPLE_RULE_ENABLE
  if (Maze_Enable != 0U && DRIVE_AUTO_KICK_ENABLE == 0U)
  {
    suppress_kick = 1U;
  }
  if (Maze_Enable != 0U &&
      (l * r) > 0.0f &&
      fabsf(l - r) > DRIVE_YAW_HOLD_CMD_DIFF_MAX_PWM)
  {
    suppress_kick = 1U;
  }
  if (Maze_Enable != 0U &&
      ((rightdist > 0.0f && rightdist < MAZE_SIMPLE_KICK_SUPPRESS_SIDE_M) ||
       (rightfrontdist > 0.0f && rightfrontdist < MAZE_SIMPLE_KICK_SUPPRESS_SIDE_M) ||
       (leftdist > 0.0f && leftdist < MAZE_SIMPLE_KICK_SUPPRESS_SIDE_M) ||
       (leftfrontdist > 0.0f && leftfrontdist < MAZE_SIMPLE_KICK_SUPPRESS_SIDE_M)))
  {
    suppress_kick = 1U;
  }
#endif

  if ((l > 0.0f && maze_prev_left_cmd < 0.0f) ||
      (l < 0.0f && maze_prev_left_cmd > 0.0f) ||
      (r > 0.0f && maze_prev_right_cmd < 0.0f) ||
      (r < 0.0f && maze_prev_right_cmd > 0.0f))
  {
    dir_changed = 1U;
  }

  if (moving != 0U)
  {
    if ((fabsf(maze_prev_left_cmd) <= 1.0f && fabsf(maze_prev_right_cmd) <= 1.0f) ||
        (dir_changed != 0U))
    {
      maze_drive_kick_tick = now_tick;
    }

    if ((now_tick - maze_drive_kick_tick) < MAZE_KICK_MS
#if MAZE_SIMPLE_RULE_ENABLE && MAZE_SIMPLE_BACKUP_DISABLE_KICK
        && !(Maze_Enable != 0U &&
             maze_simple_state == MAZE_SIMPLE_BACKUP &&
             straight_sign < 0)
#endif
        && suppress_kick == 0U
       )
    {
      l = signed_min_mag(l, MAZE_PWM_KICK);
      r = signed_min_mag(r, MAZE_PWM_KICK);
    }
  }

#if MAZE_SIMPLE_RULE_ENABLE
  if (Maze_Enable != 0U)
  {
    float limit_pwm = (straight_sign != 0) ? DRIVE_AUTO_STRAIGHT_PWM_LIMIT : DRIVE_AUTO_TURN_PWM_LIMIT;
    l = clampf(l, -limit_pwm, limit_pwm);
    r = clampf(r, -limit_pwm, limit_pwm);
  }
#endif

  drive_dbg_left_cmd_pwm = l;
  drive_dbg_right_cmd_pwm = r;
  maze_prev_left_cmd = l;
  maze_prev_right_cmd = r;
  MazeLoad(l, r, now_tick);
}

static void MazeBrake(float brake_pwm, uint32_t now_tick)
{
  drive_pid_reset_all();
  maze_prev_left_cmd = 0.0f;
  maze_prev_right_cmd = 0.0f;
  drive_dbg_left_cmd_pwm = 0.0f;
  drive_dbg_right_cmd_pwm = 0.0f;
  drive_dbg_load_left_pwm = 0.0f;
  drive_dbg_load_right_pwm = 0.0f;
  maze_drive_kick_tick = now_tick;
  Motor_Brake(brake_pwm);
}

static void maze_abort_autonomous(const char *reason, uint32_t now_tick)
{
  Maze_Enable = 0U;
  Fore = 0U;
  Back = 0U;
  Left = 0U;
  Right = 0U;
  control_prev_maze_enable = 0U;
  manual_turn_active = 0U;
  manual_turn_dir = 0U;
  manual_turn_brake_dir = 0U;
  manual_turn_brake_until_tick = 0U;
  manual_turn_settle_until_tick = 0U;
  Manual_Turn_Left_Request = 0U;
  Manual_Turn_Right_Request = 0U;
  Manual_Turn_Cancel_Request = 0U;
  maze_simple_state = MAZE_SIMPLE_FOLLOW;
  maze_simple_pending_turn_state = MAZE_SIMPLE_FOLLOW;
  maze_simple_pending_turn_valid = 0U;
  maze_simple_cardinal_ref_valid = 0U;
  maze_simple_cardinal_ref_yaw_rad = 0.0f;
  maze_simple_post_lock_phase = 0U;
  maze_simple_post_lock_tick = 0U;
  maze_simple_start_align_phase = 0U;
  maze_simple_start_align_tick = 0U;
  maze_simple_prefer_turn_dir = 0;
  maze_simple_center_turn_dir = 0;
  maze_simple_center_turn_topo_openings = 0U;
  maze_centerline_done_this_stop = 0U;
  maze_centerline_done_after_turn = 0U;
  maze_centerline_reset();
#if ODOM_OGM_ENABLE
  maze_simple_start_valid = 0U;
  maze_simple_backup_start_valid = 0U;
  maze_simple_center_start_valid = 0U;
  maze_simple_pre_turn_start_valid = 0U;
#endif
  maze_enter_state(MAZE_STATE_FOLLOW, now_tick);
  maze_last_decide_tick = now_tick;
  maze_have_right_wall = 0U;
  maze_force_dir = 0U;
  maze_force_dir_until_tick = 0U;
  maze_topo_pending_branch_dir = 0;
  maze_topo_pending_branch_until_tick = 0U;
  maze_map_pause_active = 0U;
  maze_map_pace_tick = now_tick;
  maze_uturn_block_until_tick = 0U;
  maze_post_uturn_commit_until_tick = 0U;
  maze_deadend_confirm_tick = 0U;
  maze_intersection_candidate_tick = 0U;
  maze_rule_turn_candidate_dir = 0U;
  maze_rule_turn_candidate_tick = 0U;
  maze_pending_turn_valid = 0U;
  maze_pending_turn_state = MAZE_STATE_FOLLOW;
  selfdbg_side_open_suppressed = 0U;
  selfdbg_backup_stop_pending = 0U;
  selfdbg_backup_turn_dir = 0;
  selfdbg_backup_turn_extra_active = 0U;
  selfdbg_topo_decision_dir = 0;
  selfdbg_topo_pending_dir = 0;
  selfdbg_topo_block_reason = 0U;
  selfdbg_topo_hold_until_tick = 0U;
  MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);

  if (reason != NULL)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)reason, (uint16_t)strlen(reason), 80);
  }
}

static uint32_t maze_turn_duration_ms(maze_state_t state)
{
  switch (state)
  {
    case MAZE_STATE_TURN_RIGHT: return MAZE_TURN_RIGHT_MS;
    case MAZE_STATE_TURN_LEFT:  return MAZE_TURN_LEFT_MS;
    case MAZE_STATE_TURN_BACK:  return MAZE_TURN_BACK_MS;
    default:                    return 0U;
  }
}

static float maze_turn_target_rad(maze_state_t state)
{
  switch (state)
  {
    case MAZE_STATE_TURN_RIGHT: return MAZE_TURN_RIGHT_RAD;
    case MAZE_STATE_TURN_LEFT:  return MAZE_TURN_LEFT_RAD;
    case MAZE_STATE_TURN_BACK:  return MAZE_TURN_BACK_RAD;
    default:                    return 0.0f;
  }
}

static void maze_schedule_turn(maze_state_t turn_state, uint32_t now_tick)
{
  if (turn_state != MAZE_STATE_TURN_RIGHT &&
      turn_state != MAZE_STATE_TURN_LEFT &&
      turn_state != MAZE_STATE_TURN_BACK)
  {
    return;
  }

  maze_pending_turn_state = turn_state;
  maze_pending_turn_valid = 1U;
  maze_enter_state(MAZE_STATE_PRETURN_STOP, now_tick);
}

static uint8_t lidar_is_ready_for_runtime(void)
{
  return (lidar_ready != 0U) ? 1U : 0U;
}

static uint8_t imu_is_ready_for_runtime(void)
{
  return (imu_data_valid != 0U) ? 1U : 0U;
}

static void Calibration_Print(const char *msg)
{
  if (msg == NULL)
  {
    return;
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)strlen(msg), 60);
}

static void Calibration_HandleCommand(void)
{
  if (Calib_Stop_Request != 0U)
  {
    Calib_Stop_Request = 0U;
    calib_start_armed = 0U;
    calib_wait_imu_reported = 0U;
    calib_state = CALIB_STATE_IDLE;
    Load(0, 0);
    maze_prev_left_cmd = 0.0f;
    maze_prev_right_cmd = 0.0f;
    Calibration_Print("CAL:ABORT\r\n");
  }

  if (Calib_Start_Request != 0U)
  {
    Calib_Start_Request = 0U;
    if (Maze_Enable != 0U ||
        maze_return_active != 0U ||
        calib_start_armed != 0U ||
        calib_state != CALIB_STATE_IDLE ||
        Fore != 0U ||
        Back != 0U ||
        Left != 0U ||
        Right != 0U)
    {
      Calibration_Print("CAL:BUSY\r\n");
      return;
    }

    calib_start_armed = 1U;
    calib_wait_imu_reported = 0U;
    Calibration_Print("CAL:ARMED\r\n");
  }
}

#if ODOM_OGM_ENABLE
static void Calibration_Begin(uint32_t now_tick)
{
  Maze_Enable = 0U;
  Fore = 0U;
  Back = 0U;
  Left = 0U;
  Right = 0U;
  maze_prev_left_cmd = 0.0f;
  maze_prev_right_cmd = 0.0f;
  maze_drive_kick_tick = now_tick;

  Odom_Reset();
  __disable_irq();
  odom_pulse_accum_left = 0;
  odom_pulse_accum_right = 0;
  pulse_sum = 0;
  pulse_sum_A = 0;
  pulse_sum_B = 0;
  __enable_irq();

  calib_straight_odom_m = 0.0f;
  calib_straight_ref_yaw_rad = wrap_pi(deg_to_rad(yaw));
  calib_turn_accum_rad = 0.0f;
  calib_turn_imu_accum_rad = 0.0f;
  calib_last_theta_rad = odom_theta_rad;
  calib_last_yaw_rad = wrap_pi(deg_to_rad(yaw));
  calib_turn_abs_pulse_left = 0.0f;
  calib_turn_abs_pulse_right = 0.0f;
  calib_turn_stall_tick = 0U;
  calib_turn_last_rekick_tick = HAL_GetTick();

  calib_state = CALIB_STATE_PREPARE;
  calib_state_tick = HAL_GetTick();
  Calibration_Print("CAL:START\r\n");
}

static void Calibration_Abort(const char *reason)
{
  calib_start_armed = 0U;
  calib_state = CALIB_STATE_IDLE;
  Load(0, 0);
  maze_prev_left_cmd = 0.0f;
  maze_prev_right_cmd = 0.0f;
  if (reason != NULL)
  {
    Calibration_Print(reason);
  }
}
#endif

static uint8_t Calibration_Task(uint32_t now_tick)
{
#if !ODOM_OGM_ENABLE
  (void)now_tick;
  if (calib_start_armed != 0U)
  {
    calib_start_armed = 0U;
    Calibration_Print("CAL:ODOM_DISABLED\r\n");
  }
  return 0U;
#else
  if (calib_state == CALIB_STATE_IDLE)
  {
    if (calib_start_armed == 0U)
    {
      calib_wait_imu_reported = 0U;
      return 0U;
    }

    if (imu_is_ready_for_runtime() == 0U)
    {
      Load(0, 0);
      if (calib_wait_imu_reported == 0U)
      {
        Calibration_Print("CAL:WAIT IMU\r\n");
        calib_wait_imu_reported = 1U;
      }
      return 1U;
    }
    calib_wait_imu_reported = 0U;

    if (lidar_is_ready_for_runtime() == 0U)
    {
      Load(0, 0);
      return 1U;
    }

    Calibration_Begin(now_tick);
    return 1U;
  }

  if (lidar_is_ready_for_runtime() == 0U)
  {
    Calibration_Abort("CAL:LR_LOST\r\n");
    return 1U;
  }

  if (imu_is_ready_for_runtime() == 0U)
  {
    Calibration_Abort("CAL:IMU_LOST\r\n");
    return 1U;
  }

  switch (calib_state)
  {
    case CALIB_STATE_PREPARE:
      Load(0, 0);
      if ((now_tick - calib_state_tick) >= CALIB_PREPARE_MS)
      {
        calib_start_x_m = odom_x_m;
        calib_start_y_m = odom_y_m;
        calib_straight_ref_yaw_rad = wrap_pi(deg_to_rad(yaw));
        calib_last_theta_rad = odom_theta_rad;
        calib_turn_accum_rad = 0.0f;
        calib_state = CALIB_STATE_STRAIGHT;
        calib_state_tick = now_tick;
        Calibration_Print("CAL:STRAIGHT\r\n");
      }
      break;

    case CALIB_STATE_STRAIGHT:
    {
      float dx;
      float dy;
      float cur_yaw;
      float yaw_err;
      float corr;
      float left_cmd;
      float right_cmd;

      cur_yaw = wrap_pi(deg_to_rad(yaw));
      yaw_err = wrap_pi(calib_straight_ref_yaw_rad - cur_yaw);
      corr = clampf(yaw_err * CALIB_STRAIGHT_YAW_KP,
                    -CALIB_STRAIGHT_YAW_CORR_MAX,
                    CALIB_STRAIGHT_YAW_CORR_MAX);
      left_cmd = CALIB_PWM_STRAIGHT + corr;
      right_cmd = CALIB_PWM_STRAIGHT - corr;
      MazeDrive(left_cmd, right_cmd, now_tick);
      if ((now_tick - calib_state_tick) >= CALIB_STRAIGHT_MS)
      {
        Load(0, 0);
        maze_prev_left_cmd = 0.0f;
        maze_prev_right_cmd = 0.0f;
        dx = odom_x_m - calib_start_x_m;
        dy = odom_y_m - calib_start_y_m;
        calib_straight_odom_m = sqrtf((dx * dx) + (dy * dy));
        calib_state = CALIB_STATE_PAUSE;
        calib_state_tick = now_tick;
        Calibration_Print("CAL:PAUSE\r\n");
      }
      break;
    }

    case CALIB_STATE_PAUSE:
      Load(0, 0);
      if ((now_tick - calib_state_tick) >= CALIB_PAUSE_MS)
      {
        calib_last_theta_rad = odom_theta_rad;
        calib_last_yaw_rad = wrap_pi(deg_to_rad(yaw));
        calib_turn_accum_rad = 0.0f;
        calib_turn_imu_accum_rad = 0.0f;
        calib_turn_abs_pulse_left = 0.0f;
        calib_turn_abs_pulse_right = 0.0f;
        calib_turn_stall_tick = 0U;
        calib_turn_last_rekick_tick = now_tick;
        calib_state = CALIB_STATE_TURN;
        calib_state_tick = now_tick;
        Calibration_Print("CAL:TURN\r\n");
      }
      break;

    case CALIB_STATE_TURN:
    {
      float cur_theta = odom_theta_rad;
      float cur_yaw = wrap_pi(deg_to_rad(yaw));
      float dtheta_enc = wrap_pi(cur_theta - calib_last_theta_rad);
      float dtheta_imu = wrap_pi(cur_yaw - calib_last_yaw_rad);
      float turn_motion_rad = fabsf(dtheta_enc) + fabsf(dtheta_imu);
      float cpm_new = COUNTS_PER_METER;
      float wb_new = ODOM_WHEEL_BASE_M;
      float pulse_max = 0.0f;
      float pulse_min = 0.0f;
      float pulse_ratio = 1.0f;
      uint8_t turn_timeout = 0U;
      char msg[300];
      int n;

      calib_last_theta_rad = cur_theta;
      calib_last_yaw_rad = cur_yaw;
      calib_turn_accum_rad += fabsf(dtheta_enc);
      calib_turn_imu_accum_rad += fabsf(dtheta_imu);
      calib_turn_abs_pulse_left += fabsf(fdb_A);
      calib_turn_abs_pulse_right += fabsf(fdb_B);

      if (turn_motion_rad < CALIB_TURN_STALL_MOTION_RAD)
      {
        if (calib_turn_stall_tick == 0U)
        {
          calib_turn_stall_tick = now_tick;
        }

        if ((now_tick - calib_turn_last_rekick_tick) >= CALIB_TURN_REKICK_MS)
        {
          maze_prev_left_cmd = 0.0f;
          maze_prev_right_cmd = 0.0f;
          maze_drive_kick_tick = now_tick;
          calib_turn_last_rekick_tick = now_tick;
        }

        if ((now_tick - calib_turn_stall_tick) >= CALIB_TURN_STALL_ABORT_MS)
        {
          Calibration_Abort("CAL:ABORT TURN_STALL\r\n");
          return 1U;
        }
      }
      else
      {
        calib_turn_stall_tick = 0U;
      }

      MazeDrive(-CALIB_PWM_TURN, CALIB_PWM_TURN, now_tick);

      if ((now_tick - calib_state_tick) >= CALIB_TURN_TIMEOUT_MS)
      {
        turn_timeout = 1U;
      }

      if (calib_turn_imu_accum_rad >= CALIB_TURN_TARGET_RAD ||
          turn_timeout != 0U)
      {
        Load(0, 0);
        maze_prev_left_cmd = 0.0f;
        maze_prev_right_cmd = 0.0f;

        pulse_max = (calib_turn_abs_pulse_left > calib_turn_abs_pulse_right) ? calib_turn_abs_pulse_left : calib_turn_abs_pulse_right;
        pulse_min = (calib_turn_abs_pulse_left < calib_turn_abs_pulse_right) ? calib_turn_abs_pulse_left : calib_turn_abs_pulse_right;
        if (pulse_max > 1.0f)
        {
          pulse_ratio = pulse_min / pulse_max;
        }

        if (calib_turn_imu_accum_rad < CALIB_TURN_MIN_IMU_RAD)
        {
          n = snprintf(msg, sizeof(msg),
                       "CAL:ABORT TURN_SHORT TIMU=%.3f/<%.3f\r\n",
                       calib_turn_imu_accum_rad,
                       CALIB_TURN_MIN_IMU_RAD);
          if (n > 0)
          {
            uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
            (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
          }
          calib_start_armed = 0U;
          calib_state = CALIB_STATE_IDLE;
          break;
        }

        if (pulse_ratio < CALIB_TURN_PULSE_RATIO_MIN)
        {
          n = snprintf(msg, sizeof(msg),
                       "CAL:ABORT TURN_UNBALANCED PR=%.2f/<%.2f\r\n",
                       pulse_ratio,
                       CALIB_TURN_PULSE_RATIO_MIN);
          if (n > 0)
          {
            uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
            (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
          }
          calib_start_armed = 0U;
          calib_state = CALIB_STATE_IDLE;
          break;
        }

        if (calib_straight_odom_m > 0.02f && CALIB_TRUE_DIST_M > 0.02f)
        {
          cpm_new = COUNTS_PER_METER * (calib_straight_odom_m / CALIB_TRUE_DIST_M);
        }

        if (calib_turn_accum_rad > 0.2f && calib_turn_imu_accum_rad > 0.2f)
        {
          wb_new = ODOM_WHEEL_BASE_M * (calib_turn_accum_rad / calib_turn_imu_accum_rad);
        }

        n = snprintf(msg, sizeof(msg),
                     "CAL:OK D=%.3f TENC=%.3f TIMU=%.3f\r\n"
                     "CAL:SET CPM=%.1f WB=%.4f\r\n"
                     "CAL:CFG DTRUE=%.2fm TTAR=%.2frad PR=%.2f\r\n",
                     calib_straight_odom_m,
                     calib_turn_accum_rad,
                     calib_turn_imu_accum_rad,
                     cpm_new,
                     wb_new,
                     CALIB_TRUE_DIST_M,
                     CALIB_TURN_TARGET_RAD,
                     pulse_ratio);
        if (n > 0)
        {
          uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
          (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
        }

        calib_start_armed = 0U;
        calib_state = CALIB_STATE_IDLE;
      }
      break;
    }

    default:
      Calibration_Abort("CAL:STATE_ERR\r\n");
      break;
  }

  return 1U;
#endif
}

static void maze_enter_state(maze_state_t state, uint32_t now_tick)
{
  maze_state = state;
  maze_state_tick = now_tick;
  if (state != MAZE_STATE_PRETURN_STOP)
  {
    maze_pending_turn_valid = 0U;
  }
  if (state != MAZE_STATE_FOLLOW)
  {
    maze_map_pause_active = 0U;
    maze_map_pace_tick = now_tick;
    maze_deadend_confirm_tick = 0U;
    maze_intersection_candidate_tick = 0U;
    maze_rule_turn_candidate_dir = 0U;
    maze_rule_turn_candidate_tick = 0U;
  }
  else if (maze_map_pace_tick == 0U)
  {
    maze_map_pace_tick = now_tick;
  }
  if (state == MAZE_STATE_TURN_RIGHT ||
      state == MAZE_STATE_TURN_LEFT ||
      state == MAZE_STATE_TURN_BACK)
  {
    maze_simple_turn_timeout_failed = 0U;
    maze_simple_turn_last_turned_rad = 0.0f;
    maze_simple_turn_last_signed_rad = 0.0f;
    maze_simple_turn_last_target_rad = 0.0f;
    maze_simple_turn_last_yaw_gate = 0U;
#if ODOM_OGM_ENABLE
    maze_turn_start_theta_rad = odom_theta_rad;
    if (imu_yaw_usable_for_heading() != 0U)
    {
      maze_turn_start_yaw_rad = deg_to_rad(yaw);
      maze_turn_start_yaw_valid = 1U;
    }
    else
    {
      maze_turn_start_yaw_valid = 0U;
    }
#endif
    maze_turn_start_valid = 1U;
  }
  else
  {
    maze_turn_start_valid = 0U;
    maze_turn_start_yaw_valid = 0U;
  }
}

void RefreshLidarDistances(uint32_t now_tick)
{
  if ((now_tick - lidar_fast_update_tick) < DIST_FAST_UPDATE_MS)
  {
    return;
  }
  lidar_fast_update_tick = now_tick;

  fordist = update_lidar_distance_filter(Lidar_GetFrontDistanceM(),
                                         &fordist_filtered,
                                         &fordist_last_valid_tick,
                                         now_tick);
  rightdist = update_lidar_distance_filter(Lidar_GetRightDistanceM(),
                                           &rightdist_filtered,
                                           &rightdist_last_valid_tick,
                                           now_tick);
  rightfrontdist = update_lidar_distance_filter(Lidar_GetRightFrontDistanceM(),
                                                &rightfrontdist_filtered,
                                                &rightfrontdist_last_valid_tick,
                                                now_tick);
  rightreardist = update_lidar_distance_filter(Lidar_GetRightRearDistanceM(),
                                               &rightreardist_filtered,
                                               &rightreardist_last_valid_tick,
                                               now_tick);
  reardist = update_lidar_distance_filter(Lidar_GetRearDistanceM(),
                                          &reardist_filtered,
                                          &reardist_last_valid_tick,
                                          now_tick);
  leftreardist = update_lidar_distance_filter(Lidar_GetLeftRearDistanceM(),
                                              &leftreardist_filtered,
                                              &leftreardist_last_valid_tick,
                                              now_tick);
  leftdist = update_lidar_distance_filter(Lidar_GetLeftDistanceM(),
                                          &leftdist_filtered,
                                          &leftdist_last_valid_tick,
                                          now_tick);
  leftfrontdist = update_lidar_distance_filter(Lidar_GetLeftFrontDistanceM(),
                                               &leftfrontdist_filtered,
                                               &leftfrontdist_last_valid_tick,
                                               now_tick);

  if (lidar_ranges_all_zero() == 0U)
  {
    lidar_last_nonzero_tick = now_tick;
    if (lidar_ready == 0U)
    {
      if (lidar_ready_stable_count < 255U)
      {
        lidar_ready_stable_count++;
      }
      if (lidar_ready_stable_count >= LIDAR_READY_STABLE_COUNT)
      {
        lidar_ready = 1U;
      }
    }
  }
  else
  {
    if (lidar_ready == 0U)
    {
      lidar_ready_stable_count = 0U;
    }
    else if ((now_tick - lidar_last_nonzero_tick) > DIST_HOLD_MS)
    {
      /* If LiDAR has been all-zero for a while, force re-wait to avoid blind driving. */
      lidar_ready = 0U;
      lidar_ready_stable_count = 0U;
    }
  }
}

static uint8_t maze_simple_front_blocked(float front_m)
{
  if (fordist_last_valid_tick == 0U ||
      (HAL_GetTick() - fordist_last_valid_tick) > MAZE_SIMPLE_DECISION_FRESH_MS)
  {
    return 0U;
  }

  return (front_m > 0.0f && front_m < MAZE_SIMPLE_FRONT_BLOCK_M) ? 1U : 0U;
}

static uint8_t maze_simple_front_clear(float front_m)
{
  if (fordist_last_valid_tick == 0U ||
      (HAL_GetTick() - fordist_last_valid_tick) > MAZE_SIMPLE_DECISION_FRESH_MS)
  {
    return 0U;
  }

  return (front_m > MAZE_SIMPLE_FRONT_CLEAR_M) ? 1U : 0U;
}

static uint8_t maze_simple_range_fresh(uint32_t now_tick, uint32_t last_valid_tick)
{
  return (last_valid_tick != 0U &&
          (now_tick - last_valid_tick) <= MAZE_SIMPLE_DECISION_FRESH_MS) ? 1U : 0U;
}

static uint8_t maze_simple_side_pair_ready(uint32_t now_tick,
                                           uint32_t side_tick,
                                           uint32_t front_diag_tick,
                                           uint32_t rear_diag_tick)
{
  if (maze_simple_range_fresh(now_tick, side_tick) != 0U)
  {
    return 1U;
  }

  return (maze_simple_range_fresh(now_tick, front_diag_tick) != 0U &&
          maze_simple_range_fresh(now_tick, rear_diag_tick) != 0U) ? 1U : 0U;
}

static uint8_t maze_simple_right_hard_blocked(uint32_t now_tick)
{
  if (maze_simple_range_fresh(now_tick, rightdist_last_valid_tick) != 0U &&
      rightdist > 0.0f &&
      rightdist < MAZE_SIMPLE_BACKUP_SIDE_DANGER_M)
  {
    return 1U;
  }

  if (maze_simple_range_fresh(now_tick, rightreardist_last_valid_tick) != 0U &&
      rightreardist > 0.0f &&
      rightreardist < MAZE_SIMPLE_SIDE_DIAG_BLOCK_M)
  {
    return 1U;
  }

  return 0U;
}

static uint8_t maze_simple_right_probe_candidate(uint32_t now_tick, float front_m)
{
  uint8_t side_fresh = maze_simple_range_fresh(now_tick, rightdist_last_valid_tick);
  uint8_t front_diag_fresh = maze_simple_range_fresh(now_tick, rightfrontdist_last_valid_tick);

  if (front_m <= MAZE_SIMPLE_RIGHT_PROBE_ROOM_MIN_M)
  {
    return 0U;
  }

  if (side_fresh == 0U || front_diag_fresh == 0U)
  {
    return 0U;
  }

  if (rightdist <= MAZE_SIMPLE_RIGHT_PROBE_SIDE_MIN_M ||
      rightfrontdist <= MAZE_SIMPLE_RIGHT_PROBE_FRONT_MIN_M)
  {
    return 0U;
  }

  if (rightdist >= MAZE_SIMPLE_RIGHT_OPEN_M ||
      rightfrontdist >= MAZE_SIMPLE_RIGHT_FRONT_OPEN_M)
  {
    return 0U;
  }

  return 1U;
}

static uint8_t maze_simple_side_open_with_diags(float side_m,
                                                uint32_t side_tick,
                                                float front_diag_m,
                                                uint32_t front_diag_tick,
                                                float rear_diag_m,
                                                uint32_t rear_diag_tick,
                                                float side_open_m)
{
  uint32_t now_tick = HAL_GetTick();
  uint8_t side_fresh = maze_simple_range_fresh(now_tick, side_tick);
  uint8_t front_fresh = maze_simple_range_fresh(now_tick, front_diag_tick);
  uint8_t rear_fresh = maze_simple_range_fresh(now_tick, rear_diag_tick);
  uint8_t front_open = (front_fresh != 0U &&
                        front_diag_m > MAZE_SIMPLE_SIDE_DIAG_OPEN_M) ? 1U : 0U;
  uint8_t rear_open = (rear_fresh != 0U &&
                       rear_diag_m > MAZE_SIMPLE_SIDE_DIAG_OPEN_M) ? 1U : 0U;
  uint8_t front_block = (front_fresh != 0U &&
                         front_diag_m > 0.0f &&
                         front_diag_m < MAZE_SIMPLE_SIDE_DIAG_BLOCK_M) ? 1U : 0U;
  uint8_t rear_block = (rear_fresh != 0U &&
                        rear_diag_m > 0.0f &&
                        rear_diag_m < MAZE_SIMPLE_SIDE_DIAG_BLOCK_M) ? 1U : 0U;

  if (side_fresh != 0U && side_m > side_open_m)
  {
    return 1U;
  }

  if ((front_block != 0U && rear_open == 0U) ||
      (rear_block != 0U && front_open == 0U))
  {
    return 0U;
  }

  if (front_open != 0U && rear_open != 0U)
  {
    if (side_fresh != 0U &&
        side_m > 0.0f &&
        side_m < MAZE_SIMPLE_BACKUP_SIDE_DANGER_M)
    {
      return 0U;
    }
    return 1U;
  }

  return 0U;
}

static float maze_simple_side_score_with_diags(float side_m,
                                               uint32_t side_tick,
                                               float front_diag_m,
                                               uint32_t front_diag_tick,
                                               float rear_diag_m,
                                               uint32_t rear_diag_tick)
{
  uint32_t now_tick = HAL_GetTick();
  uint8_t side_fresh = maze_simple_range_fresh(now_tick, side_tick);
  uint8_t front_fresh = maze_simple_range_fresh(now_tick, front_diag_tick);
  uint8_t rear_fresh = maze_simple_range_fresh(now_tick, rear_diag_tick);
  float score_m = (side_fresh != 0U) ? side_m : 0.0f;
  uint8_t front_open = (front_fresh != 0U &&
                        front_diag_m > MAZE_SIMPLE_SIDE_DIAG_OPEN_M) ? 1U : 0U;
  uint8_t rear_open = (rear_fresh != 0U &&
                       rear_diag_m > MAZE_SIMPLE_SIDE_DIAG_OPEN_M) ? 1U : 0U;

  if (side_fresh != 0U &&
      side_m > 0.0f &&
      side_m < MAZE_SIMPLE_BACKUP_SIDE_DANGER_M)
  {
    return score_m;
  }

  if (front_open != 0U && rear_open != 0U)
  {
    float diag_score_m = front_diag_m;
    if (rear_diag_m > diag_score_m)
    {
      diag_score_m = rear_diag_m;
    }
    diag_score_m *= MAZE_SIMPLE_SIDE_DIAG_SCORE_SCALE;
    if (diag_score_m > score_m)
    {
      score_m = diag_score_m;
    }
  }

  return score_m;
}

static uint8_t maze_simple_right_open(void)
{
  uint32_t now_tick = HAL_GetTick();
  uint8_t open = maze_simple_side_open_with_diags(rightdist,
                                                  rightdist_last_valid_tick,
                                                  rightfrontdist,
                                                  rightfrontdist_last_valid_tick,
                                                  rightreardist,
                                                  rightreardist_last_valid_tick,
                                                  MAZE_SIMPLE_RIGHT_OPEN_M);

  if (open != 0U)
  {
    return 1U;
  }

  if (maze_simple_range_fresh(now_tick, rightfrontdist_last_valid_tick) != 0U &&
      rightfrontdist > MAZE_SIMPLE_RIGHT_FRONT_OPEN_M &&
      maze_simple_right_hard_blocked(now_tick) == 0U &&
      (maze_simple_range_fresh(now_tick, rightreardist_last_valid_tick) == 0U ||
       rightreardist > MAZE_SIMPLE_SIDE_DIAG_BLOCK_M))
  {
    return 1U;
  }

  return 0U;
}

static uint8_t maze_simple_side_entry_quality_ok(uint8_t decision, uint32_t now_tick)
{
  uint8_t front_fresh = maze_simple_range_fresh(now_tick, fordist_last_valid_tick);
  uint8_t side_fresh;
  uint8_t front_diag_fresh;
  float side_m;
  float front_diag_m;

  if (front_fresh != 0U &&
      fordist > 0.0f &&
      fordist < MAZE_TOPO_COMMIT_SIDE_FRONT_MIN_M)
  {
    return 0U;
  }

  if (decision == MAZE_TOPO_DECISION_RIGHT)
  {
    side_fresh = maze_simple_range_fresh(now_tick, rightdist_last_valid_tick);
    front_diag_fresh = maze_simple_range_fresh(now_tick, rightfrontdist_last_valid_tick);
    side_m = rightdist;
    front_diag_m = rightfrontdist;
  }
  else if (decision == MAZE_TOPO_DECISION_LEFT)
  {
    side_fresh = maze_simple_range_fresh(now_tick, leftdist_last_valid_tick);
    front_diag_fresh = maze_simple_range_fresh(now_tick, leftfrontdist_last_valid_tick);
    side_m = leftdist;
    front_diag_m = leftfrontdist;
  }
  else
  {
    return 1U;
  }

  if (side_fresh != 0U &&
      side_m > 0.0f &&
      side_m < MAZE_TOPO_COMMIT_SIDE_CLEAR_MIN_M)
  {
    return 0U;
  }

  if (front_diag_fresh != 0U &&
      front_diag_m > 0.0f &&
      front_diag_m < MAZE_TOPO_COMMIT_SIDE_DIAG_MIN_M)
  {
    return 0U;
  }

  return 1U;
}

static uint8_t maze_simple_side_entry_panic(uint8_t decision, uint32_t now_tick)
{
  uint8_t side_fresh;
  uint8_t front_diag_fresh;
  float side_m;
  float front_diag_m;

  if (maze_simple_range_fresh(now_tick, fordist_last_valid_tick) != 0U &&
      fordist > 0.0f &&
      fordist < MAZE_TOPO_SIDE_ENTRY_PANIC_FRONT_M)
  {
    return 1U;
  }

  if (decision == MAZE_TOPO_DECISION_RIGHT)
  {
    side_fresh = maze_simple_range_fresh(now_tick, rightdist_last_valid_tick);
    front_diag_fresh = maze_simple_range_fresh(now_tick, rightfrontdist_last_valid_tick);
    side_m = rightdist;
    front_diag_m = rightfrontdist;
  }
  else if (decision == MAZE_TOPO_DECISION_LEFT)
  {
    side_fresh = maze_simple_range_fresh(now_tick, leftdist_last_valid_tick);
    front_diag_fresh = maze_simple_range_fresh(now_tick, leftfrontdist_last_valid_tick);
    side_m = leftdist;
    front_diag_m = leftfrontdist;
  }
  else
  {
    return 0U;
  }

  if (side_fresh != 0U &&
      side_m > 0.0f &&
      side_m < MAZE_TOPO_SIDE_ENTRY_PANIC_SIDE_M)
  {
    return 1U;
  }

  if (front_diag_fresh != 0U &&
      front_diag_m > 0.0f &&
      front_diag_m < MAZE_TOPO_SIDE_ENTRY_PANIC_DIAG_M)
  {
    return 1U;
  }

  return 0U;
}

static uint8_t maze_simple_revisit_side_turn_ready(uint8_t decision, uint32_t now_tick)
{
  maze_simple_state_t turn_state;
  uint8_t side_fresh;
  uint8_t front_diag_fresh;
  float side_m;
  float front_diag_m;

  if (decision == MAZE_TOPO_DECISION_RIGHT)
  {
    turn_state = MAZE_SIMPLE_TURN_RIGHT;
    side_fresh = maze_simple_range_fresh(now_tick, rightdist_last_valid_tick);
    front_diag_fresh = maze_simple_range_fresh(now_tick, rightfrontdist_last_valid_tick);
    side_m = rightdist;
    front_diag_m = rightfrontdist;
  }
  else if (decision == MAZE_TOPO_DECISION_LEFT)
  {
    turn_state = MAZE_SIMPLE_TURN_LEFT;
    side_fresh = maze_simple_range_fresh(now_tick, leftdist_last_valid_tick);
    front_diag_fresh = maze_simple_range_fresh(now_tick, leftfrontdist_last_valid_tick);
    side_m = leftdist;
    front_diag_m = leftfrontdist;
  }
  else
  {
    return 0U;
  }

  if (maze_simple_turn_clear_for_state(turn_state, now_tick) == 0U)
  {
    return 0U;
  }

  if (maze_simple_range_fresh(now_tick, fordist_last_valid_tick) == 0U ||
      fordist <= 0.0f ||
      fordist < MAZE_TOPO_REVISIT_TURN_FRONT_MIN_M)
  {
    return 0U;
  }

  if (side_fresh == 0U ||
      side_m <= 0.0f ||
      side_m < MAZE_TOPO_REVISIT_TURN_SIDE_MIN_M)
  {
    return 0U;
  }

  if (front_diag_fresh == 0U ||
      front_diag_m <= 0.0f ||
      front_diag_m < MAZE_TOPO_REVISIT_TURN_DIAG_MIN_M)
  {
    return 0U;
  }

  return 1U;
}

static uint8_t maze_simple_left_open(void)
{
  return maze_simple_side_open_with_diags(leftdist,
                                          leftdist_last_valid_tick,
                                          leftfrontdist,
                                          leftfrontdist_last_valid_tick,
                                          leftreardist,
                                          leftreardist_last_valid_tick,
                                          MAZE_SIMPLE_LEFT_OPEN_M);
}

static uint8_t maze_simple_decision_side_unknown(uint32_t now_tick, uint32_t last_valid_tick)
{
  return (last_valid_tick == 0U ||
          (now_tick - last_valid_tick) > MAZE_SIMPLE_DECISION_FRESH_MS) ? 1U : 0U;
}

static uint8_t maze_simple_decision_cardinal_ready(uint32_t now_tick, uint32_t state_start_tick)
{
  uint8_t wait_timeout = ((now_tick - state_start_tick) >= MAZE_SIMPLE_DECISION_WAIT_MAX_MS) ? 1U : 0U;

  if (fordist_last_valid_tick == 0U)
  {
    return 0U;
  }

  if ((now_tick - fordist_last_valid_tick) > MAZE_SIMPLE_DECISION_FRESH_MS)
  {
    return 0U;
  }

  if (wait_timeout == 0U &&
      (maze_simple_side_pair_ready(now_tick,
                                   rightdist_last_valid_tick,
                                   rightfrontdist_last_valid_tick,
                                   rightreardist_last_valid_tick) == 0U ||
       maze_simple_side_pair_ready(now_tick,
                                   leftdist_last_valid_tick,
                                   leftfrontdist_last_valid_tick,
                                   leftreardist_last_valid_tick) == 0U))
  {
    return 0U;
  }

  return 1U;
}

static float maze_simple_backup_right_score(void)
{
  float score_m = maze_simple_side_score_with_diags(rightdist,
                                                    rightdist_last_valid_tick,
                                                    rightfrontdist,
                                                    rightfrontdist_last_valid_tick,
                                                    rightreardist,
                                                    rightreardist_last_valid_tick);
  uint32_t now_tick = HAL_GetTick();
  uint8_t side_fresh = maze_simple_range_fresh(now_tick, rightdist_last_valid_tick);
  uint8_t front_fresh = maze_simple_range_fresh(now_tick, rightfrontdist_last_valid_tick);
  uint8_t rear_fresh = maze_simple_range_fresh(now_tick, rightreardist_last_valid_tick);
  uint8_t front_block = (front_fresh != 0U &&
                         rightfrontdist > 0.0f &&
                         rightfrontdist < MAZE_SIMPLE_SIDE_DIAG_BLOCK_M) ? 1U : 0U;

  if (side_fresh != 0U &&
      rightdist > 0.0f &&
      rightdist < MAZE_SIMPLE_BACKUP_SIDE_DANGER_M)
  {
    return score_m;
  }

  if (rear_fresh != 0U &&
      rightreardist > MAZE_SIMPLE_BACKUP_SIDE_OPEN_M &&
      front_block == 0U &&
      rightreardist > score_m)
  {
    score_m = rightreardist;
  }

  return score_m;
}

static float maze_simple_backup_left_score(void)
{
  float score_m = maze_simple_side_score_with_diags(leftdist,
                                                    leftdist_last_valid_tick,
                                                    leftfrontdist,
                                                    leftfrontdist_last_valid_tick,
                                                    leftreardist,
                                                    leftreardist_last_valid_tick);
  uint32_t now_tick = HAL_GetTick();
  uint8_t side_fresh = maze_simple_range_fresh(now_tick, leftdist_last_valid_tick);
  uint8_t front_fresh = maze_simple_range_fresh(now_tick, leftfrontdist_last_valid_tick);
  uint8_t rear_fresh = maze_simple_range_fresh(now_tick, leftreardist_last_valid_tick);
  uint8_t front_block = (front_fresh != 0U &&
                         leftfrontdist > 0.0f &&
                         leftfrontdist < MAZE_SIMPLE_SIDE_DIAG_BLOCK_M) ? 1U : 0U;

  if (side_fresh != 0U &&
      leftdist > 0.0f &&
      leftdist < MAZE_SIMPLE_BACKUP_SIDE_DANGER_M)
  {
    return score_m;
  }

  if (rear_fresh != 0U &&
      leftreardist > MAZE_SIMPLE_BACKUP_SIDE_OPEN_M &&
      front_block == 0U &&
      leftreardist > score_m)
  {
    score_m = leftreardist;
  }

  return score_m;
}

static uint8_t maze_simple_backup_right_open(void)
{
  return (maze_simple_backup_right_score() > MAZE_SIMPLE_BACKUP_SIDE_OPEN_M) ? 1U : 0U;
}

static uint8_t maze_simple_backup_left_open(void)
{
  return (maze_simple_backup_left_score() > MAZE_SIMPLE_BACKUP_SIDE_OPEN_M) ? 1U : 0U;
}

static uint8_t maze_simple_backup_right_topo_open(uint32_t now_tick)
{
  uint8_t score_open = (maze_simple_backup_right_score() > MAZE_SIMPLE_BACKUP_TOPO_SIDE_OPEN_M) ? 1U : 0U;
  uint8_t side_fresh = maze_simple_range_fresh(now_tick, rightdist_last_valid_tick);
  uint8_t front_diag_fresh = maze_simple_range_fresh(now_tick, rightfrontdist_last_valid_tick);
  uint8_t rear_diag_fresh = maze_simple_range_fresh(now_tick, rightreardist_last_valid_tick);

  if (score_open == 0U)
  {
    return 0U;
  }

  if (side_fresh != 0U &&
      rightdist > 0.0f &&
      rightdist < MAZE_SIMPLE_BACKUP_TOPO_DIAG_MIN_M)
  {
    return 0U;
  }

  if (front_diag_fresh != 0U &&
      rightfrontdist > 0.0f &&
      rightfrontdist < MAZE_SIMPLE_BACKUP_TOPO_DIAG_MIN_M)
  {
    return 0U;
  }

  if (rear_diag_fresh != 0U &&
      rightreardist > 0.0f &&
      rightreardist < MAZE_SIMPLE_BACKUP_TOPO_FRONT_MIN_M)
  {
    return 0U;
  }

  return 1U;
}

static uint8_t maze_simple_backup_left_topo_open(uint32_t now_tick)
{
  uint8_t score_open = (maze_simple_backup_left_score() > MAZE_SIMPLE_BACKUP_TOPO_SIDE_OPEN_M) ? 1U : 0U;
  uint8_t side_fresh = maze_simple_range_fresh(now_tick, leftdist_last_valid_tick);
  uint8_t front_diag_fresh = maze_simple_range_fresh(now_tick, leftfrontdist_last_valid_tick);
  uint8_t rear_diag_fresh = maze_simple_range_fresh(now_tick, leftreardist_last_valid_tick);

  if (score_open == 0U)
  {
    return 0U;
  }

  if (side_fresh != 0U &&
      leftdist > 0.0f &&
      leftdist < MAZE_SIMPLE_BACKUP_TOPO_DIAG_MIN_M)
  {
    return 0U;
  }

  if (front_diag_fresh != 0U &&
      leftfrontdist > 0.0f &&
      leftfrontdist < MAZE_SIMPLE_BACKUP_TOPO_DIAG_MIN_M)
  {
    return 0U;
  }

  if (rear_diag_fresh != 0U &&
      leftreardist > 0.0f &&
      leftreardist < MAZE_SIMPLE_BACKUP_TOPO_FRONT_MIN_M)
  {
    return 0U;
  }

  return 1U;
}

static int8_t maze_simple_backup_choose_turn_dir(void)
{
#if MAZE_SIMPLE_BACKUP_SIDE_TURN_ENABLE == 0U
  return 0;
#else
  float right_score = maze_simple_backup_right_score();
  float left_score = maze_simple_backup_left_score();
  uint8_t right_open_backup = (right_score > MAZE_SIMPLE_BACKUP_SIDE_OPEN_M) ? 1U : 0U;
  uint8_t left_open_backup = (left_score > MAZE_SIMPLE_BACKUP_SIDE_OPEN_M) ? 1U : 0U;

  if (maze_topo_pending_branch_dir > 0 && right_open_backup != 0U)
  {
    return 1;
  }
  if (maze_topo_pending_branch_dir < 0 && left_open_backup != 0U)
  {
    return -1;
  }

  if (right_open_backup != 0U && left_open_backup == 0U)
  {
    return 1;
  }
  if (left_open_backup != 0U && right_open_backup == 0U)
  {
    return -1;
  }
  if (right_open_backup != 0U && left_open_backup != 0U)
  {
    return (right_score >= (left_score - MAZE_SIMPLE_BACKUP_SIDE_MARGIN_M)) ? 1 : -1;
  }

  /* A dead end must remain a backup action. Do not manufacture a turn from
     small side-distance differences when neither side is a real opening. */
  return 0;
#endif
}

static uint8_t maze_simple_clear_or_unknown(uint32_t now_tick,
                                            float dist_m,
                                            uint32_t tick,
                                            float min_m)
{
  if (maze_simple_range_fresh(now_tick, tick) == 0U ||
      dist_m <= 0.0f)
  {
    return 1U;
  }

  return (dist_m >= min_m) ? 1U : 0U;
}

static uint8_t maze_simple_backup_turn_extra_clear(int8_t turn_dir,
                                                   uint32_t now_tick)
{
  if (turn_dir > 0)
  {
    if (maze_simple_clear_or_unknown(now_tick,
                                     leftdist,
                                     leftdist_last_valid_tick,
                                     MAZE_SIMPLE_BACKUP_TURN_OPPOSITE_SAFE_M) == 0U)
    {
      return 0U;
    }
    if (maze_simple_clear_or_unknown(now_tick,
                                     rightdist,
                                     rightdist_last_valid_tick,
                                     MAZE_SIMPLE_BACKUP_TURN_SAME_SAFE_M) == 0U)
    {
      return 0U;
    }
    if (maze_simple_clear_or_unknown(now_tick,
                                     rightfrontdist,
                                     rightfrontdist_last_valid_tick,
                                     MAZE_SIMPLE_BACKUP_TURN_FRONT_SAFE_M) == 0U)
    {
      return 0U;
    }
  }
  else if (turn_dir < 0)
  {
    if (maze_simple_clear_or_unknown(now_tick,
                                     rightdist,
                                     rightdist_last_valid_tick,
                                     MAZE_SIMPLE_BACKUP_TURN_OPPOSITE_SAFE_M) == 0U)
    {
      return 0U;
    }
    if (maze_simple_clear_or_unknown(now_tick,
                                     leftdist,
                                     leftdist_last_valid_tick,
                                     MAZE_SIMPLE_BACKUP_TURN_SAME_SAFE_M) == 0U)
    {
      return 0U;
    }
    if (maze_simple_clear_or_unknown(now_tick,
                                     leftfrontdist,
                                     leftfrontdist_last_valid_tick,
                                     MAZE_SIMPLE_BACKUP_TURN_FRONT_SAFE_M) == 0U)
    {
      return 0U;
    }
  }

  return 1U;
}

static float maze_simple_center_entry_target_m(void)
{
  return (maze_simple_prefer_turn_dir > 0) ?
         MAZE_SIMPLE_RIGHT_GAP_ENTRY_M :
         MAZE_SIMPLE_LEFT_GAP_ENTRY_M;
}

static float maze_simple_center_front_min_m(void)
{
  return (maze_simple_prefer_turn_dir > 0) ?
         MAZE_SIMPLE_RIGHT_CENTER_FRONT_MIN_M :
         MAZE_SIMPLE_LEFT_CENTER_FRONT_MIN_M;
}

static void maze_simple_enter(maze_simple_state_t state, uint32_t now_tick)
{
  if (state != MAZE_SIMPLE_CENTER_ENTRY)
  {
    maze_simple_center_turn_dir = 0;
    maze_simple_center_turn_topo_openings = 0U;
  }

  maze_simple_state = state;
  maze_simple_lock_reset();
  if (state != MAZE_SIMPLE_POST_TURN_LOCK)
  {
    maze_simple_post_lock_phase = 0U;
    maze_simple_post_lock_tick = 0U;
    maze_simple_post_lock_target_yaw_rad = 0.0f;
  }
  if (state != MAZE_SIMPLE_START_ALIGN)
  {
    maze_simple_start_align_phase = 0U;
    maze_simple_start_align_tick = 0U;
    maze_simple_start_align_target_yaw_rad = 0.0f;
    maze_simple_start_align_ref_valid = 0U;
  }
  if (state != MAZE_SIMPLE_YAW_RECOVER)
  {
    maze_simple_yaw_recover_phase = 0U;
    maze_simple_yaw_recover_tick = 0U;
  }
  if (state != MAZE_SIMPLE_TURN_BACK)
  {
    maze_simple_turn_back_step = 0U;
  }
  if (state != MAZE_SIMPLE_STOP_SCAN)
  {
    maze_simple_backup_align_phase = 0U;
    maze_simple_backup_align_tick = 0U;
  }
  if (state != MAZE_SIMPLE_BACKUP &&
      state != MAZE_SIMPLE_STOP_SCAN)
  {
    maze_simple_backup_ref_yaw_valid = 0U;
  }
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
  if (state == MAZE_SIMPLE_BACKUP ||
      state == MAZE_SIMPLE_TURN_BACK ||
      state == MAZE_SIMPLE_START_ALIGN)
  {
    MazeTopo_CancelPendingDecision();
  }
#endif
  if (state != MAZE_SIMPLE_STOP_SCAN &&
      state != MAZE_SIMPLE_POST_FORWARD)
  {
    maze_centerline_done_this_stop = 0U;
    maze_centerline_done_after_turn = 0U;
    maze_centerline_reset();
  }

  switch (state)
  {
    case MAZE_SIMPLE_START_ALIGN:
      maze_simple_turn_from_backup = 0U;
      maze_simple_prefer_turn_dir = 0;
      maze_simple_post_turn_brake_dir = 0;
      maze_simple_start_align_phase = 0U;
      maze_simple_start_align_tick = now_tick;
      maze_simple_start_align_target_yaw_rad = 0.0f;
      maze_simple_start_align_ref_valid = 0U;
#if ODOM_OGM_ENABLE
      if (maze_simple_start_align_preserve_map != 0U &&
          imu_yaw_usable_for_heading() != 0U)
      {
        maze_simple_start_align_ref_yaw_rad = deg_to_rad(yaw);
        maze_simple_start_align_ref_map_heading_rad = OGM_GetMapThetaRad();
        maze_simple_start_align_ref_valid = 1U;
      }
#endif
      maze_state = MAZE_STATE_PRETURN_STOP;
      maze_state_tick = now_tick;
#if ODOM_OGM_ENABLE
      maze_simple_start_valid = 0U;
      maze_simple_backup_start_valid = 0U;
      maze_simple_center_start_valid = 0U;
      maze_simple_pre_turn_start_valid = 0U;
#endif
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)"ALIGN:START\r\n", 13U, 80);
      break;

    case MAZE_SIMPLE_YAW_RECOVER:
      maze_simple_turn_from_backup = 0U;
      maze_simple_prefer_turn_dir = 0;
      maze_simple_post_turn_brake_dir = 0;
      maze_simple_yaw_recover_phase = 0U;
      maze_simple_yaw_recover_tick = now_tick;
      maze_state = MAZE_STATE_PRETURN_STOP;
      maze_state_tick = now_tick;
#if ODOM_OGM_ENABLE
      maze_simple_start_valid = 0U;
      maze_simple_backup_start_valid = 0U;
      maze_simple_center_start_valid = 0U;
      maze_simple_pre_turn_start_valid = 0U;
#endif
      break;

    case MAZE_SIMPLE_START_STRAIGHT:
      maze_simple_prefer_turn_dir = 0;
      if (maze_simple_cardinal_ref_valid == 0U &&
          imu_yaw_usable_for_heading() != 0U)
      {
        maze_simple_cardinal_ref_yaw_rad = deg_to_rad(yaw);
        maze_simple_cardinal_ref_valid = 1U;
      }
      maze_enter_state(MAZE_STATE_FOLLOW, now_tick);
#if ODOM_OGM_ENABLE
      maze_simple_start_x_m = odom_x_m;
      maze_simple_start_y_m = odom_y_m;
      maze_simple_start_valid = 1U;
      maze_simple_backup_start_valid = 0U;
      maze_simple_center_start_valid = 0U;
      maze_simple_pre_turn_start_valid = 0U;
#endif
      break;

    case MAZE_SIMPLE_APPROACH_JUNCTION:
      maze_enter_state(MAZE_STATE_FOLLOW, now_tick);
#if ODOM_OGM_ENABLE
      maze_simple_start_valid = 0U;
      maze_simple_backup_start_valid = 0U;
      maze_simple_center_start_valid = 0U;
      maze_simple_pre_turn_start_valid = 0U;
#endif
      break;

    case MAZE_SIMPLE_TURN_RIGHT:
      maze_simple_prefer_turn_dir = 0;
      maze_simple_post_turn_brake_dir = 0;
      maze_enter_state(MAZE_STATE_TURN_RIGHT, now_tick);
#if ODOM_OGM_ENABLE
      maze_simple_start_valid = 0U;
      maze_simple_backup_start_valid = 0U;
      maze_simple_center_start_valid = 0U;
      maze_simple_pre_turn_start_valid = 0U;
#endif
      break;

    case MAZE_SIMPLE_TURN_LEFT:
      maze_simple_prefer_turn_dir = 0;
      maze_simple_post_turn_brake_dir = 0;
      maze_enter_state(MAZE_STATE_TURN_LEFT, now_tick);
#if ODOM_OGM_ENABLE
      maze_simple_start_valid = 0U;
      maze_simple_backup_start_valid = 0U;
      maze_simple_center_start_valid = 0U;
      maze_simple_pre_turn_start_valid = 0U;
#endif
      break;

    case MAZE_SIMPLE_TURN_BACK:
      maze_simple_prefer_turn_dir = 0;
      maze_simple_post_turn_brake_dir = 0;
      if (maze_simple_turn_back_step == 0U)
      {
        maze_simple_turn_back_step = 1U;
      }
      maze_enter_state(MAZE_STATE_TURN_LEFT, now_tick);
#if ODOM_OGM_ENABLE
      maze_simple_start_valid = 0U;
      maze_simple_backup_start_valid = 0U;
      maze_simple_center_start_valid = 0U;
      maze_simple_pre_turn_start_valid = 0U;
#endif
      break;

    case MAZE_SIMPLE_POST_FORWARD:
      maze_simple_turn_from_backup = 0U;
      maze_simple_prefer_turn_dir = 0;
      maze_centerline_done_after_turn = 0U;
      maze_centerline_reset();
      maze_enter_state(MAZE_STATE_POST_TURN_FORWARD, now_tick);
#if ODOM_OGM_ENABLE
      maze_simple_start_valid = 0U;
      maze_simple_backup_start_valid = 0U;
      maze_simple_center_start_valid = 0U;
      maze_simple_pre_turn_start_valid = 0U;
#endif
      break;

    case MAZE_SIMPLE_POST_TURN_LOCK:
      maze_simple_turn_from_backup = 0U;
      maze_simple_prefer_turn_dir = 0;
      maze_state = MAZE_STATE_PRETURN_STOP;
      maze_state_tick = now_tick;
#if ODOM_OGM_ENABLE
      maze_simple_start_valid = 0U;
      maze_simple_backup_start_valid = 0U;
      maze_simple_center_start_valid = 0U;
      maze_simple_pre_turn_start_valid = 0U;
#endif
      break;

    case MAZE_SIMPLE_CENTER_ENTRY:
      maze_simple_turn_from_backup = 0U;
      maze_state = MAZE_STATE_PRETURN_STOP;
      maze_state_tick = now_tick;
#if ODOM_OGM_ENABLE
      maze_simple_start_valid = 0U;
      maze_simple_backup_start_valid = 0U;
      maze_simple_center_start_x_m = odom_x_m;
      maze_simple_center_start_y_m = odom_y_m;
      maze_simple_center_start_valid = 1U;
      maze_simple_pre_turn_start_valid = 0U;
#endif
      break;

    case MAZE_SIMPLE_PRE_TURN_FORWARD:
      maze_simple_turn_from_backup = 0U;
      maze_state = MAZE_STATE_PRETURN_STOP;
      maze_state_tick = now_tick;
#if ODOM_OGM_ENABLE
      maze_simple_start_valid = 0U;
      maze_simple_backup_start_valid = 0U;
      maze_simple_center_start_valid = 0U;
      maze_simple_pre_turn_start_x_m = odom_x_m;
      maze_simple_pre_turn_start_y_m = odom_y_m;
      maze_simple_pre_turn_start_valid = 1U;
#endif
      break;

    case MAZE_SIMPLE_STOP_SCAN:
      maze_simple_turn_from_backup = 0U;
      maze_centerline_done_this_stop = 0U;
      maze_centerline_reset();
      maze_state = MAZE_STATE_PRETURN_STOP;
      maze_state_tick = now_tick;
#if ODOM_OGM_ENABLE
      maze_simple_start_valid = 0U;
      maze_simple_backup_start_valid = 0U;
      maze_simple_center_start_valid = 0U;
      maze_simple_pre_turn_start_valid = 0U;
#endif
      break;

    case MAZE_SIMPLE_BACKUP:
      maze_simple_turn_from_backup = 0U;
      maze_simple_prefer_turn_dir = 0;
      if (imu_yaw_usable_for_heading() != 0U)
      {
        maze_simple_backup_ref_yaw_rad = deg_to_rad(yaw);
        maze_simple_backup_ref_yaw_valid = 1U;
      }
      else
      {
        maze_simple_backup_ref_yaw_valid = 0U;
      }
      maze_state = MAZE_STATE_FOLLOW;
      maze_state_tick = now_tick;
#if ODOM_OGM_ENABLE
      maze_simple_start_valid = 0U;
      maze_simple_backup_start_x_m = odom_x_m;
      maze_simple_backup_start_y_m = odom_y_m;
      maze_simple_backup_start_valid = 1U;
      maze_simple_center_start_valid = 0U;
      maze_simple_pre_turn_start_valid = 0U;
#endif
      break;

    case MAZE_SIMPLE_FOLLOW:
    default:
      maze_simple_turn_from_backup = 0U;
      maze_simple_prefer_turn_dir = 0;
      maze_simple_post_turn_brake_dir = 0;
      maze_enter_state(MAZE_STATE_FOLLOW, now_tick);
#if ODOM_OGM_ENABLE
      maze_simple_start_valid = 0U;
      maze_simple_backup_start_valid = 0U;
      maze_simple_center_start_valid = 0U;
      maze_simple_pre_turn_start_valid = 0U;
#endif
      break;
  }
}

static void maze_simple_schedule_turn(maze_simple_state_t turn_state, uint32_t now_tick)
{
  if (turn_state != MAZE_SIMPLE_TURN_RIGHT &&
      turn_state != MAZE_SIMPLE_TURN_LEFT)
  {
    maze_simple_enter(turn_state, now_tick);
    return;
  }

  maze_simple_pending_turn_state = turn_state;
  maze_simple_pending_turn_valid = 1U;
  maze_simple_enter(MAZE_SIMPLE_PRE_TURN_FORWARD, now_tick);
}

static float maze_simple_turn_target_rad(maze_simple_state_t state)
{
  if (state == MAZE_SIMPLE_TURN_BACK)
  {
    return MAZE_SIMPLE_TURN_BACK_STEP_RAD;
  }

  if (maze_simple_turn_from_backup != 0U)
  {
    return (state == MAZE_SIMPLE_TURN_RIGHT) ?
           MAZE_SIMPLE_BACKUP_TURN_RIGHT_RAD :
           MAZE_SIMPLE_BACKUP_TURN_LEFT_RAD;
  }

  return (state == MAZE_SIMPLE_TURN_RIGHT) ? MAZE_TURN_RIGHT_RAD : MAZE_TURN_LEFT_RAD;
}

static uint8_t maze_simple_turn_clear_for_state(maze_simple_state_t state, uint32_t now_tick)
{
#if MAZE_SIMPLE_TURN_CLEAR_GUARD_ENABLE == 0U
  (void)state;
  (void)now_tick;
  return 1U;
#else
  if (state == MAZE_SIMPLE_TURN_RIGHT)
  {
    if (maze_simple_range_fresh(now_tick, rightdist_last_valid_tick) != 0U &&
        rightdist > 0.0f &&
        rightdist <= MAZE_SIMPLE_TURN_SIDE_SAFE_M)
    {
      return 0U;
    }
    if (maze_simple_range_fresh(now_tick, rightfrontdist_last_valid_tick) != 0U &&
        rightfrontdist > 0.0f &&
        rightfrontdist <= MAZE_SIMPLE_TURN_FRONT_DIAG_SAFE_M)
    {
      return 0U;
    }
  }
  else if (state == MAZE_SIMPLE_TURN_LEFT)
  {
    if (maze_simple_range_fresh(now_tick, leftdist_last_valid_tick) != 0U &&
        leftdist > 0.0f &&
        leftdist <= MAZE_SIMPLE_TURN_SIDE_SAFE_M)
    {
      return 0U;
    }
    if (maze_simple_range_fresh(now_tick, leftfrontdist_last_valid_tick) != 0U &&
        leftfrontdist > 0.0f &&
        leftfrontdist <= MAZE_SIMPLE_TURN_FRONT_DIAG_SAFE_M)
    {
      return 0U;
    }
  }

  return 1U;
#endif
}

static uint8_t maze_simple_return_align_turn_active(void)
{
#if MAZE_TOPO_ENABLE
  return (maze_return_active != 0U &&
          maze_return_phase == MAZE_RETURN_PHASE_ALIGN_TO_PATH) ? 1U : 0U;
#else
  return 0U;
#endif
}

static uint8_t maze_simple_turn_signed_ok(maze_simple_state_t state,
                                          float signed_turn_rad,
                                          float target_rad)
{
  float min_dir_rad = target_rad * 0.25f;

  if (state == MAZE_SIMPLE_TURN_RIGHT)
  {
    return (signed_turn_rad <= -min_dir_rad) ? 1U : 0U;
  }

  return (signed_turn_rad >= min_dir_rad) ? 1U : 0U;
}

static uint8_t maze_simple_turn_finished(maze_simple_state_t state, uint32_t now_tick)
{
  uint32_t elapsed_ms = now_tick - maze_state_tick;
  maze_state_t turn_state = MAZE_STATE_TURN_LEFT;
  uint32_t turn_ms;
  uint32_t turn_max_ms;
  float target_rad = maze_simple_turn_target_rad(state);
  float signed_turn_rad = 0.0f;
  float turned_rad = 0.0f;
  uint8_t use_angle_gate = 0U;
  uint8_t return_align_turn = maze_simple_return_align_turn_active();

  maze_simple_turn_timeout_failed = 0U;
  maze_simple_turn_last_turned_rad = 0.0f;
  maze_simple_turn_last_signed_rad = 0.0f;
  maze_simple_turn_last_target_rad = target_rad;
  maze_simple_turn_last_yaw_gate = 0U;

  if (state == MAZE_SIMPLE_TURN_RIGHT)
  {
    turn_state = MAZE_STATE_TURN_RIGHT;
  }
  else if (state == MAZE_SIMPLE_TURN_BACK)
  {
    turn_state = MAZE_STATE_TURN_LEFT;
  }
  turn_ms = maze_turn_duration_ms(turn_state);
  turn_max_ms = turn_ms + MAZE_TURN_MAX_EXTRA_MS;

  if (elapsed_ms < MAZE_TURN_MIN_MS)
  {
    return 0U;
  }

#if ODOM_OGM_ENABLE
  if (maze_turn_start_valid != 0U)
  {
    uint8_t yaw_gate_valid = 0U;

    signed_turn_rad = wrap_pi(odom_theta_rad - maze_turn_start_theta_rad);
    turned_rad = fabsf(signed_turn_rad);
    use_angle_gate = 1U;

    if (maze_turn_start_yaw_valid != 0U &&
        imu_yaw_usable_for_heading() != 0U)
    {
      float yaw_now_rad = deg_to_rad(yaw);
      signed_turn_rad = wrap_pi(yaw_now_rad - maze_turn_start_yaw_rad);
      turned_rad = fabsf(signed_turn_rad);
      yaw_gate_valid = 1U;
      maze_simple_turn_last_yaw_gate = 1U;
    }
    maze_simple_turn_last_turned_rad = turned_rad;
    maze_simple_turn_last_signed_rad = signed_turn_rad;

    if (return_align_turn != 0U && yaw_gate_valid == 0U)
    {
      if (elapsed_ms >= turn_max_ms)
      {
        maze_simple_turn_timeout_failed = 1U;
        return 1U;
      }
      return 0U;
    }

    if (return_align_turn != 0U &&
        turned_rad >= target_rad &&
        maze_simple_turn_signed_ok(state, signed_turn_rad, target_rad) == 0U)
    {
      maze_simple_turn_timeout_failed = 1U;
      return 1U;
    }

    if (turned_rad >= target_rad)
    {
      return 1U;
    }
  }
#else
  (void)target_rad;
#endif

  if (turn_max_ms < turn_ms)
  {
    turn_max_ms = turn_ms;
  }

  if (use_angle_gate != 0U)
  {
    if (elapsed_ms >= turn_max_ms)
    {
      maze_simple_turn_timeout_failed = 1U;
      return 1U;
    }
    return 0U;
  }

  if (return_align_turn != 0U)
  {
    if (elapsed_ms >= turn_max_ms)
    {
      maze_simple_turn_timeout_failed = 1U;
      return 1U;
    }
    return 0U;
  }

  return (elapsed_ms >= turn_ms) ? 1U : 0U;
}

static void maze_simple_turn_drive(maze_simple_state_t state, uint32_t now_tick)
{
  float turn_cmd_pwm = MAZE_PWM_TURN;

  if (maze_simple_turn_from_backup != 0U)
  {
    turn_cmd_pwm = MAZE_SIMPLE_BACKUP_TURN_PWM;
  }

#if ODOM_OGM_ENABLE
  if (maze_turn_start_valid != 0U)
  {
    float target_rad = maze_simple_turn_target_rad(state);
    float slowdown_ratio = (maze_simple_turn_from_backup != 0U) ?
                           MAZE_SIMPLE_BACKUP_TURN_SLOWDOWN_RATIO :
                           MAZE_TURN_SLOWDOWN_RATIO;
    float final_pwm = (maze_simple_turn_from_backup != 0U) ?
                      MAZE_SIMPLE_BACKUP_TURN_PWM_FINAL :
                      MAZE_TURN_PWM_FINAL;
    float turned_rad = fabsf(wrap_pi(odom_theta_rad - maze_turn_start_theta_rad));

    if (maze_turn_start_yaw_valid != 0U &&
        imu_yaw_usable_for_heading() != 0U)
    {
      float yaw_now_rad = deg_to_rad(yaw);
      turned_rad = fabsf(wrap_pi(yaw_now_rad - maze_turn_start_yaw_rad));
    }

    if (target_rad > 0.2f &&
        turned_rad > (target_rad * slowdown_ratio))
    {
      turn_cmd_pwm = final_pwm;
    }
  }
#endif

  if (state == MAZE_SIMPLE_TURN_RIGHT)
  {
#if MAZE_RIGHT_MOTOR_REVERSED
    MazeDrive(-turn_cmd_pwm, turn_cmd_pwm, now_tick);
#else
    MazeDrive(turn_cmd_pwm, -turn_cmd_pwm, now_tick);
#endif
  }
  else
  {
    /* Turn-back uses the same in-place left rotation path for repeatability. */
#if MAZE_RIGHT_MOTOR_REVERSED
    MazeDrive(turn_cmd_pwm, -turn_cmd_pwm, now_tick);
#else
    MazeDrive(-turn_cmd_pwm, turn_cmd_pwm, now_tick);
#endif
  }
}

static void maze_simple_drive_straight(uint32_t now_tick)
{
  float left_cmd = MAZE_PWM_BASE;
  float right_cmd = MAZE_PWM_BASE;
#if MAZE_SIMPLE_AXIS_DRIVE_ENABLE
  uint8_t front_slow = 0U;
  uint8_t side_bias_enable =
    (maze_simple_state == MAZE_SIMPLE_FOLLOW ||
     maze_simple_state == MAZE_SIMPLE_START_STRAIGHT) ? 1U : 0U;
#endif
#if MAZE_SIMPLE_AXIS_DRIVE_ENABLE
  float right_clear = rightdist;
  float left_clear = leftdist;
#endif

  if (maze_simple_state == MAZE_SIMPLE_CENTER_ENTRY &&
      fordist > 0.0f &&
      fordist < MAZE_SIMPLE_CENTER_SLOW_FRONT_M)
  {
#if MAZE_SIMPLE_AXIS_DRIVE_ENABLE
    front_slow = 1U;
#endif
    left_cmd = MAZE_SIMPLE_FORWARD_SLOW_PWM;
    right_cmd = MAZE_SIMPLE_FORWARD_SLOW_PWM;
  }

#if MAZE_SIMPLE_AXIS_DRIVE_ENABLE
  if (rightfrontdist > 0.0f && (right_clear <= 0.0f || rightfrontdist < right_clear))
  {
    right_clear = rightfrontdist;
  }
  if (leftfrontdist > 0.0f && (left_clear <= 0.0f || leftfrontdist < left_clear))
  {
    left_clear = leftfrontdist;
  }

  if (front_slow == 0U &&
      right_clear > 0.0f && right_clear < MAZE_SIMPLE_FORWARD_SIDE_PANIC_M)
  {
    left_cmd = MAZE_SIMPLE_FORWARD_SLOW_PWM + MAZE_SIMPLE_FORWARD_STEER_PWM;
    right_cmd = MAZE_SIMPLE_FORWARD_SLOW_PWM;
  }
  else if (front_slow == 0U &&
           left_clear > 0.0f && left_clear < MAZE_SIMPLE_FORWARD_SIDE_PANIC_M)
  {
    left_cmd = MAZE_SIMPLE_FORWARD_SLOW_PWM;
    right_cmd = MAZE_SIMPLE_FORWARD_SLOW_PWM + MAZE_SIMPLE_FORWARD_STEER_PWM;
  }
  else if (front_slow == 0U &&
           side_bias_enable != 0U &&
           right_clear > 0.0f && right_clear < MAZE_SIMPLE_FORWARD_SIDE_DANGER_M)
  {
    left_cmd += MAZE_SIMPLE_FORWARD_STEER_PWM;
    right_cmd -= MAZE_SIMPLE_FORWARD_STEER_PWM;
  }
  else if (front_slow == 0U &&
           side_bias_enable != 0U &&
           left_clear > 0.0f && left_clear < MAZE_SIMPLE_FORWARD_SIDE_DANGER_M)
  {
    left_cmd -= MAZE_SIMPLE_FORWARD_STEER_PWM;
    right_cmd += MAZE_SIMPLE_FORWARD_STEER_PWM;
  }
  else if (front_slow == 0U &&
           side_bias_enable != 0U &&
           right_clear > 0.0f &&
           left_clear > 0.0f)
  {
#if MAZE_SIMPLE_FORWARD_CENTER_ENABLE
    float corridor_width = left_clear + right_clear;

    if (corridor_width >= MAZE_SIMPLE_FORWARD_CENTER_MIN_WIDTH_M &&
        corridor_width <= MAZE_SIMPLE_FORWARD_CENTER_MAX_WIDTH_M)
    {
      float center_err_m = (left_clear - right_clear) * 0.5f;

      if (fabsf(center_err_m) > MAZE_SIMPLE_FORWARD_CENTER_DEADBAND_M)
      {
        float center_corr = clampf(center_err_m *
                                   MAZE_SIMPLE_FORWARD_CENTER_KP_PWM_PER_M,
                                   -MAZE_SIMPLE_FORWARD_CENTER_CORR_MAX_PWM,
                                   MAZE_SIMPLE_FORWARD_CENTER_CORR_MAX_PWM);
        left_cmd += center_corr;
        right_cmd -= center_corr;
      }
    }
#endif
  }
#endif

  MazeDrive(left_cmd, right_cmd, now_tick);
}

static void maze_simple_drive_post_turn(uint32_t now_tick)
{
  MazeDrive(MAZE_SIMPLE_POST_SLOW_PWM, MAZE_SIMPLE_POST_SLOW_PWM, now_tick);
}

static void maze_simple_drive_turn_brake(int8_t last_turn_dir, uint32_t now_tick)
{
  uint32_t brake_elapsed_ms = now_tick - maze_state_tick;

  if (brake_elapsed_ms < MAZE_SIMPLE_POST_TURN_COUNTER_BRAKE_MS)
  {
    if (last_turn_dir > 0)
    {
#if MAZE_RIGHT_MOTOR_REVERSED
      MazeDrive(MAZE_SIMPLE_POST_TURN_COUNTER_BRAKE_PWM,
                -MAZE_SIMPLE_POST_TURN_COUNTER_BRAKE_PWM,
                now_tick);
#else
      MazeDrive(-MAZE_SIMPLE_POST_TURN_COUNTER_BRAKE_PWM,
                MAZE_SIMPLE_POST_TURN_COUNTER_BRAKE_PWM,
                now_tick);
#endif
    }
    else if (last_turn_dir < 0)
    {
#if MAZE_RIGHT_MOTOR_REVERSED
      MazeDrive(-MAZE_SIMPLE_POST_TURN_COUNTER_BRAKE_PWM,
                MAZE_SIMPLE_POST_TURN_COUNTER_BRAKE_PWM,
                now_tick);
#else
      MazeDrive(MAZE_SIMPLE_POST_TURN_COUNTER_BRAKE_PWM,
                -MAZE_SIMPLE_POST_TURN_COUNTER_BRAKE_PWM,
                now_tick);
#endif
    }
    else
    {
      MazeBrake(MAZE_SIMPLE_POST_TURN_BRAKE_PWM, now_tick);
    }
    return;
  }

  MazeBrake(MAZE_SIMPLE_POST_TURN_BRAKE_PWM, now_tick);
}

static void maze_simple_drive_stop_brake(uint32_t now_tick)
{
  MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
}

static void maze_simple_drive_backup(uint32_t now_tick)
{
  float base_pwm = MAZE_SIMPLE_BACKUP_PWM;
  float left_cmd;
  float right_cmd;
  float wall_nudge_pwm = 0.0f;
  float right_clear = rightdist;
  float left_clear = leftdist;

  if (rightfrontdist > 0.0f && (right_clear <= 0.0f || rightfrontdist < right_clear))
  {
    right_clear = rightfrontdist;
  }
  if (rightreardist > 0.0f && (right_clear <= 0.0f || rightreardist < right_clear))
  {
    right_clear = rightreardist;
  }
  if (leftfrontdist > 0.0f && (left_clear <= 0.0f || leftfrontdist < left_clear))
  {
    left_clear = leftfrontdist;
  }
  if (leftreardist > 0.0f && (left_clear <= 0.0f || leftreardist < left_clear))
  {
    left_clear = leftreardist;
  }

  if ((right_clear > 0.0f && right_clear < MAZE_SIMPLE_BACKUP_SIDE_PANIC_M) ||
      (left_clear > 0.0f && left_clear < MAZE_SIMPLE_BACKUP_SIDE_PANIC_M))
  {
    base_pwm = MAZE_SIMPLE_BACKUP_SLOW_PWM;
  }

  if (right_clear > 0.0f &&
      right_clear < MAZE_SIMPLE_BACKUP_WALL_NUDGE_M &&
      (left_clear <= 0.0f || right_clear <= left_clear))
  {
    wall_nudge_pwm = (right_clear < MAZE_SIMPLE_BACKUP_WALL_PANIC_M) ?
                     MAZE_SIMPLE_BACKUP_WALL_PANIC_PWM :
                     MAZE_SIMPLE_BACKUP_WALL_NUDGE_PWM;
  }
  else if (left_clear > 0.0f &&
           left_clear < MAZE_SIMPLE_BACKUP_WALL_NUDGE_M)
  {
    wall_nudge_pwm = (left_clear < MAZE_SIMPLE_BACKUP_WALL_PANIC_M) ?
                     -MAZE_SIMPLE_BACKUP_WALL_PANIC_PWM :
                     -MAZE_SIMPLE_BACKUP_WALL_NUDGE_PWM;
  }

  left_cmd = -base_pwm - MAZE_SIMPLE_BACKUP_TRIM_PWM;
  right_cmd = -base_pwm + MAZE_SIMPLE_BACKUP_TRIM_PWM;
  if (wall_nudge_pwm != 0.0f)
  {
    left_cmd -= wall_nudge_pwm;
    right_cmd += wall_nudge_pwm;
  }

  MazeDrive(left_cmd, right_cmd, now_tick);
}

static void maze_centerline_reset(void)
{
  maze_centerline_phase = MAZE_CENTERLINE_IDLE;
  maze_centerline_dir = 0;
  maze_centerline_phase_tick = 0U;
  maze_centerline_error_m = 0.0f;
  maze_centerline_shift_target_m = 0.0f;
  maze_centerline_dbg_yaw_err_deg = 0.0f;
}

static uint8_t maze_centerline_side_pair_parallel(uint32_t now_tick,
                                                  float front_side_m,
                                                  float rear_side_m,
                                                  uint32_t front_tick,
                                                  uint32_t rear_tick)
{
  if (maze_simple_range_fresh(now_tick, front_tick) == 0U ||
      maze_simple_range_fresh(now_tick, rear_tick) == 0U)
  {
    return 1U;
  }

  if (front_side_m <= 0.0f || rear_side_m <= 0.0f ||
      front_side_m > MAZE_CENTERLINE_SIDE_MAX_M ||
      rear_side_m > MAZE_CENTERLINE_SIDE_MAX_M)
  {
    return 1U;
  }

  return (fabsf(front_side_m - rear_side_m) <= MAZE_CENTERLINE_PARALLEL_MAX_M) ? 1U : 0U;
}

static uint8_t maze_centerline_prepare(uint32_t now_tick)
{
#if MAZE_CENTERLINE_ENABLE
  float width_m;
  float abs_error_m;
  float angle_rad;
  float shift_m;

  if (imu_yaw_usable_for_heading() == 0U)
  {
    return 0U;
  }

  if (maze_simple_range_fresh(now_tick, leftdist_last_valid_tick) == 0U ||
      maze_simple_range_fresh(now_tick, rightdist_last_valid_tick) == 0U)
  {
    return 0U;
  }

  if (leftdist < MAZE_CENTERLINE_SIDE_MIN_M ||
      rightdist < MAZE_CENTERLINE_SIDE_MIN_M ||
      leftdist > MAZE_CENTERLINE_SIDE_MAX_M ||
      rightdist > MAZE_CENTERLINE_SIDE_MAX_M)
  {
    return 0U;
  }

  width_m = leftdist + rightdist;
  if (width_m < MAZE_CENTERLINE_WIDTH_MIN_M ||
      width_m > MAZE_CENTERLINE_WIDTH_MAX_M)
  {
    return 0U;
  }

  if (maze_simple_range_fresh(now_tick, fordist_last_valid_tick) != 0U &&
      fordist > 0.0f &&
      fordist < MAZE_CENTERLINE_FRONT_ROOM_M)
  {
    return 0U;
  }

  if (maze_centerline_side_pair_parallel(now_tick,
                                         rightfrontdist,
                                         rightreardist,
                                         rightfrontdist_last_valid_tick,
                                         rightreardist_last_valid_tick) == 0U ||
      maze_centerline_side_pair_parallel(now_tick,
                                         leftfrontdist,
                                         leftreardist,
                                         leftfrontdist_last_valid_tick,
                                         leftreardist_last_valid_tick) == 0U)
  {
    return 0U;
  }

  maze_centerline_error_m = (leftdist - rightdist) * 0.5f;
  abs_error_m = fabsf(maze_centerline_error_m);
  if (abs_error_m < MAZE_CENTERLINE_DEADBAND_M)
  {
    return 0U;
  }
  if (abs_error_m > MAZE_CENTERLINE_MAX_ERROR_M)
  {
    abs_error_m = MAZE_CENTERLINE_MAX_ERROR_M;
  }

  angle_rad = MAZE_CENTERLINE_SHIFT_ANGLE_DEG * (PI_F / 180.0f);
  shift_m = (sinf(angle_rad) > 0.01f) ?
            ((abs_error_m * MAZE_CENTERLINE_SHIFT_GAIN) / sinf(angle_rad)) :
            MAZE_CENTERLINE_SHIFT_MAX_M;
  maze_centerline_shift_target_m = clampf(shift_m,
                                          MAZE_CENTERLINE_SHIFT_MIN_M,
                                          MAZE_CENTERLINE_SHIFT_MAX_M);
  maze_centerline_ref_yaw_rad = deg_to_rad(yaw);
  maze_centerline_dir = (maze_centerline_error_m > 0.0f) ? 1 : -1;
  return 1U;
#else
  (void)now_tick;
  return 0U;
#endif
}

static void maze_centerline_drive_yaw_turn(float yaw_err_deg, float turn_pwm, uint32_t now_tick)
{
  if (yaw_err_deg > 0.0f)
  {
#if MAZE_RIGHT_MOTOR_REVERSED
    MazeDrive(turn_pwm, -turn_pwm, now_tick);
#else
    MazeDrive(-turn_pwm, turn_pwm, now_tick);
#endif
  }
  else
  {
#if MAZE_RIGHT_MOTOR_REVERSED
    MazeDrive(-turn_pwm, turn_pwm, now_tick);
#else
    MazeDrive(turn_pwm, -turn_pwm, now_tick);
#endif
  }
}

static uint8_t maze_centerline_turn_to_yaw(float target_yaw_rad, uint32_t now_tick)
{
  float yaw_now_rad = deg_to_rad(yaw);
  float yaw_err_deg = wrap_pi(target_yaw_rad - yaw_now_rad) * (180.0f / PI_F);
  float abs_err_deg = fabsf(yaw_err_deg);
  float turn_pwm;

  maze_centerline_dbg_yaw_err_deg = yaw_err_deg;
  if (abs_err_deg <= MAZE_CENTERLINE_TURN_DEADBAND_DEG ||
      (now_tick - maze_centerline_phase_tick) >= MAZE_CENTERLINE_TURN_MAX_MS)
  {
    MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
    return 1U;
  }

  turn_pwm = clampf(abs_err_deg * MAZE_CENTERLINE_TURN_KP_PWM_PER_DEG,
                    MAZE_CENTERLINE_TURN_PWM_MIN,
                    MAZE_CENTERLINE_TURN_PWM_MAX);
  maze_centerline_drive_yaw_turn(yaw_err_deg, turn_pwm, now_tick);
  return 0U;
}

static uint8_t maze_centerline_step(uint32_t now_tick)
{
#if MAZE_CENTERLINE_ENABLE
  switch (maze_centerline_phase)
  {
    case MAZE_CENTERLINE_IDLE:
      if (maze_centerline_prepare(now_tick) == 0U)
      {
        return 1U;
      }

      maze_centerline_phase = MAZE_CENTERLINE_BRAKE;
      maze_centerline_phase_tick = now_tick;
      MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
      return 0U;

    case MAZE_CENTERLINE_BRAKE:
      if ((now_tick - maze_centerline_phase_tick) < MAZE_CENTERLINE_BRAKE_MS)
      {
        MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
        return 0U;
      }

      maze_centerline_target_yaw_rad = wrap_pi(maze_centerline_ref_yaw_rad +
                                               ((maze_centerline_dir > 0) ? 1.0f : -1.0f) *
                                               (MAZE_CENTERLINE_SHIFT_ANGLE_DEG * (PI_F / 180.0f)));
      maze_centerline_phase = MAZE_CENTERLINE_TURN_OUT;
      maze_centerline_phase_tick = now_tick;
      return 0U;

    case MAZE_CENTERLINE_TURN_OUT:
      if (maze_centerline_turn_to_yaw(maze_centerline_target_yaw_rad, now_tick) == 0U)
      {
        return 0U;
      }

      maze_centerline_start_x_m = odom_x_m;
      maze_centerline_start_y_m = odom_y_m;
      maze_centerline_phase = MAZE_CENTERLINE_SHIFT;
      maze_centerline_phase_tick = now_tick;
      return 0U;

    case MAZE_CENTERLINE_SHIFT:
    {
      float dx = odom_x_m - maze_centerline_start_x_m;
      float dy = odom_y_m - maze_centerline_start_y_m;
      float ds_m = sqrtf((dx * dx) + (dy * dy));

      if (ds_m >= maze_centerline_shift_target_m ||
          (now_tick - maze_centerline_phase_tick) >= MAZE_CENTERLINE_SHIFT_MAX_MS ||
          (maze_simple_range_fresh(now_tick, fordist_last_valid_tick) != 0U &&
           fordist > 0.0f &&
           fordist < MAZE_SIMPLE_APPROACH_FRONT_MIN_M))
      {
        maze_centerline_phase = MAZE_CENTERLINE_TURN_BACK;
        maze_centerline_phase_tick = now_tick;
        return 0U;
      }

      MazeDrive(MAZE_CENTERLINE_SHIFT_PWM, MAZE_CENTERLINE_SHIFT_PWM, now_tick);
      return 0U;
    }

    case MAZE_CENTERLINE_TURN_BACK:
      if (maze_centerline_turn_to_yaw(maze_centerline_ref_yaw_rad, now_tick) == 0U)
      {
        return 0U;
      }

      maze_centerline_phase = MAZE_CENTERLINE_SETTLE;
      maze_centerline_phase_tick = now_tick;
      return 0U;

    case MAZE_CENTERLINE_SETTLE:
      if ((now_tick - maze_centerline_phase_tick) < MAZE_CENTERLINE_SETTLE_MS)
      {
        MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
        return 0U;
      }

      maze_centerline_reset();
      return 1U;

    default:
      maze_centerline_reset();
      return 1U;
  }
#else
  (void)now_tick;
  return 1U;
#endif
}

static uint8_t maze_simple_backup_align_before_turn(uint32_t now_tick)
{
#if MAZE_SIMPLE_BACKUP_ALIGN_ENABLE
  float yaw_now_rad;
  float yaw_err_rad;
  float yaw_err_deg;
  float abs_err_deg;
  float turn_pwm;

  if (maze_simple_backup_ref_yaw_valid == 0U ||
      imu_yaw_usable_for_heading() == 0U)
  {
    maze_simple_backup_align_phase = 0U;
    selfdbg_backup_align_phase = 0U;
    selfdbg_backup_align_age_ms = 0U;
    selfdbg_backup_align_err_deg = 0.0f;
    selfdbg_backup_align_ref_deg = 0.0f;
    selfdbg_backup_align_pwm = 0.0f;
    return 1U;
  }

  yaw_now_rad = deg_to_rad(yaw);
  yaw_err_rad = wrap_pi(maze_simple_backup_ref_yaw_rad - yaw_now_rad);
  yaw_err_deg = yaw_err_rad * (180.0f / PI_F);
  abs_err_deg = fabsf(yaw_err_deg);
  selfdbg_backup_align_err_deg = yaw_err_deg;
  selfdbg_backup_align_ref_deg = maze_simple_backup_ref_yaw_rad * (180.0f / PI_F);
  selfdbg_backup_align_pwm = 0.0f;

  if (maze_simple_backup_align_phase == 0U)
  {
    if (abs_err_deg <= MAZE_SIMPLE_BACKUP_ALIGN_DEADBAND_DEG)
    {
      selfdbg_backup_align_phase = 0U;
      selfdbg_backup_align_age_ms = 0U;
      return 1U;
    }
    maze_simple_backup_align_phase = 1U;
    maze_simple_backup_align_tick = now_tick;
  }

  if (maze_simple_backup_align_phase == 2U)
  {
    selfdbg_backup_align_phase = maze_simple_backup_align_phase;
    selfdbg_backup_align_age_ms = now_tick - maze_simple_backup_align_tick;
    if ((now_tick - maze_simple_backup_align_tick) < MAZE_SIMPLE_BACKUP_ALIGN_BRAKE_MS)
    {
      MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
      return 0U;
    }

    maze_simple_backup_align_phase = 0U;
    selfdbg_backup_align_phase = 0U;
    selfdbg_backup_align_age_ms = 0U;
    return 1U;
  }

  if (abs_err_deg <= MAZE_SIMPLE_BACKUP_ALIGN_DEADBAND_DEG ||
      (now_tick - maze_simple_backup_align_tick) >= MAZE_SIMPLE_BACKUP_ALIGN_MAX_MS)
  {
    maze_simple_backup_align_phase = 2U;
    maze_simple_backup_align_tick = now_tick;
    selfdbg_backup_align_phase = maze_simple_backup_align_phase;
    selfdbg_backup_align_age_ms = 0U;
    MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
    return 0U;
  }

  turn_pwm = clampf(abs_err_deg * MAZE_SIMPLE_BACKUP_ALIGN_KP_PWM_PER_DEG,
                    MAZE_SIMPLE_BACKUP_ALIGN_PWM_MIN,
                    MAZE_SIMPLE_BACKUP_ALIGN_PWM_MAX);
  selfdbg_backup_align_phase = maze_simple_backup_align_phase;
  selfdbg_backup_align_age_ms = now_tick - maze_simple_backup_align_tick;
  selfdbg_backup_align_pwm = turn_pwm;

  if (yaw_err_deg > 0.0f)
  {
#if MAZE_RIGHT_MOTOR_REVERSED
    MazeDrive(turn_pwm, -turn_pwm, now_tick);
#else
    MazeDrive(-turn_pwm, turn_pwm, now_tick);
#endif
  }
  else
  {
#if MAZE_RIGHT_MOTOR_REVERSED
    MazeDrive(-turn_pwm, turn_pwm, now_tick);
#else
    MazeDrive(turn_pwm, -turn_pwm, now_tick);
#endif
  }

  return 0U;
#else
  (void)now_tick;
  return 1U;
#endif
}

static void maze_simple_lock_reset(void)
{
  maze_simple_lock_phase = 0U;
  maze_simple_lock_tick = 0U;
  maze_simple_lock_sample_tick = 0U;
  maze_simple_lock_stable_count = 0U;
  maze_simple_lock_last_signature = 0xFFFFU;
  maze_simple_lock_target_yaw_rad = 0.0f;
  maze_simple_lock_dbg_yaw_err_deg = 0.0f;
}

static float maze_simple_nearest_cardinal_yaw_rad(float yaw_rad)
{
  const float quadrant_rad = PI_F * 0.5f;
  float ref_rad = (maze_simple_cardinal_ref_valid != 0U) ?
                  maze_simple_cardinal_ref_yaw_rad :
                  0.0f;
  float rel_rad = wrap_pi(yaw_rad - ref_rad);
  float q = rel_rad / quadrant_rad;
  int q_round = (q >= 0.0f) ? (int)(q + 0.5f) : (int)(q - 0.5f);
  return wrap_pi(ref_rad + ((float)q_round * quadrant_rad));
}

static uint16_t maze_simple_lock_decision_signature(uint32_t now_tick)
{
  uint16_t sig = 0U;

  if (maze_simple_front_blocked(fordist) != 0U) { sig |= (uint16_t)(1U << 0); }
  if (maze_simple_front_clear(fordist) != 0U)   { sig |= (uint16_t)(1U << 1); }
  if (maze_simple_right_open() != 0U)           { sig |= (uint16_t)(1U << 2); }
  if (maze_simple_left_open() != 0U)            { sig |= (uint16_t)(1U << 3); }
  if (maze_simple_right_probe_candidate(now_tick, fordist) != 0U)
  {
    sig |= (uint16_t)(1U << 4);
  }
  if (maze_simple_backup_right_open() != 0U)    { sig |= (uint16_t)(1U << 5); }
  if (maze_simple_backup_left_open() != 0U)     { sig |= (uint16_t)(1U << 6); }
  if (maze_simple_side_pair_ready(now_tick,
                                  rightdist_last_valid_tick,
                                  rightfrontdist_last_valid_tick,
                                  rightreardist_last_valid_tick) != 0U)
  {
    sig |= (uint16_t)(1U << 7);
  }
  if (maze_simple_side_pair_ready(now_tick,
                                  leftdist_last_valid_tick,
                                  leftfrontdist_last_valid_tick,
                                  leftreardist_last_valid_tick) != 0U)
  {
    sig |= (uint16_t)(1U << 8);
  }

  return sig;
}

static uint8_t maze_simple_manhattan_lock_step(uint32_t now_tick)
{
#if MAZE_SIMPLE_LOCK_ENABLE
  uint16_t sig;

  if ((now_tick - maze_state_tick) < MAZE_SIMPLE_STOP_SCAN_MS)
  {
    MazeDrive(0.0f, 0.0f, now_tick);
    return 0U;
  }

  if (maze_simple_lock_phase == 0U)
  {
    maze_simple_lock_phase = 1U;
    maze_simple_lock_tick = now_tick;
    maze_simple_lock_sample_tick = 0U;
    maze_simple_lock_stable_count = 0U;
    maze_simple_lock_last_signature = 0xFFFFU;
    maze_simple_lock_dbg_yaw_err_deg = 0.0f;

    if (imu_yaw_usable_for_heading() != 0U)
    {
      maze_simple_lock_target_yaw_rad =
        maze_simple_nearest_cardinal_yaw_rad(deg_to_rad(yaw));
    }
    else
    {
      maze_simple_lock_phase = 3U;
      maze_simple_lock_tick = now_tick;
    }
  }

  if (maze_simple_lock_phase == 1U)
  {
    float yaw_err_deg;
    float abs_err_deg;
    float turn_pwm;

    if (imu_yaw_usable_for_heading() == 0U)
    {
      maze_simple_lock_phase = 3U;
      maze_simple_lock_tick = now_tick;
      MazeDrive(0.0f, 0.0f, now_tick);
      return 0U;
    }

    yaw_err_deg = wrap_pi(maze_simple_lock_target_yaw_rad - deg_to_rad(yaw)) *
                  (180.0f / PI_F);
    abs_err_deg = fabsf(yaw_err_deg);
    maze_simple_lock_dbg_yaw_err_deg = yaw_err_deg;

    if (abs_err_deg <= MAZE_SIMPLE_LOCK_YAW_DEADBAND_DEG ||
        (now_tick - maze_simple_lock_tick) >= MAZE_SIMPLE_LOCK_YAW_MAX_MS)
    {
      maze_simple_backup_ref_yaw_rad = maze_simple_lock_target_yaw_rad;
      maze_simple_backup_ref_yaw_valid = 1U;
      maze_simple_lock_phase = 2U;
      maze_simple_lock_tick = now_tick;
      MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
      return 0U;
    }

    turn_pwm = clampf(abs_err_deg * MAZE_SIMPLE_LOCK_YAW_KP_PWM_PER_DEG,
                      MAZE_SIMPLE_LOCK_YAW_PWM_MIN,
                      MAZE_SIMPLE_LOCK_YAW_PWM_MAX);
    maze_centerline_drive_yaw_turn(yaw_err_deg, turn_pwm, now_tick);
    return 0U;
  }

  if (maze_simple_lock_phase == 2U)
  {
    if ((now_tick - maze_simple_lock_tick) < MAZE_SIMPLE_LOCK_SETTLE_MS)
    {
      MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
      return 0U;
    }

    maze_simple_lock_phase = 3U;
    maze_simple_lock_tick = now_tick;
    maze_simple_lock_sample_tick = 0U;
    maze_simple_lock_stable_count = 0U;
    maze_simple_lock_last_signature = 0xFFFFU;
  }

  if ((now_tick - maze_state_tick) >= MAZE_SIMPLE_LOCK_DECISION_MAX_MS)
  {
    maze_simple_lock_phase = 0U;
    return 1U;
  }

  if (maze_simple_decision_cardinal_ready(now_tick, maze_state_tick) == 0U)
  {
    MazeDrive(0.0f, 0.0f, now_tick);
    return 0U;
  }

  if (maze_simple_lock_sample_tick != 0U &&
      (now_tick - maze_simple_lock_sample_tick) < MAZE_SIMPLE_LOCK_FRAME_MS)
  {
    MazeDrive(0.0f, 0.0f, now_tick);
    return 0U;
  }

  sig = maze_simple_lock_decision_signature(now_tick);
  if (sig == maze_simple_lock_last_signature)
  {
    if (maze_simple_lock_stable_count < 255U)
    {
      maze_simple_lock_stable_count++;
    }
  }
  else
  {
    maze_simple_lock_last_signature = sig;
    maze_simple_lock_stable_count = 1U;
  }
  maze_simple_lock_sample_tick = now_tick;

  if (maze_simple_lock_stable_count >= MAZE_SIMPLE_LOCK_STABLE_FRAMES)
  {
    return 1U;
  }

  MazeDrive(0.0f, 0.0f, now_tick);
  return 0U;
#else
  (void)now_tick;
  return 1U;
#endif
}

static uint8_t maze_simple_post_turn_lock_step(uint32_t now_tick)
{
#if MAZE_SIMPLE_POST_LOCK_ENABLE
  if (maze_simple_post_lock_phase == 0U)
  {
    maze_simple_post_lock_phase = 1U;
    maze_simple_post_lock_tick = now_tick;

    if (imu_yaw_usable_for_heading() != 0U)
    {
      maze_simple_post_lock_target_yaw_rad =
        maze_simple_nearest_cardinal_yaw_rad(deg_to_rad(yaw));
    }
    else
    {
      maze_simple_post_lock_phase = 3U;
      maze_simple_post_lock_tick = now_tick;
    }
  }

  if (maze_simple_post_lock_phase == 1U)
  {
    if ((now_tick - maze_simple_post_lock_tick) < MAZE_SIMPLE_POST_LOCK_BRAKE_MS)
    {
      MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
      return 0U;
    }

    if (imu_yaw_usable_for_heading() == 0U)
    {
      maze_simple_post_lock_phase = 3U;
      maze_simple_post_lock_tick = now_tick;
      MazeDrive(0.0f, 0.0f, now_tick);
      return 0U;
    }

    maze_simple_post_lock_phase = 2U;
    maze_simple_post_lock_tick = now_tick;
  }

  if (maze_simple_post_lock_phase == 2U)
  {
    float yaw_err_deg;
    float abs_err_deg;
    float turn_pwm;

    if (imu_yaw_usable_for_heading() == 0U)
    {
      maze_simple_post_lock_phase = 3U;
      maze_simple_post_lock_tick = now_tick;
      MazeDrive(0.0f, 0.0f, now_tick);
      return 0U;
    }

    yaw_err_deg = wrap_pi(maze_simple_post_lock_target_yaw_rad - deg_to_rad(yaw)) *
                  (180.0f / PI_F);
    abs_err_deg = fabsf(yaw_err_deg);

    if (abs_err_deg <= MAZE_SIMPLE_POST_LOCK_YAW_DEADBAND_DEG ||
        (now_tick - maze_simple_post_lock_tick) >= MAZE_SIMPLE_POST_LOCK_YAW_MAX_MS)
    {
      maze_simple_backup_ref_yaw_rad = maze_simple_post_lock_target_yaw_rad;
      maze_simple_backup_ref_yaw_valid = 1U;
      maze_simple_post_lock_phase = 3U;
      maze_simple_post_lock_tick = now_tick;
      MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
      return 0U;
    }

    turn_pwm = clampf(abs_err_deg * MAZE_SIMPLE_POST_LOCK_KP_PWM_PER_DEG,
                      MAZE_SIMPLE_POST_LOCK_PWM_MIN,
                      MAZE_SIMPLE_POST_LOCK_PWM_MAX);
    maze_centerline_drive_yaw_turn(yaw_err_deg, turn_pwm, now_tick);
    return 0U;
  }

  if (maze_simple_post_lock_phase == 3U)
  {
    if ((now_tick - maze_simple_post_lock_tick) < MAZE_SIMPLE_POST_LOCK_SETTLE_MS)
    {
      MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
      return 0U;
    }

    maze_simple_post_lock_phase = 0U;
    return 1U;
  }

  return 1U;
#else
  (void)now_tick;
  return 1U;
#endif
}

static uint8_t maze_simple_start_align_fit_side(int8_t side_sign,
                                                uint16_t count,
                                                float *angle_rad,
                                                uint16_t *points)
{
#if ODOM_OGM_ENABLE
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_xx = 0.0f;
  float sum_xy = 0.0f;
  uint16_t n = 0U;
  float den;
  float slope;
  float angle;

  if (angle_rad == NULL || points == NULL || side_sign == 0)
  {
    return 0U;
  }

  for (uint16_t i = 0U; i < count; i++)
  {
    float dist_m;
    float lidar_angle_deg;
    float rel_angle_rad;
    float x_m;
    float y_m;

    if (ogm_scan_filtered_bins_mm[i] == 0U)
    {
      continue;
    }

    dist_m = (float)ogm_scan_filtered_bins_mm[i] / 1000.0f;
    if (dist_m < MAZE_SIMPLE_START_ALIGN_MIN_SIDE_M ||
        dist_m > MAZE_SIMPLE_START_ALIGN_MAX_SIDE_M)
    {
      continue;
    }

    lidar_angle_deg = ((((float)i) + 0.5f) * OGM_SCAN_BIN_DEG) + OGM_LIDAR_YAW_OFFSET_DEG;
    rel_angle_rad = wrap_pi(-deg_to_rad(lidar_angle_deg));
    x_m = dist_m * cosf(rel_angle_rad);
    y_m = dist_m * sinf(rel_angle_rad);

    if (fabsf(x_m) > MAZE_SIMPLE_START_ALIGN_MAX_ABS_X_M)
    {
      continue;
    }
    if (side_sign > 0)
    {
      if (y_m < MAZE_SIMPLE_START_ALIGN_MIN_SIDE_M)
      {
        continue;
      }
    }
    else
    {
      if (y_m > -MAZE_SIMPLE_START_ALIGN_MIN_SIDE_M)
      {
        continue;
      }
    }

    sum_x += x_m;
    sum_y += y_m;
    sum_xx += x_m * x_m;
    sum_xy += x_m * y_m;
    n++;
  }

  *points = n;
  if (n < MAZE_SIMPLE_START_ALIGN_MIN_POINTS)
  {
    return 0U;
  }

  den = ((float)n * sum_xx) - (sum_x * sum_x);
  if (fabsf(den) < 0.0001f)
  {
    return 0U;
  }

  slope = (((float)n * sum_xy) - (sum_x * sum_y)) / den;
  angle = atanf(slope);
  if (fabsf(angle) > deg_to_rad(MAZE_SIMPLE_START_ALIGN_MAX_ANGLE_DEG))
  {
    return 0U;
  }

  *angle_rad = angle;
  return 1U;
#else
  (void)side_sign;
  (void)count;
  (void)angle_rad;
  (void)points;
  return 0U;
#endif
}

static uint8_t maze_simple_start_align_measure(float *angle_rad, uint16_t *support)
{
#if ODOM_OGM_ENABLE
  uint16_t count;
  uint16_t left_points = 0U;
  uint16_t right_points = 0U;
  float left_angle = 0.0f;
  float right_angle = 0.0f;
  uint8_t left_ok;
  uint8_t right_ok;

  if (angle_rad == NULL || support == NULL)
  {
    return 0U;
  }

  count = Lidar_CopyScanBinsMm(ogm_scan_bins_mm,
                               (uint16_t)LIDAR_SCAN_BIN_COUNT);
  if (count == 0U)
  {
    return 0U;
  }

  for (uint16_t i = 0U; i < count; i++)
  {
    ogm_scan_filtered_bins_mm[i] = OGM_FilterScanBinMm(i, count);
  }

  left_ok = maze_simple_start_align_fit_side(1, count, &left_angle, &left_points);
  right_ok = maze_simple_start_align_fit_side(-1, count, &right_angle, &right_points);

  if (left_ok != 0U && right_ok != 0U)
  {
    uint16_t total = (uint16_t)(left_points + right_points);
    if (total == 0U)
    {
      return 0U;
    }
    *angle_rad = ((left_angle * (float)left_points) +
                  (right_angle * (float)right_points)) / (float)total;
    *support = total;
    return 1U;
  }
  if (left_ok != 0U)
  {
    *angle_rad = left_angle;
    *support = left_points;
    return 1U;
  }
  if (right_ok != 0U)
  {
    *angle_rad = right_angle;
    *support = right_points;
    return 1U;
  }

  return 0U;
#else
  (void)angle_rad;
  (void)support;
  return 0U;
#endif
}

static void maze_simple_start_align_commit_heading(void)
{
#if ODOM_OGM_ENABLE
  if (maze_simple_start_align_preserve_map == 0U)
  {
    Odom_Reset();
    __disable_irq();
    odom_pulse_accum_left = 0;
    odom_pulse_accum_right = 0;
    __enable_irq();
#if OGM_RUNTIME_ENABLE
    OGM_Reset();
#endif
  }
#endif

  if (imu_yaw_usable_for_heading() != 0U)
  {
    maze_simple_cardinal_ref_yaw_rad = deg_to_rad(yaw);
    maze_simple_cardinal_ref_valid = 1U;
    maze_simple_backup_ref_yaw_rad = deg_to_rad(yaw);
    maze_simple_backup_ref_yaw_valid = 1U;
#if ODOM_OGM_ENABLE
    if (maze_simple_start_align_preserve_map != 0U)
    {
      if (maze_simple_start_align_ref_valid != 0U)
      {
        float yaw_delta_rad = wrap_pi(deg_to_rad(yaw) -
                                      maze_simple_start_align_ref_yaw_rad);
        float map_heading_rad =
          wrap_pi(maze_simple_start_align_ref_map_heading_rad +
                  (OGM_IMU_HEADING_SIGN * yaw_delta_rad));

        ogm_heading_ref_yaw_rad = deg_to_rad(yaw);
        ogm_heading_ref_theta_rad = map_heading_rad;
        ogm_heading_ref_valid = 1U;
        ogm_grid_heading_rad = map_heading_rad;
        ogm_grid_heading_valid = 1U;
      }
      else
      {
        OGM_EnsureGridHeading(odom_theta_rad);
      }
    }
    else
    {
      ogm_heading_ref_yaw_rad = deg_to_rad(yaw);
      ogm_heading_ref_theta_rad = odom_theta_rad;
      ogm_heading_ref_valid = 1U;
      ogm_grid_heading_rad = odom_theta_rad;
      ogm_grid_heading_valid = 1U;
    }
    OGM_ClearPoseHistory();
#endif
    maze_simple_start_align_ref_valid = 0U;
  }
}

static uint8_t maze_simple_start_align_step(uint32_t now_tick)
{
#if MAZE_SIMPLE_START_ALIGN_ENABLE
  if (maze_simple_start_align_phase == 0U)
  {
    float wall_angle_rad = 0.0f;
    uint16_t support = 0U;

    if ((now_tick - maze_state_tick) < MAZE_SIMPLE_START_ALIGN_WAIT_MS)
    {
      MazeDrive(0.0f, 0.0f, now_tick);
      return 0U;
    }

    if (imu_yaw_usable_for_heading() == 0U)
    {
      if ((now_tick - maze_state_tick) >= MAZE_SIMPLE_START_ALIGN_SCAN_MAX_MS)
      {
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)"ALIGN:SKIP IMU\r\n", 16U, 80);
        maze_simple_start_align_commit_heading();
        return 1U;
      }
      MazeDrive(0.0f, 0.0f, now_tick);
      return 0U;
    }

    if (maze_simple_start_align_measure(&wall_angle_rad, &support) != 0U)
    {
      char msg[72];
      int n;
      float wall_angle_deg = wall_angle_rad * (180.0f / PI_F);
      float corr_deg = clampf(wall_angle_deg * MAZE_SIMPLE_START_ALIGN_GAIN,
                              -MAZE_SIMPLE_START_ALIGN_MAX_CORR_DEG,
                              MAZE_SIMPLE_START_ALIGN_MAX_CORR_DEG);
      float abs_corr_deg = fabsf(corr_deg);

      if (support < MAZE_SIMPLE_START_ALIGN_MIN_SUPPORT)
      {
        if ((now_tick - maze_state_tick) >= MAZE_SIMPLE_START_ALIGN_SCAN_MAX_MS)
        {
          n = snprintf(msg, sizeof(msg),
                       "ALIGN:SKIP WEAK E=%.1f N=%u\r\n",
                       (double)wall_angle_deg,
                       (unsigned)support);
          if (n > 0)
          {
            uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
            (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
          }
          maze_simple_start_align_commit_heading();
          return 1U;
        }

        MazeDrive(0.0f, 0.0f, now_tick);
        return 0U;
      }

      maze_simple_start_align_target_yaw_rad = wrap_pi(deg_to_rad(yaw) + deg_to_rad(corr_deg));
      n = snprintf(msg, sizeof(msg),
                   "ALIGN:WALL E=%.1f C=%.1f N=%u\r\n",
                   (double)wall_angle_deg,
                   (double)corr_deg,
                   (unsigned)support);
      if (n > 0)
      {
        uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
      }

#if MAZE_SIMPLE_START_ALIGN_PHYSICAL_ENABLE == 0U
      maze_simple_start_align_commit_heading();
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)"ALIGN:SOFT\r\n", 12U, 80);
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)"ALIGN:DONE\r\n", 12U, 80);
      return 1U;
#else

      if (abs_corr_deg < MAZE_SIMPLE_START_ALIGN_MIN_CORR_DEG ||
          fabsf(wall_angle_deg) <= MAZE_SIMPLE_START_ALIGN_DEADBAND_DEG)
      {
        maze_simple_start_align_commit_heading();
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)"ALIGN:DONE\r\n", 12U, 80);
        return 1U;
      }

      maze_simple_start_align_phase = 1U;
      maze_simple_start_align_tick = now_tick;
#endif
    }
    else
    {
      if ((now_tick - maze_state_tick) >= MAZE_SIMPLE_START_ALIGN_SCAN_MAX_MS)
      {
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)"ALIGN:SKIP WALL\r\n", 17U, 80);
        maze_simple_start_align_commit_heading();
        return 1U;
      }
      MazeDrive(0.0f, 0.0f, now_tick);
      return 0U;
    }
  }

  if (maze_simple_start_align_phase == 1U)
  {
    float yaw_err_deg;
    float abs_err_deg;
    float turn_pwm;

    if (imu_yaw_usable_for_heading() == 0U)
    {
      maze_simple_start_align_phase = 2U;
      maze_simple_start_align_tick = now_tick;
      MazeDrive(0.0f, 0.0f, now_tick);
      return 0U;
    }

    yaw_err_deg = wrap_pi(maze_simple_start_align_target_yaw_rad - deg_to_rad(yaw)) *
                  (180.0f / PI_F);
    abs_err_deg = fabsf(yaw_err_deg);

    if (abs_err_deg <= MAZE_SIMPLE_START_ALIGN_DEADBAND_DEG ||
        (now_tick - maze_simple_start_align_tick) >= MAZE_SIMPLE_START_ALIGN_TURN_MAX_MS)
    {
      maze_simple_start_align_phase = 2U;
      maze_simple_start_align_tick = now_tick;
      MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
      return 0U;
    }

    turn_pwm = clampf(abs_err_deg * MAZE_SIMPLE_START_ALIGN_KP_PWM_PER_DEG,
                      MAZE_SIMPLE_START_ALIGN_PWM_MIN,
                      MAZE_SIMPLE_START_ALIGN_PWM_MAX);
    maze_centerline_drive_yaw_turn(yaw_err_deg, turn_pwm, now_tick);
    return 0U;
  }

  if (maze_simple_start_align_phase == 2U)
  {
    if ((now_tick - maze_simple_start_align_tick) < MAZE_SIMPLE_START_ALIGN_SETTLE_MS)
    {
      MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
      return 0U;
    }

    maze_simple_start_align_commit_heading();
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)"ALIGN:DONE\r\n", 12U, 80);
    return 1U;
  }

  maze_simple_start_align_commit_heading();
  return 1U;
#else
  (void)now_tick;
  return 1U;
#endif
}

static int8_t maze_simple_manhattan_side_guard(uint32_t now_tick, float front_m)
{
#if MAZE_MANHATTAN_GRID_ENABLE
  uint8_t right_front_fresh;
  uint8_t left_front_fresh;
  uint8_t front_too_close =
    (front_m > 0.0f && front_m < MAZE_MANHATTAN_GUARD_FRONT_MIN_M) ? 1U : 0U;

  right_front_fresh = maze_simple_range_fresh(now_tick, rightfrontdist_last_valid_tick);
  left_front_fresh = maze_simple_range_fresh(now_tick, leftfrontdist_last_valid_tick);

  if (front_too_close == 0U)
  {
    return 0;
  }

  if (right_front_fresh != 0U &&
      rightfrontdist > 0.0f &&
      rightfrontdist < MAZE_MANHATTAN_FRONT_DIAG_PANIC_M)
  {
    return 1;
  }

  if (left_front_fresh != 0U &&
      leftfrontdist > 0.0f &&
      leftfrontdist < MAZE_MANHATTAN_FRONT_DIAG_PANIC_M)
  {
    return -1;
  }
#else
  (void)now_tick;
  (void)front_m;
#endif

  return 0;
}

static uint8_t maze_simple_yaw_recover_state_allowed(maze_simple_state_t state)
{
  return (state == MAZE_SIMPLE_FOLLOW ||
          state == MAZE_SIMPLE_START_STRAIGHT ||
          state == MAZE_SIMPLE_CENTER_ENTRY ||
          state == MAZE_SIMPLE_PRE_TURN_FORWARD ||
          state == MAZE_SIMPLE_POST_FORWARD) ? 1U : 0U;
}

static uint8_t maze_simple_yaw_guard_check(uint32_t now_tick)
{
#if MAZE_SIMPLE_YAW_GUARD_ENABLE
  float target_rad;
  float err_deg;

  if (maze_simple_yaw_recover_state_allowed(maze_simple_state) == 0U ||
      imu_yaw_usable_for_heading() == 0U ||
      maze_simple_cardinal_ref_valid == 0U ||
      maze_return_active != 0U)
  {
    maze_simple_yaw_guard_tick = 0U;
    return 0U;
  }

  target_rad = maze_simple_nearest_cardinal_yaw_rad(deg_to_rad(yaw));
  err_deg = wrap_pi(target_rad - deg_to_rad(yaw)) * (180.0f / PI_F);
  if (fabsf(err_deg) < MAZE_SIMPLE_YAW_GUARD_ERR_DEG)
  {
    maze_simple_yaw_guard_tick = 0U;
    return 0U;
  }

  if (maze_simple_yaw_guard_tick == 0U)
  {
    maze_simple_yaw_guard_tick = now_tick;
    return 0U;
  }

  if ((now_tick - maze_simple_yaw_guard_tick) < MAZE_SIMPLE_YAW_GUARD_CONFIRM_MS)
  {
    return 0U;
  }

  maze_simple_yaw_recover_resume_state = maze_simple_state;
  maze_simple_yaw_recover_target_rad = target_rad;
  maze_simple_yaw_guard_tick = 0U;
  maze_simple_enter(MAZE_SIMPLE_YAW_RECOVER, now_tick);
  MazeDrive(0.0f, 0.0f, now_tick);
  return 1U;
#else
  (void)now_tick;
  return 0U;
#endif
}

static uint8_t maze_simple_yaw_recover_step(uint32_t now_tick)
{
  if (maze_simple_yaw_recover_phase == 0U)
  {
    if ((now_tick - maze_simple_yaw_recover_tick) < MAZE_SIMPLE_YAW_RECOVER_BRAKE_MS)
    {
      MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
      return 0U;
    }
    maze_simple_yaw_recover_phase = 1U;
    maze_simple_yaw_recover_tick = now_tick;
  }

  if (maze_simple_yaw_recover_phase == 1U)
  {
    float yaw_err_deg;
    float abs_err_deg;
    float turn_pwm;

    if (imu_yaw_usable_for_heading() == 0U)
    {
      maze_simple_yaw_recover_phase = 2U;
      maze_simple_yaw_recover_tick = now_tick;
      MazeDrive(0.0f, 0.0f, now_tick);
      return 0U;
    }

    yaw_err_deg = wrap_pi(maze_simple_yaw_recover_target_rad - deg_to_rad(yaw)) *
                  (180.0f / PI_F);
    abs_err_deg = fabsf(yaw_err_deg);
    if (abs_err_deg <= MAZE_SIMPLE_YAW_RECOVER_DEADBAND_DEG ||
        (now_tick - maze_simple_yaw_recover_tick) >= MAZE_SIMPLE_YAW_RECOVER_MAX_MS)
    {
      maze_simple_yaw_recover_phase = 2U;
      maze_simple_yaw_recover_tick = now_tick;
      MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
      return 0U;
    }

    turn_pwm = clampf(abs_err_deg * MAZE_SIMPLE_YAW_RECOVER_KP_PWM_PER_DEG,
                      MAZE_SIMPLE_YAW_RECOVER_PWM_MIN,
                      MAZE_SIMPLE_YAW_RECOVER_PWM_MAX);
    maze_centerline_drive_yaw_turn(yaw_err_deg, turn_pwm, now_tick);
    return 0U;
  }

  if ((now_tick - maze_simple_yaw_recover_tick) < MAZE_SIMPLE_YAW_RECOVER_SETTLE_MS)
  {
    MazeBrake(MAZE_SIMPLE_STOP_BRAKE_PWM, now_tick);
    return 0U;
  }

  maze_simple_backup_ref_yaw_rad = maze_simple_yaw_recover_target_rad;
  maze_simple_backup_ref_yaw_valid = 1U;
  return 1U;
}

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
static maze_return_action_t MazeReturn_HandleStopScan(uint32_t now_tick,
                                                      const maze_return_stop_scan_input_t *in)
{
  uint8_t return_decision;

  if (in == NULL || maze_return_active == 0U)
  {
    return MAZE_RETURN_ACTION_NONE;
  }

#if MAZE_ASTAR_ENABLE
  if (maze_return_astar_active != 0U)
  {
    uint8_t goal_ready;

    if (MazeAstar_ReturnDynamicDecision(&return_decision, &goal_ready) == 0U)
    {
      MazeTopo_StopReturn(0U, now_tick);
      return MazeReturn_ActionFromState();
    }

    if (maze_return_phase == MAZE_RETURN_PHASE_ALIGN_TO_PATH)
    {
      if (in->astar_block_tick != NULL)
      {
        *in->astar_block_tick = 0U;
      }
      return MazeReturn_ActionFromState();
    }

    if (goal_ready != 0U)
    {
      if (maze_return_home_pending == 0U)
      {
        MazeAstar_ReturnBeginHome(now_tick);
      }
      return MazeReturn_EnterFollow(now_tick);
    }

    if (return_decision == MAZE_TOPO_DECISION_RIGHT)
    {
      if ((in->right_open_stop != 0U ||
           in->right_probe != 0U ||
           in->right_front_open != 0U) &&
          maze_simple_turn_clear_for_state(MAZE_SIMPLE_TURN_RIGHT, now_tick) != 0U)
      {
        if (in->astar_block_tick != NULL)
        {
          *in->astar_block_tick = 0U;
        }
        return MazeReturn_ScheduleTurnAction(MAZE_SIMPLE_TURN_RIGHT, now_tick);
      }

      if (maze_astar_segment_turn_pending == 0U &&
          (in->front_clear != 0U ||
           in->front_m <= 0.0f ||
           in->front_m > MAZE_SIMPLE_APPROACH_FRONT_MIN_M ||
           (MazeReturn_PostAlignReplanGuardActive(now_tick) != 0U &&
            in->front_m > MAZE_ASTAR_POST_ALIGN_FRONT_MIN_M)))
      {
        if (in->astar_block_tick != NULL)
        {
          *in->astar_block_tick = 0U;
        }
        return MazeReturn_EnterFollow(now_tick);
      }

      if (in->topo_deadend != 0U &&
          MazeReturn_PostAlignReplanGuardActive(now_tick) != 0U)
      {
        if (in->astar_block_tick != NULL)
        {
          *in->astar_block_tick = 0U;
        }
        return MazeReturn_StopDrive(now_tick);
      }

      if (in->astar_block_tick != NULL)
      {
        if (*in->astar_block_tick == 0U)
        {
          *in->astar_block_tick = now_tick;
        }
        else if ((now_tick - *in->astar_block_tick) >= MAZE_ASTAR_REPLAN_BLOCK_MS)
        {
          if (MazeAstar_ReturnReplanForBlock(now_tick,
                                             in->front_m,
                                             now_tick - *in->astar_block_tick,
                                             return_decision) == 0U)
          {
            MazeTopo_StopReturn(0U, now_tick);
            return MazeReturn_ActionFromState();
          }
          *in->astar_block_tick = 0U;
          if (maze_return_phase == MAZE_RETURN_PHASE_ALIGN_TO_PATH ||
              maze_return_phase == MAZE_RETURN_PHASE_HOME_APPROACH)
          {
            return MazeReturn_ActionFromState();
          }
        }
      }

      return MazeReturn_StopDrive(now_tick);
    }

    if (return_decision == MAZE_TOPO_DECISION_LEFT)
    {
      if ((in->left_open != 0U ||
           in->left_front_open != 0U) &&
          maze_simple_turn_clear_for_state(MAZE_SIMPLE_TURN_LEFT, now_tick) != 0U)
      {
        if (in->astar_block_tick != NULL)
        {
          *in->astar_block_tick = 0U;
        }
        return MazeReturn_ScheduleTurnAction(MAZE_SIMPLE_TURN_LEFT, now_tick);
      }

      if (maze_astar_segment_turn_pending == 0U &&
          (in->front_clear != 0U ||
           in->front_m <= 0.0f ||
           in->front_m > MAZE_SIMPLE_APPROACH_FRONT_MIN_M ||
           (MazeReturn_PostAlignReplanGuardActive(now_tick) != 0U &&
            in->front_m > MAZE_ASTAR_POST_ALIGN_FRONT_MIN_M)))
      {
        if (in->astar_block_tick != NULL)
        {
          *in->astar_block_tick = 0U;
        }
        return MazeReturn_EnterFollow(now_tick);
      }

      if (in->topo_deadend != 0U &&
          MazeReturn_PostAlignReplanGuardActive(now_tick) != 0U)
      {
        if (in->astar_block_tick != NULL)
        {
          *in->astar_block_tick = 0U;
        }
        return MazeReturn_StopDrive(now_tick);
      }

      if (in->astar_block_tick != NULL)
      {
        if (*in->astar_block_tick == 0U)
        {
          *in->astar_block_tick = now_tick;
        }
        else if ((now_tick - *in->astar_block_tick) >= MAZE_ASTAR_REPLAN_BLOCK_MS)
        {
          if (MazeAstar_ReturnReplanForBlock(now_tick,
                                             in->front_m,
                                             now_tick - *in->astar_block_tick,
                                             return_decision) == 0U)
          {
            MazeTopo_StopReturn(0U, now_tick);
            return MazeReturn_ActionFromState();
          }
          *in->astar_block_tick = 0U;
          if (maze_return_phase == MAZE_RETURN_PHASE_ALIGN_TO_PATH ||
              maze_return_phase == MAZE_RETURN_PHASE_HOME_APPROACH)
          {
            return MazeReturn_ActionFromState();
          }
        }
      }

      return MazeReturn_StopDrive(now_tick);
    }

    if (return_decision == MAZE_TOPO_DECISION_STRAIGHT)
    {
      if (in->front_clear != 0U ||
          in->front_m <= 0.0f ||
          in->front_m > MAZE_SIMPLE_APPROACH_FRONT_MIN_M ||
          (MazeReturn_PostAlignReplanGuardActive(now_tick) != 0U &&
           in->front_m > MAZE_ASTAR_POST_ALIGN_FRONT_MIN_M) ||
          in->wait_timeout != 0U)
      {
        if (in->astar_block_tick != NULL)
        {
          *in->astar_block_tick = 0U;
        }
        return MazeReturn_EnterFollow(now_tick);
      }

      if (in->astar_block_tick != NULL)
      {
        if (*in->astar_block_tick == 0U)
        {
          *in->astar_block_tick = now_tick;
        }
        else if ((now_tick - *in->astar_block_tick) >= MAZE_ASTAR_REPLAN_BLOCK_MS)
        {
          if (MazeAstar_ReturnReplanForBlock(now_tick,
                                             in->front_m,
                                             now_tick - *in->astar_block_tick,
                                             return_decision) == 0U)
          {
            MazeTopo_StopReturn(0U, now_tick);
            return MazeReturn_ActionFromState();
          }
          *in->astar_block_tick = 0U;
          if (maze_return_phase == MAZE_RETURN_PHASE_ALIGN_TO_PATH ||
              maze_return_phase == MAZE_RETURN_PHASE_HOME_APPROACH)
          {
            return MazeReturn_ActionFromState();
          }
        }
      }

      return MazeReturn_StopDrive(now_tick);
    }

    if (return_decision == MAZE_TOPO_DECISION_BACKUP)
    {
      if (in->astar_block_tick != NULL)
      {
        *in->astar_block_tick = 0U;
      }
      return MazeReturn_EnterTurnBack(now_tick);
    }

    return MazeReturn_StopDrive(now_tick);
  }
#endif

  return_decision = MazeTopo_ReturnPeekDecision();

  if (return_decision == MAZE_TOPO_DECISION_NONE)
  {
    if (maze_return_home_pending != 0U ||
        maze_return_remaining == 0U)
    {
      if (maze_return_home_pending == 0U)
      {
        MazeTopo_ReturnConsumeDecision(now_tick);
      }
      return MazeReturn_EnterFollow(now_tick);
    }

    MazeTopo_StopReturn(0U, now_tick);
    return MazeReturn_ActionFromState();
  }

  if (return_decision == MAZE_TOPO_DECISION_RIGHT)
  {
    if ((in->right_open_stop != 0U ||
         in->right_probe != 0U ||
         in->right_front_open != 0U ||
         in->wait_timeout != 0U) &&
        maze_simple_turn_clear_for_state(MAZE_SIMPLE_TURN_RIGHT, now_tick) != 0U)
    {
      MazeTopo_ReturnConsumeDecision(now_tick);
      return MazeReturn_ScheduleTurnAction(MAZE_SIMPLE_TURN_RIGHT, now_tick);
    }

    return MazeReturn_StopDrive(now_tick);
  }

  if (return_decision == MAZE_TOPO_DECISION_LEFT)
  {
    if ((in->left_open != 0U ||
         in->left_front_open != 0U ||
         in->wait_timeout != 0U) &&
        maze_simple_turn_clear_for_state(MAZE_SIMPLE_TURN_LEFT, now_tick) != 0U)
    {
      MazeTopo_ReturnConsumeDecision(now_tick);
      return MazeReturn_ScheduleTurnAction(MAZE_SIMPLE_TURN_LEFT, now_tick);
    }

    return MazeReturn_StopDrive(now_tick);
  }

  if (return_decision == MAZE_TOPO_DECISION_STRAIGHT)
  {
    if (in->front_clear != 0U ||
        in->front_m <= 0.0f ||
        in->front_m > MAZE_SIMPLE_APPROACH_FRONT_MIN_M ||
        in->wait_timeout != 0U)
    {
      MazeTopo_ReturnConsumeDecision(now_tick);
      return MazeReturn_EnterFollow(now_tick);
    }

    return MazeReturn_StopDrive(now_tick);
  }

  if (return_decision == MAZE_TOPO_DECISION_BACKUP)
  {
    MazeTopo_ReturnConsumeDecision(now_tick);
    return MazeReturn_EnterTurnBack(now_tick);
  }

  return MazeReturn_ActionFromState();
}
#endif

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE && MAZE_ASTAR_ENABLE
static maze_return_action_t MazeReturn_HandleFollowAstar(uint32_t now_tick,
                                                         const maze_return_follow_input_t *in)
{
  uint8_t return_decision;
  uint8_t goal_ready;

  if (in == NULL ||
      maze_return_active == 0U ||
      maze_return_astar_active == 0U ||
      maze_simple_state != MAZE_SIMPLE_FOLLOW)
  {
    return MAZE_RETURN_ACTION_NONE;
  }

  if (maze_return_phase == MAZE_RETURN_PHASE_ALIGN_TO_PATH)
  {
    MazeReturn_FinishPathAlign(now_tick);
  }

  if (in->left_branch_candidate_tick != NULL)
  {
    *in->left_branch_candidate_tick = 0U;
  }
  if (in->right_branch_candidate_tick != NULL)
  {
    *in->right_branch_candidate_tick = 0U;
  }
  if (in->stall_start_tick != NULL)
  {
    *in->stall_start_tick = 0U;
  }
  maze_simple_prefer_turn_dir = 0;

  if (MazeAstar_ReturnDynamicDecision(&return_decision, &goal_ready) == 0U)
  {
    MazeTopo_StopReturn(0U, now_tick);
    return MazeReturn_ActionFromState();
  }

  if (maze_return_phase == MAZE_RETURN_PHASE_ALIGN_TO_PATH ||
      maze_simple_state != MAZE_SIMPLE_FOLLOW)
  {
    return MazeReturn_ActionFromState();
  }

  if (goal_ready != 0U)
  {
    if (in->astar_block_tick != NULL)
    {
      *in->astar_block_tick = 0U;
    }
    if (maze_return_home_pending == 0U)
    {
      MazeAstar_ReturnBeginHome(now_tick);
    }
    return MazeReturn_DriveStraightAction(now_tick);
  }

  if (return_decision != MAZE_TOPO_DECISION_STRAIGHT)
  {
    if (in->astar_block_tick != NULL)
    {
      *in->astar_block_tick = 0U;
    }
    if (return_decision == MAZE_TOPO_DECISION_RIGHT)
    {
      uint8_t right_ready =
        ((in->right_open != 0U) ||
         (maze_simple_right_probe_candidate(now_tick, in->front_m) != 0U) ||
         (maze_simple_range_fresh(now_tick, rightfrontdist_last_valid_tick) != 0U &&
          rightfrontdist > MAZE_SIMPLE_RIGHT_FRONT_OPEN_M &&
          maze_simple_right_hard_blocked(now_tick) == 0U)) ? 1U : 0U;

      if (maze_astar_segment_turn_pending == 0U &&
          right_ready == 0U &&
          (maze_simple_front_clear(in->front_m) != 0U ||
           in->front_m <= 0.0f ||
           in->front_m > MAZE_SIMPLE_APPROACH_FRONT_MIN_M))
      {
        return MazeReturn_DriveStraightAction(now_tick);
      }
    }
    else if (return_decision == MAZE_TOPO_DECISION_LEFT)
    {
      uint8_t left_ready =
        ((in->left_open != 0U) ||
         (maze_simple_range_fresh(now_tick, leftfrontdist_last_valid_tick) != 0U &&
          leftfrontdist > MAZE_SIMPLE_SIDE_DIAG_OPEN_M)) ? 1U : 0U;

      if (maze_astar_segment_turn_pending == 0U &&
          left_ready == 0U &&
          (maze_simple_front_clear(in->front_m) != 0U ||
           in->front_m <= 0.0f ||
           in->front_m > MAZE_SIMPLE_APPROACH_FRONT_MIN_M))
      {
        return MazeReturn_DriveStraightAction(now_tick);
      }
    }

    return MazeReturn_StopScanBrake(now_tick);
  }

  if (in->front_m > 0.0f &&
      in->front_m <= MAZE_SIMPLE_APPROACH_FRONT_MIN_M)
  {
    if (MazeReturn_PostAlignReplanGuardActive(now_tick) != 0U &&
        in->front_m > MAZE_ASTAR_POST_ALIGN_FRONT_MIN_M)
    {
      if (in->astar_block_tick != NULL)
      {
        *in->astar_block_tick = 0U;
      }
      return MazeReturn_DriveStraightAction(now_tick);
    }

    if (in->astar_block_tick != NULL && *in->astar_block_tick == 0U)
    {
      *in->astar_block_tick = now_tick;
    }
    return MazeReturn_StopScanBrake(now_tick);
  }

  if (in->astar_block_tick != NULL)
  {
    *in->astar_block_tick = 0U;
  }
  return MazeReturn_DriveStraightAction(now_tick);
}
#endif

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
static maze_return_action_t MazeReturn_ActionFromState(void)
{
  if (maze_return_phase == MAZE_RETURN_PHASE_DONE)
  {
    return MAZE_RETURN_ACTION_DONE;
  }
  if (maze_return_phase == MAZE_RETURN_PHASE_FAIL)
  {
    return MAZE_RETURN_ACTION_FAIL;
  }

  switch (maze_simple_state)
  {
    case MAZE_SIMPLE_FOLLOW:
    case MAZE_SIMPLE_START_STRAIGHT:
    case MAZE_SIMPLE_POST_FORWARD:
      return MAZE_RETURN_ACTION_DRIVE;

    case MAZE_SIMPLE_TURN_LEFT:
      return MAZE_RETURN_ACTION_TURN_LEFT;

    case MAZE_SIMPLE_TURN_RIGHT:
      return MAZE_RETURN_ACTION_TURN_RIGHT;

    case MAZE_SIMPLE_TURN_BACK:
      return MAZE_RETURN_ACTION_TURN_BACK;

    case MAZE_SIMPLE_STOP_SCAN:
    case MAZE_SIMPLE_APPROACH_JUNCTION:
    case MAZE_SIMPLE_CENTER_ENTRY:
    case MAZE_SIMPLE_BACKUP:
    case MAZE_SIMPLE_PRE_TURN_FORWARD:
    case MAZE_SIMPLE_POST_TURN_LOCK:
    case MAZE_SIMPLE_START_ALIGN:
    case MAZE_SIMPLE_YAW_RECOVER:
    default:
      return MAZE_RETURN_ACTION_STOP;
  }
}

static maze_return_action_t MazeReturn_Update(uint32_t now_tick,
                                              const maze_return_update_input_t *in)
{
  maze_return_action_t action = MAZE_RETURN_ACTION_NONE;

  if (in == NULL || maze_return_active == 0U)
  {
    maze_return_last_action = MAZE_RETURN_ACTION_NONE;
    return MAZE_RETURN_ACTION_NONE;
  }

  if (in->mode == MAZE_RETURN_UPDATE_STOP_SCAN)
  {
    action = MazeReturn_HandleStopScan(now_tick, in->stop_scan);
    maze_return_last_action = action;
    return action;
  }

  if (in->mode == MAZE_RETURN_UPDATE_FOLLOW)
  {
#if MAZE_ASTAR_ENABLE
    action = MazeReturn_HandleFollowAstar(now_tick, in->follow);
    maze_return_last_action = action;
    return action;
#else
    maze_return_last_action = MAZE_RETURN_ACTION_NONE;
    return MAZE_RETURN_ACTION_NONE;
#endif
  }

  maze_return_last_action = MAZE_RETURN_ACTION_NONE;
  return MAZE_RETURN_ACTION_NONE;
}
#endif

static void MazeSimpleControlTask(uint32_t now_tick)
{
  static uint32_t stall_start_tick = 0U;
  static uint32_t astar_return_block_tick = 0U;
  static uint8_t backup_center_active = 0U;
#if ODOM_OGM_ENABLE
  static float backup_center_start_x_m = 0.0f;
  static float backup_center_start_y_m = 0.0f;
#endif
  static uint32_t backup_center_start_tick = 0U;
  static uint8_t backup_stop_pending = 0U;
  static uint8_t backup_turn_extra_active = 0U;
#if ODOM_OGM_ENABLE
  static float backup_turn_extra_start_x_m = 0.0f;
  static float backup_turn_extra_start_y_m = 0.0f;
#endif
  static uint32_t backup_turn_extra_start_tick = 0U;
  static uint32_t side_open_ignore_until_tick = 0U;
  static uint32_t right_branch_candidate_tick = 0U;
  static uint32_t right_branch_cooldown_until_tick = 0U;
  static uint32_t left_branch_candidate_tick = 0U;
  static uint32_t left_branch_cooldown_until_tick = 0U;
  float front_m = fordist;
  float motion_pulse_abs = fabsf(fdb_A) + fabsf(fdb_B);
  uint8_t front_block_now;
  uint8_t right_open_now;
  uint8_t left_open_now;
  uint8_t side_open_suppressed;

  if (lidar_ranges_all_zero() != 0U &&
      maze_simple_state != MAZE_SIMPLE_START_ALIGN)
  {
    stall_start_tick = 0U;
    if ((int32_t)(now_tick - selfdbg_topo_hold_until_tick) >= 0)
    {
      selfdbg_topo_decision_dir = 0;
      selfdbg_topo_block_reason = 0U;
    }
    selfdbg_topo_pending_dir = maze_topo_pending_branch_dir;
    MazeDrive(0.0f, 0.0f, now_tick);
    return;
  }

  if (maze_topo_pending_branch_dir != 0 &&
      (int32_t)(now_tick - maze_topo_pending_branch_until_tick) >= 0)
  {
    maze_topo_pending_branch_dir = 0;
    maze_topo_pending_branch_until_tick = 0U;
  }
  if ((int32_t)(now_tick - selfdbg_topo_hold_until_tick) >= 0)
  {
    selfdbg_topo_decision_dir = 0;
    selfdbg_topo_block_reason = 0U;
  }
  selfdbg_topo_pending_dir = maze_topo_pending_branch_dir;

  front_block_now = maze_simple_front_blocked(front_m);
  right_open_now = maze_simple_right_open();
  side_open_suppressed =
    ((int32_t)(now_tick - side_open_ignore_until_tick) < 0) ? 1U : 0U;
  if (side_open_suppressed != 0U)
  {
    right_open_now = 0U;
  }
  left_open_now = (side_open_suppressed == 0U) ? maze_simple_left_open() : 0U;
  selfdbg_side_open_suppressed = side_open_suppressed;
  selfdbg_backup_stop_pending = backup_stop_pending;
  selfdbg_backup_turn_extra_active = backup_turn_extra_active;
  if (maze_simple_state != MAZE_SIMPLE_STOP_SCAN)
  {
    selfdbg_backup_turn_dir = 0;
  }
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE && MAZE_ASTAR_ENABLE
  if (maze_return_active == 0U ||
      maze_return_astar_active == 0U ||
      (maze_simple_state != MAZE_SIMPLE_STOP_SCAN &&
       maze_simple_state != MAZE_SIMPLE_FOLLOW))
  {
    astar_return_block_tick = 0U;
  }
#endif

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
  if (MazeTopo_ReturnHomeReached(now_tick) != 0U &&
      maze_simple_state != MAZE_SIMPLE_TURN_RIGHT &&
      maze_simple_state != MAZE_SIMPLE_TURN_LEFT &&
      maze_simple_state != MAZE_SIMPLE_TURN_BACK &&
      maze_simple_state != MAZE_SIMPLE_POST_TURN_LOCK &&
      maze_simple_state != MAZE_SIMPLE_STOP_SCAN)
  {
    MazeTopo_StopReturn(1U, now_tick);
    return;
  }
#endif

  if (maze_simple_state != MAZE_SIMPLE_BACKUP)
  {
    backup_center_active = 0U;
    backup_center_start_tick = 0U;
  }
  if (maze_simple_state != MAZE_SIMPLE_BACKUP &&
      maze_simple_state != MAZE_SIMPLE_STOP_SCAN)
  {
    backup_stop_pending = 0U;
    backup_turn_extra_active = 0U;
    backup_turn_extra_start_tick = 0U;
    selfdbg_backup_stop_pending = 0U;
    selfdbg_backup_turn_extra_active = 0U;
  }

#if MAZE_MANHATTAN_GRID_ENABLE && MAZE_MANHATTAN_SIDE_GUARD_BACKUP_ENABLE
  {
    int8_t side_guard_dir = maze_simple_manhattan_side_guard(now_tick, front_m);
    if (side_guard_dir != 0 &&
        (maze_simple_state == MAZE_SIMPLE_APPROACH_JUNCTION ||
         maze_simple_state == MAZE_SIMPLE_CENTER_ENTRY ||
         maze_simple_state == MAZE_SIMPLE_FOLLOW))
    {
      stall_start_tick = 0U;
      backup_stop_pending = 0U;
      backup_turn_extra_active = 0U;
      backup_turn_extra_start_tick = 0U;
      maze_simple_enter(MAZE_SIMPLE_BACKUP, now_tick);
      maze_simple_prefer_turn_dir = (side_guard_dir > 0) ? -1 : 1;
      maze_simple_drive_backup(now_tick);
      return;
    }
  }
#endif

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
  if ((maze_simple_state == MAZE_SIMPLE_FOLLOW ||
       maze_simple_state == MAZE_SIMPLE_APPROACH_JUNCTION ||
       maze_simple_state == MAZE_SIMPLE_CENTER_ENTRY ||
       maze_simple_state == MAZE_SIMPLE_POST_FORWARD) &&
      MazeTopo_AbortPendingSideIfBlocked(now_tick) != 0U)
  {
    stall_start_tick = 0U;
    backup_stop_pending = 0U;
    backup_turn_extra_active = 0U;
    backup_turn_extra_start_tick = 0U;
    maze_topo_pending_branch_dir = 0;
    maze_topo_pending_branch_until_tick = 0U;
    selfdbg_topo_pending_dir = 0;
    selfdbg_topo_block_reason = 5U;
    selfdbg_topo_hold_until_tick = now_tick + MAZE_SELFDBG_TOPO_HOLD_MS;
    maze_simple_prefer_turn_dir = 0;
    maze_simple_enter(MAZE_SIMPLE_STOP_SCAN, now_tick);
    maze_simple_drive_stop_brake(now_tick);
    return;
  }
#endif

  if (maze_simple_yaw_guard_check(now_tick) != 0U)
  {
    return;
  }

  switch (maze_simple_state)
  {
    case MAZE_SIMPLE_YAW_RECOVER:
      if (maze_simple_yaw_recover_step(now_tick) != 0U)
      {
        maze_simple_enter(maze_simple_yaw_recover_resume_state, now_tick);
      }
      return;

    case MAZE_SIMPLE_START_ALIGN:
      if (maze_simple_start_align_step(now_tick) != 0U)
      {
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
        if (maze_return_align_pending != 0U)
        {
          (void)MazeTopo_StartReturnAfterAlign(now_tick);
          return;
        }
#endif
        maze_simple_start_align_preserve_map = 0U;
        maze_simple_enter(MAZE_SIMPLE_START_STRAIGHT, now_tick);
      }
      return;

    case MAZE_SIMPLE_START_STRAIGHT:
    {
      uint8_t start_reached = 0U;
#if ODOM_OGM_ENABLE
      if (maze_simple_start_valid != 0U)
      {
        float dx = odom_x_m - maze_simple_start_x_m;
        float dy = odom_y_m - maze_simple_start_y_m;
        float ds_m = sqrtf((dx * dx) + (dy * dy));
        if (ds_m >= MAZE_SIMPLE_START_STRAIGHT_M)
        {
          start_reached = 1U;
        }
      }
#endif
      if (front_m > 0.0f && front_m < MAZE_SIMPLE_START_FRONT_MIN_M)
      {
        maze_simple_prefer_turn_dir = 0;
        maze_simple_enter(MAZE_SIMPLE_STOP_SCAN, now_tick);
        maze_simple_drive_stop_brake(now_tick);
        return;
      }

      if (start_reached != 0U ||
          (now_tick - maze_state_tick) >= MAZE_SIMPLE_START_STRAIGHT_MS)
      {
        maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
        return;
      }

      maze_simple_drive_straight(now_tick);
      return;
    }

    case MAZE_SIMPLE_APPROACH_JUNCTION:
    {
      uint8_t left_open_approach =
        (side_open_suppressed == 0U) ? maze_simple_left_open() : 0U;
      if (right_open_now != 0U || left_open_approach != 0U)
      {
        maze_simple_prefer_turn_dir = (right_open_now != 0U) ? 1 : -1;
        maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
        maze_simple_drive_straight(now_tick);
        return;
      }

      if ((front_m > 0.0f && front_m < MAZE_SIMPLE_APPROACH_FRONT_MIN_M) ||
          (now_tick - maze_state_tick) >= MAZE_SIMPLE_APPROACH_JUNCTION_MS)
      {
        maze_simple_prefer_turn_dir = 0;
        maze_simple_enter(MAZE_SIMPLE_STOP_SCAN, now_tick);
        maze_simple_drive_stop_brake(now_tick);
        return;
      }

      maze_simple_drive_straight(now_tick);
      return;
    }

    case MAZE_SIMPLE_CENTER_ENTRY:
    {
      uint8_t center_reached = 0U;
      uint8_t center_turn_min_reached = 0U;
      float center_entry_target_m = maze_simple_center_entry_target_m();
      float center_front_min_m = maze_simple_center_front_min_m();
#if ODOM_OGM_ENABLE
      if (maze_simple_center_start_valid != 0U)
      {
        float dx = odom_x_m - maze_simple_center_start_x_m;
        float dy = odom_y_m - maze_simple_center_start_y_m;
        float ds_m = sqrtf((dx * dx) + (dy * dy));
        if (ds_m >= MAZE_SIMPLE_CENTER_TURN_MIN_M)
        {
          center_turn_min_reached = 1U;
        }
        if (ds_m >= center_entry_target_m)
        {
          center_reached = 1U;
        }
      }
#endif
      uint8_t center_pending_turn = (maze_simple_center_turn_dir != 0) ? 1U : 0U;
      uint8_t center_front_stop =
        (front_m > 0.0f &&
         front_m < ((center_pending_turn != 0U) ?
                    MAZE_SIMPLE_CENTER_TURN_FRONT_HARD_MIN_M :
                    center_front_min_m)) ? 1U : 0U;
      uint8_t center_entry_done =
        (center_front_stop != 0U ||
         center_reached != 0U ||
         (now_tick - maze_state_tick) >= MAZE_SIMPLE_CENTER_ENTRY_MS ||
         (center_pending_turn != 0U && center_turn_min_reached != 0U)) ? 1U : 0U;

      if (center_entry_done != 0U)
      {
        if (maze_simple_center_turn_dir > 0)
        {
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
          MazeTopo_RecordDecision(maze_simple_center_turn_topo_openings,
                                  MAZE_TOPO_DECISION_RIGHT,
                                  now_tick);
#endif
          maze_simple_center_turn_dir = 0;
          maze_simple_center_turn_topo_openings = 0U;
          maze_simple_prefer_turn_dir = 0;
          maze_simple_schedule_turn(MAZE_SIMPLE_TURN_RIGHT, now_tick);
          return;
        }
        if (maze_simple_center_turn_dir < 0)
        {
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
          MazeTopo_RecordDecision(maze_simple_center_turn_topo_openings,
                                  MAZE_TOPO_DECISION_LEFT,
                                  now_tick);
#endif
          maze_simple_center_turn_dir = 0;
          maze_simple_center_turn_topo_openings = 0U;
          maze_simple_prefer_turn_dir = 0;
          maze_simple_schedule_turn(MAZE_SIMPLE_TURN_LEFT, now_tick);
          return;
        }

        maze_simple_enter(MAZE_SIMPLE_STOP_SCAN, now_tick);
        maze_simple_drive_stop_brake(now_tick);
        return;
      }
      maze_simple_drive_straight(now_tick);
      return;
    }

    case MAZE_SIMPLE_PRE_TURN_FORWARD:
    {
      uint8_t nudge_reached = 0U;

      if ((now_tick - maze_state_tick) < MAZE_SIMPLE_PRE_TURN_BRAKE_MS)
      {
        maze_simple_drive_stop_brake(now_tick);
        return;
      }

      if (maze_simple_pending_turn_valid == 0U)
      {
        maze_simple_enter(MAZE_SIMPLE_STOP_SCAN, now_tick);
        maze_simple_drive_stop_brake(now_tick);
        return;
      }

      if (front_m > 0.0f && front_m < MAZE_SIMPLE_PRE_TURN_FRONT_MIN_M)
      {
        maze_simple_state_t target_state = maze_simple_pending_turn_state;
        maze_simple_pending_turn_valid = 0U;
        maze_simple_enter(target_state, now_tick);
        return;
      }

#if ODOM_OGM_ENABLE
      if (maze_simple_pre_turn_start_valid != 0U)
      {
        float dx = odom_x_m - maze_simple_pre_turn_start_x_m;
        float dy = odom_y_m - maze_simple_pre_turn_start_y_m;
        float ds_m = sqrtf((dx * dx) + (dy * dy));
        if (ds_m >= MAZE_SIMPLE_PRE_TURN_FORWARD_M)
        {
          nudge_reached = 1U;
        }
      }
#endif

      if (nudge_reached != 0U ||
          (now_tick - maze_state_tick) >=
          (MAZE_SIMPLE_PRE_TURN_BRAKE_MS + MAZE_SIMPLE_PRE_TURN_FORWARD_MS))
      {
        maze_simple_state_t target_state = maze_simple_pending_turn_state;
        maze_simple_pending_turn_valid = 0U;
        maze_simple_enter(target_state, now_tick);
        return;
      }

      maze_simple_drive_straight(now_tick);
      return;
    }

    case MAZE_SIMPLE_STOP_SCAN:
      if ((now_tick - maze_state_tick) < MAZE_SIMPLE_STOP_BRAKE_MS)
      {
        maze_simple_drive_stop_brake(now_tick);
        return;
      }

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
      MazeTopo_CommitPendingDecision(MAZE_TOPO_DECISION_NONE, now_tick);
#endif

      if (maze_return_active == 0U &&
          backup_stop_pending == 0U &&
          maze_simple_front_blocked(front_m) != 0U &&
          maze_simple_right_open() == 0U &&
          maze_simple_left_open() == 0U &&
          maze_simple_right_probe_candidate(now_tick, front_m) == 0U &&
          (reardist <= 0.0f || reardist > MAZE_SIMPLE_BACKUP_REAR_STOP_M))
      {
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
        MazeTopo_RecordDecision(MAZE_TOPO_OPEN_BACK, MAZE_TOPO_DECISION_BACKUP, now_tick);
#endif
        maze_simple_prefer_turn_dir = 0;
        maze_simple_enter(MAZE_SIMPLE_BACKUP, now_tick);
        maze_simple_drive_backup(now_tick);
        return;
      }

      if (maze_simple_manhattan_lock_step(now_tick) == 0U)
      {
        return;
      }

#if MAZE_CENTERLINE_ENABLE && MAZE_CENTERLINE_RUN_ON_STOP_SCAN
      if (backup_stop_pending == 0U &&
          maze_centerline_done_this_stop == 0U)
      {
        if (maze_centerline_step(now_tick) == 0U)
        {
          return;
        }
        maze_centerline_done_this_stop = 1U;
      }
#endif

      {
        uint8_t wait_timeout = ((now_tick - maze_state_tick) >= MAZE_SIMPLE_DECISION_WAIT_MAX_MS) ? 1U : 0U;
        uint8_t right_unknown =
          (maze_simple_side_pair_ready(now_tick,
                                       rightdist_last_valid_tick,
                                       rightfrontdist_last_valid_tick,
                                       rightreardist_last_valid_tick) == 0U) ? 1U : 0U;
        uint8_t left_unknown =
          (maze_simple_side_pair_ready(now_tick,
                                       leftdist_last_valid_tick,
                                       leftfrontdist_last_valid_tick,
                                       leftreardist_last_valid_tick) == 0U) ? 1U : 0U;
        uint8_t right_open_stop =
          (side_open_suppressed == 0U) ? maze_simple_right_open() : 0U;
        uint8_t left_open_now =
          (side_open_suppressed == 0U) ? maze_simple_left_open() : 0U;
        uint8_t front_clear_now = maze_simple_front_clear(front_m);
        uint8_t right_probe_now = maze_simple_right_probe_candidate(now_tick, front_m);
        uint8_t right_front_open_now =
          (maze_simple_range_fresh(now_tick, rightfrontdist_last_valid_tick) != 0U &&
           rightfrontdist > MAZE_SIMPLE_RIGHT_FRONT_OPEN_M) ? 1U : 0U;
        uint8_t left_front_open_now =
          (maze_simple_range_fresh(now_tick, leftfrontdist_last_valid_tick) != 0U &&
           leftfrontdist > MAZE_SIMPLE_SIDE_DIAG_OPEN_M) ? 1U : 0U;
        uint8_t backup_right_open_now = maze_simple_backup_right_open();
        uint8_t backup_left_open_now = maze_simple_backup_left_open();
        uint8_t stop_scan_side_open_now =
          ((right_open_stop != 0U) ||
           (left_open_now != 0U) ||
           (right_probe_now != 0U) ||
           (right_front_open_now != 0U) ||
           (left_front_open_now != 0U)) ? 1U : 0U;
        uint8_t deadend_turnback_now = 0U;

        if (backup_stop_pending == 0U)
        {
          if (maze_simple_prefer_turn_dir > 0 &&
              maze_topo_pending_branch_dir <= 0 &&
              right_open_stop == 0U &&
              right_probe_now == 0U &&
              right_front_open_now == 0U)
          {
            maze_simple_prefer_turn_dir = 0;
          }
          else if (maze_simple_prefer_turn_dir < 0 &&
                   maze_topo_pending_branch_dir >= 0 &&
                   left_open_now == 0U &&
                   left_front_open_now == 0U)
          {
            maze_simple_prefer_turn_dir = 0;
          }
        }

        int8_t backup_turn_dir = (backup_stop_pending != 0U) ?
                                 maze_simple_backup_choose_turn_dir() :
                                 maze_simple_prefer_turn_dir;

#if MAZE_SIMPLE_BACKUP_SIDE_TURN_ENABLE != 0U
        if (backup_stop_pending != 0U &&
            backup_turn_dir == 0 &&
            maze_simple_prefer_turn_dir != 0)
        {
          backup_turn_dir = maze_simple_prefer_turn_dir;
        }
#endif
        selfdbg_backup_stop_pending = backup_stop_pending;
        selfdbg_backup_turn_dir = backup_turn_dir;
        selfdbg_backup_turn_extra_active = backup_turn_extra_active;

        if (front_clear_now == 0U && wait_timeout != 0U)
        {
          /* Unknown side data is not enough to commit to a turn. It only ends
             the wait; the actual decision still uses fresh F/R/L evidence. */
          (void)right_unknown;
          (void)left_unknown;
        }

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
        uint8_t topo_right_open_now;
        uint8_t topo_left_open_now;
        uint8_t topo_openings = MAZE_TOPO_OPEN_BACK;
        uint8_t topo_explore_decision = MAZE_TOPO_DECISION_NONE;
        uint8_t topo_local_deadend_now = 0U;

        if (backup_stop_pending != 0U)
        {
          topo_right_open_now = maze_simple_backup_right_topo_open(now_tick);
          topo_left_open_now = maze_simple_backup_left_topo_open(now_tick);
        }
        else
        {
          topo_right_open_now =
            ((right_open_stop != 0U) ||
             (right_probe_now != 0U) ||
             (right_front_open_now != 0U &&
              maze_simple_right_hard_blocked(now_tick) == 0U)) ? 1U : 0U;
          topo_left_open_now =
            ((left_open_now != 0U) ||
             (left_front_open_now != 0U)) ? 1U : 0U;
        }

        if (front_clear_now != 0U)
        {
          topo_openings |= MAZE_TOPO_OPEN_FRONT;
        }
        if (topo_right_open_now != 0U)
        {
          topo_openings |= MAZE_TOPO_OPEN_RIGHT;
        }
        if (topo_left_open_now != 0U)
        {
          topo_openings |= MAZE_TOPO_OPEN_LEFT;
        }
        topo_local_deadend_now =
          ((topo_openings & (MAZE_TOPO_OPEN_FRONT |
                             MAZE_TOPO_OPEN_RIGHT |
                             MAZE_TOPO_OPEN_LEFT)) == 0U) ? 1U : 0U;

        if (maze_return_active == 0U)
        {
          topo_explore_decision =
            MazeTopo_SelectExploreDecision(topo_openings, now_tick, backup_stop_pending);
          if (backup_stop_pending == 0U &&
              maze_topo_pending_branch_dir != 0)
          {
            if (maze_topo_pending_branch_dir > 0 &&
                topo_right_open_now != 0U)
            {
              topo_explore_decision = MAZE_TOPO_DECISION_RIGHT;
            }
            else if (maze_topo_pending_branch_dir < 0 &&
                     topo_left_open_now != 0U)
            {
              topo_explore_decision = MAZE_TOPO_DECISION_LEFT;
            }
            else
            {
              maze_topo_pending_branch_dir = 0;
              maze_topo_pending_branch_until_tick = 0U;
            }
          }

          selfdbg_topo_decision_dir = MazeTopo_DecisionDebugValue(topo_explore_decision);
          selfdbg_topo_pending_dir = maze_topo_pending_branch_dir;
          if (selfdbg_topo_decision_dir != 0 ||
              selfdbg_topo_pending_dir != 0)
          {
            selfdbg_topo_hold_until_tick = now_tick + MAZE_SELFDBG_TOPO_HOLD_MS;
          }
          MazeTopo_PrintDecisionDebug(topo_openings,
                                      topo_explore_decision,
                                      backup_stop_pending,
                                      maze_topo_pending_branch_dir,
                                      now_tick);

          if (backup_stop_pending != 0U)
          {
#if MAZE_SIMPLE_BACKUP_SIDE_TURN_ENABLE
            if (topo_explore_decision == MAZE_TOPO_DECISION_RIGHT)
            {
              backup_turn_dir = 1;
            }
            else if (topo_explore_decision == MAZE_TOPO_DECISION_LEFT)
            {
              backup_turn_dir = -1;
            }
            else
#endif
            if (topo_explore_decision == MAZE_TOPO_DECISION_BACKUP)
            {
              backup_turn_dir = 0;
              if (front_clear_now == 0U &&
                  topo_right_open_now == 0U &&
                  topo_left_open_now == 0U)
              {
                deadend_turnback_now = 1U;
              }
            }
          }
          else if (topo_explore_decision == MAZE_TOPO_DECISION_RIGHT)
          {
            maze_simple_prefer_turn_dir = 1;
          }
          else if (topo_explore_decision == MAZE_TOPO_DECISION_LEFT)
          {
            maze_simple_prefer_turn_dir = -1;
          }
        }
        selfdbg_backup_turn_dir = backup_turn_dir;
#endif

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
        if (maze_return_active != 0U)
        {
          maze_return_stop_scan_input_t return_in;
          maze_return_update_input_t return_update;
          maze_return_action_t return_action;
          return_in.front_m = front_m;
          return_in.wait_timeout = wait_timeout;
          return_in.front_clear = front_clear_now;
          return_in.right_open_stop = right_open_stop;
          return_in.right_probe = right_probe_now;
          return_in.right_front_open = right_front_open_now;
          return_in.left_open = left_open_now;
          return_in.left_front_open = left_front_open_now;
          return_in.topo_deadend = topo_local_deadend_now;
          return_in.astar_block_tick = &astar_return_block_tick;
          return_update.mode = MAZE_RETURN_UPDATE_STOP_SCAN;
          return_update.stop_scan = &return_in;
          return_update.follow = NULL;

          return_action = MazeReturn_Update(now_tick, &return_update);
          if (return_action != MAZE_RETURN_ACTION_NONE)
          {
            return;
          }
        }

#endif

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
      if (maze_return_active == 0U &&
          backup_stop_pending == 0U &&
          topo_explore_decision == MAZE_TOPO_DECISION_RIGHT)
      {
        uint8_t right_branch_evidence =
          (topo_right_open_now != 0U ||
           right_front_open_now != 0U ||
           right_probe_now != 0U ||
           right_open_stop != 0U) ? 1U : 0U;

        if (right_branch_evidence != 0U)
        {
          maze_topo_pending_branch_dir = 0;
          maze_topo_pending_branch_until_tick = 0U;
          selfdbg_topo_pending_dir = 0;
          selfdbg_topo_block_reason = 0U;
          maze_simple_prefer_turn_dir = 1;
          maze_simple_center_turn_dir = 1;
          maze_simple_center_turn_topo_openings = topo_openings;
          maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
          maze_simple_drive_straight(now_tick);
          return;
        }

        selfdbg_topo_block_reason = 1U;
        selfdbg_topo_hold_until_tick = now_tick + MAZE_SELFDBG_TOPO_HOLD_MS;
      }

      if (maze_return_active == 0U &&
          backup_stop_pending == 0U &&
          topo_explore_decision == MAZE_TOPO_DECISION_LEFT)
      {
        uint8_t left_branch_evidence =
          (topo_left_open_now != 0U ||
           left_front_open_now != 0U ||
           left_open_now != 0U) ? 1U : 0U;

        if (left_branch_evidence != 0U)
        {
          maze_topo_pending_branch_dir = 0;
          maze_topo_pending_branch_until_tick = 0U;
          selfdbg_topo_pending_dir = 0;
          selfdbg_topo_block_reason = 0U;
          maze_simple_prefer_turn_dir = -1;
          maze_simple_center_turn_dir = -1;
          maze_simple_center_turn_topo_openings = topo_openings;
          maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
          maze_simple_drive_straight(now_tick);
          return;
        }

        selfdbg_topo_block_reason = 1U;
        selfdbg_topo_hold_until_tick = now_tick + MAZE_SELFDBG_TOPO_HOLD_MS;
      }

      if (backup_stop_pending == 0U &&
          topo_explore_decision == MAZE_TOPO_DECISION_STRAIGHT &&
          front_clear_now != 0U)
      {
        MazeTopo_RecordDecision(topo_openings, MAZE_TOPO_DECISION_STRAIGHT, now_tick);
        maze_topo_pending_branch_dir = 0;
        maze_topo_pending_branch_until_tick = 0U;
        selfdbg_topo_pending_dir = 0;
        maze_simple_prefer_turn_dir = 0;
        maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
        return;
      }

      if (backup_stop_pending == 0U &&
          topo_explore_decision == MAZE_TOPO_DECISION_BACKUP)
      {
        MazeTopo_RecordDecision(topo_openings, MAZE_TOPO_DECISION_BACKUP, now_tick);
        maze_topo_pending_branch_dir = 0;
        maze_topo_pending_branch_until_tick = 0U;
        selfdbg_topo_pending_dir = 0;
        maze_simple_prefer_turn_dir = 0;
        if (topo_local_deadend_now != 0U)
        {
          backup_turn_extra_active = 0U;
          backup_turn_extra_start_tick = 0U;
          backup_stop_pending = 0U;
          maze_simple_enter(MAZE_SIMPLE_TURN_BACK, now_tick);
          return;
        }
#if MAZE_TOPO_EXHAUSTED_BACKUP_ENABLE
        backup_turn_extra_active = 0U;
        backup_turn_extra_start_tick = 0U;
        maze_simple_enter(MAZE_SIMPLE_BACKUP, now_tick);
        maze_simple_drive_backup(now_tick);
#else
        maze_simple_enter(MAZE_SIMPLE_TURN_BACK, now_tick);
#endif
        return;
      }
#endif

      if (backup_stop_pending == 0U &&
          front_clear_now != 0U &&
          maze_simple_prefer_turn_dir == 0 &&
          stop_scan_side_open_now == 0U)
      {
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
        MazeTopo_RecordDecision(topo_openings, MAZE_TOPO_DECISION_STRAIGHT, now_tick);
#endif
        maze_topo_pending_branch_dir = 0;
        maze_topo_pending_branch_until_tick = 0U;
        selfdbg_topo_pending_dir = 0;
        maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
        return;
      }

#if MAZE_SIMPLE_BACKUP_SIDE_TURN_ENABLE == 0U
      if (backup_stop_pending != 0U &&
          front_clear_now != 0U)
      {
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
        MazeTopo_RecordDecision(topo_openings, MAZE_TOPO_DECISION_STRAIGHT, now_tick);
#endif
        maze_topo_pending_branch_dir = 0;
        maze_topo_pending_branch_until_tick = 0U;
        selfdbg_topo_pending_dir = 0;
        backup_stop_pending = 0U;
        backup_turn_extra_active = 0U;
        backup_turn_extra_start_tick = 0U;
        maze_simple_prefer_turn_dir = 0;
        maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
        return;
      }
#endif

      if (backup_stop_pending != 0U && backup_turn_dir > 0)
      {
        uint8_t need_extra_backup =
          (maze_simple_backup_turn_extra_clear(backup_turn_dir, now_tick) == 0U &&
           reardist > MAZE_SIMPLE_BACKUP_TURN_REAR_MIN_M) ? 1U : 0U;
        uint8_t extra_done = (need_extra_backup == 0U) ? 1U : 0U;

        if (need_extra_backup != 0U && backup_turn_extra_active == 0U)
        {
          backup_turn_extra_active = 1U;
          backup_turn_extra_start_tick = now_tick;
#if ODOM_OGM_ENABLE
          backup_turn_extra_start_x_m = odom_x_m;
          backup_turn_extra_start_y_m = odom_y_m;
#endif
          extra_done = 0U;
        }

        if (backup_turn_extra_active != 0U)
        {
#if ODOM_OGM_ENABLE
          float ex = odom_x_m - backup_turn_extra_start_x_m;
          float ey = odom_y_m - backup_turn_extra_start_y_m;
          float eds_m = sqrtf((ex * ex) + (ey * ey));
          if (eds_m >= MAZE_SIMPLE_BACKUP_TURN_EXTRA_M)
          {
            extra_done = 1U;
          }
#endif
          if ((now_tick - backup_turn_extra_start_tick) >= MAZE_SIMPLE_BACKUP_TURN_EXTRA_MS ||
              (reardist > 0.0f && reardist <= MAZE_SIMPLE_BACKUP_TURN_REAR_MIN_M) ||
              (maze_simple_backup_turn_extra_clear(backup_turn_dir, now_tick) != 0U))
          {
            extra_done = 1U;
          }

          if (extra_done == 0U)
          {
            maze_simple_drive_backup(now_tick);
            return;
          }

          backup_turn_extra_active = 0U;
          backup_turn_extra_start_tick = 0U;
        }

        if (maze_simple_backup_align_before_turn(now_tick) == 0U)
        {
          selfdbg_topo_block_reason = 4U;
          selfdbg_topo_hold_until_tick = now_tick + MAZE_SELFDBG_TOPO_HOLD_MS;
          return;
        }

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
        MazeTopo_RecordDecision(topo_openings, MAZE_TOPO_DECISION_RIGHT, now_tick);
#endif
        maze_topo_pending_branch_dir = 0;
        maze_topo_pending_branch_until_tick = 0U;
        selfdbg_topo_pending_dir = 0;
        backup_stop_pending = 0U;
        maze_simple_turn_from_backup = 1U;
        maze_simple_prefer_turn_dir = 0;
        maze_simple_enter(MAZE_SIMPLE_TURN_RIGHT, now_tick);
        return;
      }

      if (backup_stop_pending != 0U && backup_turn_dir < 0)
      {
        uint8_t need_extra_backup =
          (maze_simple_backup_turn_extra_clear(backup_turn_dir, now_tick) == 0U &&
           reardist > MAZE_SIMPLE_BACKUP_TURN_REAR_MIN_M) ? 1U : 0U;
        uint8_t extra_done = (need_extra_backup == 0U) ? 1U : 0U;

        if (need_extra_backup != 0U && backup_turn_extra_active == 0U)
        {
          backup_turn_extra_active = 1U;
          backup_turn_extra_start_tick = now_tick;
#if ODOM_OGM_ENABLE
          backup_turn_extra_start_x_m = odom_x_m;
          backup_turn_extra_start_y_m = odom_y_m;
#endif
          extra_done = 0U;
        }

        if (backup_turn_extra_active != 0U)
        {
#if ODOM_OGM_ENABLE
          float ex = odom_x_m - backup_turn_extra_start_x_m;
          float ey = odom_y_m - backup_turn_extra_start_y_m;
          float eds_m = sqrtf((ex * ex) + (ey * ey));
          if (eds_m >= MAZE_SIMPLE_BACKUP_TURN_EXTRA_M)
          {
            extra_done = 1U;
          }
#endif
          if ((now_tick - backup_turn_extra_start_tick) >= MAZE_SIMPLE_BACKUP_TURN_EXTRA_MS ||
              (reardist > 0.0f && reardist <= MAZE_SIMPLE_BACKUP_TURN_REAR_MIN_M) ||
              (maze_simple_backup_turn_extra_clear(backup_turn_dir, now_tick) != 0U))
          {
            extra_done = 1U;
          }

          if (extra_done == 0U)
          {
            maze_simple_drive_backup(now_tick);
            return;
          }

          backup_turn_extra_active = 0U;
          backup_turn_extra_start_tick = 0U;
        }

        if (maze_simple_backup_align_before_turn(now_tick) == 0U)
        {
          selfdbg_topo_block_reason = 4U;
          selfdbg_topo_hold_until_tick = now_tick + MAZE_SELFDBG_TOPO_HOLD_MS;
          return;
        }

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
        MazeTopo_RecordDecision(topo_openings, MAZE_TOPO_DECISION_LEFT, now_tick);
#endif
        maze_topo_pending_branch_dir = 0;
        maze_topo_pending_branch_until_tick = 0U;
        selfdbg_topo_pending_dir = 0;
        backup_stop_pending = 0U;
        maze_simple_turn_from_backup = 1U;
        maze_simple_prefer_turn_dir = 0;
        maze_simple_enter(MAZE_SIMPLE_TURN_LEFT, now_tick);
        return;
      }

      if (backup_stop_pending != 0U && backup_turn_dir == 0)
      {
        if (deadend_turnback_now != 0U)
        {
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
          MazeTopo_RecordDecision(topo_openings, MAZE_TOPO_DECISION_BACKUP, now_tick);
#endif
          maze_topo_pending_branch_dir = 0;
          maze_topo_pending_branch_until_tick = 0U;
          selfdbg_topo_pending_dir = 0;
          backup_stop_pending = 0U;
          backup_center_active = 0U;
          backup_center_start_tick = 0U;
          maze_simple_prefer_turn_dir = 0;
          maze_simple_enter(MAZE_SIMPLE_TURN_BACK, now_tick);
          return;
        }

        if (reardist <= 0.0f || reardist > MAZE_SIMPLE_BACKUP_REAR_STOP_M)
        {
          maze_simple_enter(MAZE_SIMPLE_BACKUP, now_tick);
          maze_simple_drive_backup(now_tick);
          return;
        }

        MazeDrive(0.0f, 0.0f, now_tick);
        return;
      }

      if (backup_stop_pending == 0U &&
          (right_open_stop != 0U || maze_simple_prefer_turn_dir > 0) &&
          ((rightdist > 0.0f &&
            rightdist < MAZE_SIMPLE_RIGHT_TURN_SIDE_SAFE_M) ||
           (rightreardist > 0.0f &&
            rightreardist < MAZE_SIMPLE_RIGHT_TURN_REAR_SAFE_M &&
            (rightdist <= 0.0f || rightdist < MAZE_SIMPLE_RIGHT_OPEN_M))))
      {
        uint8_t right_front_entry_open =
          (maze_simple_range_fresh(now_tick, rightfrontdist_last_valid_tick) != 0U &&
           rightfrontdist > MAZE_SIMPLE_RIGHT_FRONT_OPEN_M) ? 1U : 0U;

        if (maze_simple_prefer_turn_dir > 0 &&
            front_m > MAZE_SIMPLE_RIGHT_CENTER_FRONT_MIN_M)
        {
          maze_simple_prefer_turn_dir = 1;
          maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
          maze_simple_drive_straight(now_tick);
          return;
        }

        if (maze_simple_prefer_turn_dir > 0 &&
            front_m <= 0.0f &&
            maze_simple_turn_clear_for_state(MAZE_SIMPLE_TURN_RIGHT, now_tick) != 0U)
        {
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
          MazeTopo_RecordDecision(topo_openings, MAZE_TOPO_DECISION_RIGHT, now_tick);
#endif
          maze_simple_schedule_turn(MAZE_SIMPLE_TURN_RIGHT, now_tick);
          return;
        }

        if ((right_front_entry_open != 0U || right_open_stop != 0U) &&
            front_m > MAZE_SIMPLE_RIGHT_CENTER_FRONT_MIN_M)
        {
          maze_simple_prefer_turn_dir = 1;
          maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
          maze_simple_drive_straight(now_tick);
          return;
        }

        if (right_probe_now != 0U)
        {
          maze_simple_prefer_turn_dir = 1;
          maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
          maze_simple_drive_straight(now_tick);
          return;
        }

        if (front_m > 0.0f && front_m <= MAZE_SIMPLE_APPROACH_FRONT_MIN_M)
        {
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
          MazeTopo_RecordDecision(topo_openings, MAZE_TOPO_DECISION_BACKUP, now_tick);
#endif
          maze_simple_prefer_turn_dir = 0;
          maze_simple_enter(MAZE_SIMPLE_BACKUP, now_tick);
          maze_simple_drive_backup(now_tick);
          return;
        }
      }

      if (backup_stop_pending == 0U &&
          front_clear_now == 0U &&
          right_probe_now != 0U)
      {
        maze_simple_prefer_turn_dir = 1;
        maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
        maze_simple_drive_straight(now_tick);
        return;
      }

      if (backup_stop_pending == 0U &&
          front_clear_now == 0U &&
          maze_simple_prefer_turn_dir == 0 &&
          right_open_stop == 0U &&
          left_open_now == 0U &&
          right_probe_now == 0U &&
          right_front_open_now == 0U &&
          left_front_open_now == 0U)
      {
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
        MazeTopo_RecordDecision(topo_openings, MAZE_TOPO_DECISION_BACKUP, now_tick);
#endif
        maze_simple_prefer_turn_dir = 0;
        maze_simple_enter(MAZE_SIMPLE_BACKUP, now_tick);
        maze_simple_drive_backup(now_tick);
        return;
      }

      if (backup_stop_pending == 0U &&
          front_clear_now == 0U &&
          right_open_stop != 0U &&
          front_m > MAZE_SIMPLE_RIGHT_CENTER_FRONT_MIN_M)
      {
        maze_simple_prefer_turn_dir = 1;
        maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
        maze_simple_drive_straight(now_tick);
        return;
      }

      if (right_open_stop != 0U &&
          maze_simple_turn_clear_for_state(MAZE_SIMPLE_TURN_RIGHT, now_tick) != 0U)
      {
        maze_simple_prefer_turn_dir = 1;
        maze_simple_center_turn_dir = 1;
        maze_simple_center_turn_topo_openings = topo_openings;
        maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
        maze_simple_drive_straight(now_tick);
        return;
      }

      if (maze_simple_prefer_turn_dir > 0 &&
          maze_simple_turn_clear_for_state(MAZE_SIMPLE_TURN_RIGHT, now_tick) != 0U)
      {
        maze_simple_center_turn_dir = 1;
        maze_simple_center_turn_topo_openings = topo_openings;
        maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
        maze_simple_drive_straight(now_tick);
        return;
      }

      if (front_clear_now != 0U &&
          maze_simple_prefer_turn_dir == 0 &&
          stop_scan_side_open_now == 0U)
      {
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
        MazeTopo_RecordDecision(topo_openings, MAZE_TOPO_DECISION_STRAIGHT, now_tick);
#endif
        maze_simple_prefer_turn_dir = 0;
        maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
        return;
      }

      if (left_open_now != 0U &&
          maze_simple_turn_clear_for_state(MAZE_SIMPLE_TURN_LEFT, now_tick) != 0U)
      {
        maze_simple_prefer_turn_dir = -1;
        maze_simple_center_turn_dir = -1;
        maze_simple_center_turn_topo_openings = topo_openings;
        maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
        maze_simple_drive_straight(now_tick);
        return;
      }

      if (maze_simple_prefer_turn_dir < 0 &&
          maze_simple_turn_clear_for_state(MAZE_SIMPLE_TURN_LEFT, now_tick) != 0U)
      {
        maze_simple_center_turn_dir = -1;
        maze_simple_center_turn_topo_openings = topo_openings;
        maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
        maze_simple_drive_straight(now_tick);
        return;
      }

      if (front_m <= 0.0f || front_m > MAZE_SIMPLE_APPROACH_FRONT_MIN_M)
      {
        maze_simple_prefer_turn_dir = 0;
        maze_simple_enter(MAZE_SIMPLE_APPROACH_JUNCTION, now_tick);
        maze_simple_drive_straight(now_tick);
        return;
      }

      maze_simple_prefer_turn_dir = 0;
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
      MazeTopo_RecordDecision(topo_openings, MAZE_TOPO_DECISION_BACKUP, now_tick);
#endif
      maze_simple_enter(MAZE_SIMPLE_BACKUP, now_tick);
      maze_simple_drive_backup(now_tick);
      return;
      }

    case MAZE_SIMPLE_TURN_RIGHT:
    case MAZE_SIMPLE_TURN_LEFT:
    case MAZE_SIMPLE_TURN_BACK:
      if ((maze_simple_state == MAZE_SIMPLE_TURN_RIGHT ||
           maze_simple_state == MAZE_SIMPLE_TURN_LEFT) &&
          maze_simple_turn_clear_for_state(maze_simple_state, now_tick) == 0U)
      {
        maze_simple_prefer_turn_dir = 0;
        maze_simple_enter(MAZE_SIMPLE_STOP_SCAN, now_tick);
        maze_simple_drive_stop_brake(now_tick);
        return;
      }
      if (maze_simple_turn_finished(maze_simple_state, now_tick) != 0U)
      {
#if MAZE_TOPO_ENABLE
        if (maze_simple_turn_timeout_failed != 0U &&
            maze_simple_return_align_turn_active() != 0U)
        {
          char msg[96];
          int n = snprintf(msg, sizeof(msg),
                           "RET:ALIGN_FAIL TURN A=%.1f S=%.1f T=%.1f Y=%u\r\n",
                           (double)(maze_simple_turn_last_turned_rad * 57.2957795f),
                           (double)(maze_simple_turn_last_signed_rad * 57.2957795f),
                           (double)(maze_simple_turn_last_target_rad * 57.2957795f),
                           (unsigned)maze_simple_turn_last_yaw_gate);
          if (n > 0)
          {
            uint16_t tx_len = (n < (int)sizeof(msg)) ? (uint16_t)n : (uint16_t)(sizeof(msg) - 1U);
            (void)HAL_UART_Transmit(&huart1, (uint8_t *)msg, tx_len, 80);
          }
          MazeTopo_StopReturn(0U, now_tick);
          return;
        }
#endif
        if (maze_simple_state == MAZE_SIMPLE_TURN_BACK &&
            maze_simple_turn_back_step < 2U)
        {
          maze_simple_turn_back_step = 2U;
          maze_simple_enter(MAZE_SIMPLE_TURN_BACK, now_tick);
          maze_simple_turn_drive(MAZE_SIMPLE_TURN_BACK, now_tick);
          return;
        }

#if ODOM_OGM_ENABLE
        OGM_NoteMazeTurnComplete(maze_simple_state);
#endif
        if (maze_simple_state == MAZE_SIMPLE_TURN_BACK)
        {
          maze_simple_turn_back_step = 0U;
        }
        side_open_ignore_until_tick = now_tick + MAZE_SIMPLE_POST_TURN_SIDE_IGNORE_MS;
        maze_simple_post_turn_brake_dir =
          (maze_simple_state == MAZE_SIMPLE_TURN_RIGHT) ? 1 :
          ((maze_simple_state == MAZE_SIMPLE_TURN_LEFT) ? -1 : 0);
        maze_simple_enter(MAZE_SIMPLE_POST_TURN_LOCK, now_tick);
        maze_simple_drive_turn_brake(maze_simple_post_turn_brake_dir, now_tick);
        return;
      }
      maze_simple_turn_drive(maze_simple_state, now_tick);
      return;

    case MAZE_SIMPLE_POST_TURN_LOCK:
      if (maze_simple_post_turn_lock_step(now_tick) != 0U)
      {
        maze_simple_enter(MAZE_SIMPLE_POST_FORWARD, now_tick);
        MazeDrive(0.0f, 0.0f, now_tick);
        return;
      }
      return;

    case MAZE_SIMPLE_BACKUP:
    {
      uint8_t backup_at_junction = 0U;
      uint8_t backup_timed_out = 0U;
      uint8_t left_open_now = maze_simple_left_open();
      uint8_t backup_right_open_now = maze_simple_backup_right_open();
      uint8_t backup_left_open_now = maze_simple_backup_left_open();
      uint8_t side_open_now = (backup_right_open_now != 0U ||
                               backup_left_open_now != 0U ||
                               right_open_now != 0U ||
                               left_open_now != 0U) ? 1U : 0U;
      uint8_t rear_stop_now = (reardist > 0.0f && reardist < MAZE_SIMPLE_BACKUP_REAR_STOP_M) ? 1U : 0U;
      uint8_t rear_hard_stop_now =
        ((maze_simple_range_fresh(now_tick, reardist_last_valid_tick) != 0U &&
          reardist > 0.0f &&
          reardist < MAZE_SIMPLE_BACKUP_REAR_HARD_STOP_M) ||
         (maze_simple_range_fresh(now_tick, rightreardist_last_valid_tick) != 0U &&
          rightreardist > 0.0f &&
          rightreardist < MAZE_SIMPLE_BACKUP_REAR_DIAG_HARD_STOP_M) ||
         (maze_simple_range_fresh(now_tick, leftreardist_last_valid_tick) != 0U &&
          leftreardist > 0.0f &&
          leftreardist < MAZE_SIMPLE_BACKUP_REAR_DIAG_HARD_STOP_M)) ? 1U : 0U;
      uint8_t rear_stop_allowed = rear_stop_now;
#if ODOM_OGM_ENABLE
      uint8_t backup_distance_out = 0U;
      uint8_t backup_center_reached = 0U;
      uint8_t backup_center_min_reached = 0U;
#endif

      if (rear_hard_stop_now != 0U)
      {
        backup_center_active = 0U;
        backup_center_start_tick = 0U;
        backup_stop_pending = 1U;
        maze_simple_enter(MAZE_SIMPLE_STOP_SCAN, now_tick);
        maze_simple_drive_stop_brake(now_tick);
        return;
      }

      if ((now_tick - maze_state_tick) >= MAZE_SIMPLE_BACKUP_MIN_MS &&
          side_open_now != 0U &&
          backup_center_active == 0U)
      {
        backup_center_active = 1U;
        backup_center_start_tick = now_tick;
        maze_simple_prefer_turn_dir = maze_simple_backup_choose_turn_dir();
#if ODOM_OGM_ENABLE
        backup_center_start_x_m = odom_x_m;
        backup_center_start_y_m = odom_y_m;
#endif
      }

#if ODOM_OGM_ENABLE
      if (backup_center_active != 0U)
      {
        float cdx = odom_x_m - backup_center_start_x_m;
        float cdy = odom_y_m - backup_center_start_y_m;
        float cds_m = sqrtf((cdx * cdx) + (cdy * cdy));
        if (cds_m >= MAZE_SIMPLE_BACKUP_CENTER_MIN_M)
        {
          backup_center_min_reached = 1U;
        }
        if (cds_m >= MAZE_SIMPLE_BACKUP_CENTER_M)
        {
          backup_center_reached = 1U;
        }
      }
#endif

      if (backup_center_active != 0U &&
          rear_stop_now != 0U &&
          (reardist <= 0.0f || reardist > MAZE_SIMPLE_BACKUP_REAR_EMERGENCY_M))
      {
        uint8_t center_min_ok =
          ((now_tick - backup_center_start_tick) >= MAZE_SIMPLE_BACKUP_CENTER_MIN_MS) ? 1U : 0U;
#if ODOM_OGM_ENABLE
        if (backup_center_min_reached != 0U)
        {
          center_min_ok = 1U;
        }
#endif
        if (center_min_ok == 0U)
        {
          rear_stop_allowed = 0U;
        }
      }

      if ((now_tick - maze_state_tick) >= MAZE_SIMPLE_BACKUP_MAX_MS)
      {
        backup_timed_out = 1U;
      }

#if ODOM_OGM_ENABLE
      if (maze_simple_backup_start_valid != 0U)
      {
        float dx = odom_x_m - maze_simple_backup_start_x_m;
        float dy = odom_y_m - maze_simple_backup_start_y_m;
        float ds_m = sqrtf((dx * dx) + (dy * dy));
        if (ds_m >= MAZE_SIMPLE_BACKUP_MAX_M)
        {
          backup_distance_out = 1U;
        }
      }
#endif

      if (backup_center_active != 0U)
      {
        uint8_t turn_center_ok =
          ((maze_simple_prefer_turn_dir != 0) &&
           ((now_tick - backup_center_start_tick) >= MAZE_SIMPLE_BACKUP_CENTER_MIN_MS)) ? 1U : 0U;
#if ODOM_OGM_ENABLE
        if (maze_simple_prefer_turn_dir != 0 &&
            backup_center_min_reached != 0U)
        {
          turn_center_ok = 1U;
        }
#endif
        if (turn_center_ok != 0U)
        {
          backup_at_junction = 1U;
        }
        else
        {
          if (rear_stop_allowed == 0U &&
              backup_timed_out == 0U
#if ODOM_OGM_ENABLE
              && backup_distance_out == 0U &&
              backup_center_reached == 0U
#endif
              && (now_tick - backup_center_start_tick) < MAZE_SIMPLE_BACKUP_CENTER_MS)
          {
            maze_simple_drive_backup(now_tick);
            return;
          }

          backup_at_junction = 1U;
        }
      }

      if (backup_at_junction == 0U &&
          backup_timed_out == 0U
#if ODOM_OGM_ENABLE
          && backup_distance_out == 0U
#endif
         )
      {
        maze_simple_drive_backup(now_tick);
        return;
      }

      if (backup_at_junction != 0U)
      {
        backup_center_active = 0U;
        backup_center_start_tick = 0U;
        backup_stop_pending = 1U;
        maze_simple_enter(MAZE_SIMPLE_STOP_SCAN, now_tick);
        maze_simple_drive_stop_brake(now_tick);
        return;
      }

      maze_simple_prefer_turn_dir = maze_simple_backup_choose_turn_dir();
      backup_stop_pending = 1U;
      maze_simple_enter(MAZE_SIMPLE_STOP_SCAN, now_tick);
      maze_simple_drive_stop_brake(now_tick);
      return;
    }

    case MAZE_SIMPLE_POST_FORWARD:
      if ((now_tick - maze_state_tick) < MAZE_SIMPLE_POST_TURN_BRAKE_MS)
      {
        maze_simple_drive_turn_brake(maze_simple_post_turn_brake_dir, now_tick);
        return;
      }

      if ((now_tick - maze_state_tick) < MAZE_SIMPLE_POST_TURN_SETTLE_MS)
      {
        MazeDrive(0.0f, 0.0f, now_tick);
        return;
      }

#if MAZE_CENTERLINE_ENABLE && MAZE_CENTERLINE_RUN_AFTER_TURN
      if (maze_centerline_done_after_turn == 0U)
      {
        if (maze_centerline_step(now_tick) == 0U)
        {
          return;
        }
        maze_centerline_done_after_turn = 1U;
      }
#endif

      if ((now_tick - maze_state_tick) < MAZE_SIMPLE_POST_FORWARD_MS)
      {
        maze_simple_drive_post_turn(now_tick);
        return;
      }
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
      MazeReturn_FinishPathAlign(now_tick);
#endif
      maze_simple_enter(MAZE_SIMPLE_FOLLOW, now_tick);
      return;

    case MAZE_SIMPLE_FOLLOW:
    default:
      break;
  }

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE && MAZE_ASTAR_ENABLE
  if (maze_return_active != 0U &&
      maze_return_astar_active != 0U &&
      maze_simple_state == MAZE_SIMPLE_FOLLOW)
  {
    maze_return_follow_input_t return_in;
    maze_return_update_input_t return_update;
    maze_return_action_t return_action;
    return_in.front_m = front_m;
    return_in.right_open = right_open_now;
    return_in.left_open = left_open_now;
    return_in.astar_block_tick = &astar_return_block_tick;
    return_in.left_branch_candidate_tick = &left_branch_candidate_tick;
    return_in.right_branch_candidate_tick = &right_branch_candidate_tick;
    return_in.stall_start_tick = &stall_start_tick;
    return_update.mode = MAZE_RETURN_UPDATE_FOLLOW;
    return_update.stop_scan = NULL;
    return_update.follow = &return_in;

    return_action = MazeReturn_Update(now_tick, &return_update);
    if (return_action != MAZE_RETURN_ACTION_NONE)
    {
      return;
    }
  }
#endif

  if (maze_simple_state == MAZE_SIMPLE_FOLLOW &&
      left_open_now != 0U &&
      front_m > MAZE_SIMPLE_LEFT_BRANCH_FRONT_MIN_M &&
      ((int32_t)(now_tick - left_branch_cooldown_until_tick) >= 0))
  {
    if (left_branch_candidate_tick == 0U)
    {
      left_branch_candidate_tick = now_tick;
    }

    if ((now_tick - left_branch_candidate_tick) >= MAZE_SIMPLE_LEFT_BRANCH_CONFIRM_MS)
    {
      stall_start_tick = 0U;
      left_branch_candidate_tick = 0U;
      left_branch_cooldown_until_tick = now_tick + MAZE_SIMPLE_LEFT_BRANCH_COOLDOWN_MS;
      maze_simple_prefer_turn_dir = -1;
      maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
      maze_simple_drive_straight(now_tick);
      return;
    }
  }
  else
  {
    left_branch_candidate_tick = 0U;
  }

  if (maze_simple_state == MAZE_SIMPLE_FOLLOW &&
      right_open_now != 0U &&
      front_block_now == 0U &&
      front_m > MAZE_SIMPLE_RIGHT_BRANCH_FRONT_MIN_M &&
      ((int32_t)(now_tick - right_branch_cooldown_until_tick) >= 0))
  {
    uint8_t right_branch_strong =
      ((maze_simple_range_fresh(now_tick, rightdist_last_valid_tick) != 0U &&
        rightdist > MAZE_SIMPLE_RIGHT_BRANCH_STRONG_M) ||
       (maze_simple_range_fresh(now_tick, rightfrontdist_last_valid_tick) != 0U &&
        rightfrontdist > MAZE_SIMPLE_RIGHT_BRANCH_STRONG_M) ||
       (maze_simple_range_fresh(now_tick, rightreardist_last_valid_tick) != 0U &&
        rightreardist > MAZE_SIMPLE_RIGHT_BRANCH_STRONG_M)) ? 1U : 0U;

    if (right_branch_strong != 0U)
    {
      if (right_branch_candidate_tick == 0U)
      {
        right_branch_candidate_tick = now_tick;
      }

      if ((now_tick - right_branch_candidate_tick) >= MAZE_SIMPLE_RIGHT_BRANCH_CONFIRM_MS)
      {
        stall_start_tick = 0U;
        right_branch_candidate_tick = 0U;
        right_branch_cooldown_until_tick = now_tick + MAZE_SIMPLE_RIGHT_BRANCH_COOLDOWN_MS;
        maze_simple_prefer_turn_dir = 1;
        maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
        maze_simple_drive_straight(now_tick);
        return;
      }
    }
    else
    {
      right_branch_candidate_tick = 0U;
    }
  }
  else
  {
    right_branch_candidate_tick = 0U;
  }

  if (right_open_now != 0U &&
      front_m > MAZE_SIMPLE_FRONT_CLEAR_M &&
      front_block_now == 0U)
  {
    right_open_now = 0U;
  }

  if (front_block_now != 0U || right_open_now != 0U)
  {
    stall_start_tick = 0U;
    if (right_open_now != 0U)
    {
      maze_simple_prefer_turn_dir = 1;
    }

    if (right_open_now != 0U &&
        (front_m <= 0.0f || front_m > MAZE_SIMPLE_CENTER_FRONT_MIN_M))
    {
      maze_simple_prefer_turn_dir = 1;
      maze_simple_enter(MAZE_SIMPLE_CENTER_ENTRY, now_tick);
      maze_simple_drive_straight(now_tick);
    }
    else
    {
      if (front_m > MAZE_SIMPLE_APPROACH_FRONT_MIN_M)
      {
        if (right_open_now == 0U)
        {
          maze_simple_prefer_turn_dir = 0;
        }
        maze_simple_enter(MAZE_SIMPLE_APPROACH_JUNCTION, now_tick);
        maze_simple_drive_straight(now_tick);
      }
      else
      {
        if (right_open_now == 0U)
        {
          maze_simple_prefer_turn_dir = 0;
        }
        maze_simple_enter(MAZE_SIMPLE_STOP_SCAN, now_tick);
        maze_simple_drive_stop_brake(now_tick);
      }
    }
    return;
  }

  if (((fabsf(maze_prev_left_cmd) > MAZE_STUCK_CMD_PWM) ||
       (fabsf(maze_prev_right_cmd) > MAZE_STUCK_CMD_PWM)) &&
      motion_pulse_abs < MAZE_STUCK_PULSE_EPS &&
      ((front_m > 0.0f && front_m < (MAZE_SIMPLE_FRONT_BLOCK_M + 0.12f)) ||
       (rightdist > 0.0f && rightdist < MAZE_RIGHT_DANGER_M) ||
       (rightfrontdist > 0.0f && rightfrontdist < MAZE_RIGHT_FRONT_DANGER_M) ||
       (leftdist > 0.0f && leftdist < MAZE_LEFT_DANGER_M) ||
       (leftfrontdist > 0.0f && leftfrontdist < MAZE_LEFT_FRONT_DANGER_M)))
  {
    if (stall_start_tick == 0U)
    {
      stall_start_tick = now_tick;
    }
    else if ((now_tick - stall_start_tick) >= MAZE_STUCK_DETECT_MS)
    {
      stall_start_tick = 0U;
      maze_simple_prefer_turn_dir = 0;
      maze_simple_enter(MAZE_SIMPLE_BACKUP, now_tick);
      maze_simple_drive_backup(now_tick);
      return;
    }
  }
  else
  {
    stall_start_tick = 0U;
  }

  maze_simple_drive_straight(now_tick);
}

static uint8_t ManualTurnTask(uint32_t now_tick)
{
  if (Manual_Turn_Cancel_Request != 0U)
  {
    Manual_Turn_Cancel_Request = 0U;
    Manual_Turn_Left_Request = 0U;
    Manual_Turn_Right_Request = 0U;
    manual_turn_active = 0U;
    manual_turn_dir = 0U;
    manual_turn_brake_dir = 0U;
    manual_turn_brake_until_tick = 0U;
    manual_turn_settle_until_tick = 0U;
    MazeDrive(0.0f, 0.0f, now_tick);
    return 1U;
  }

  if (manual_turn_brake_until_tick != 0U)
  {
    if ((int32_t)(now_tick - manual_turn_brake_until_tick) < 0)
    {
      if (manual_turn_brake_dir == 1U)
      {
#if MAZE_RIGHT_MOTOR_REVERSED
        MazeDrive(MANUAL_TURN_COUNTER_BRAKE_PWM,
                  -MANUAL_TURN_COUNTER_BRAKE_PWM,
                  now_tick);
#else
        MazeDrive(-MANUAL_TURN_COUNTER_BRAKE_PWM,
                  MANUAL_TURN_COUNTER_BRAKE_PWM,
                  now_tick);
#endif
      }
      else if (manual_turn_brake_dir == 2U)
      {
#if MAZE_RIGHT_MOTOR_REVERSED
        MazeDrive(-MANUAL_TURN_COUNTER_BRAKE_PWM,
                  MANUAL_TURN_COUNTER_BRAKE_PWM,
                  now_tick);
#else
        MazeDrive(MANUAL_TURN_COUNTER_BRAKE_PWM,
                  -MANUAL_TURN_COUNTER_BRAKE_PWM,
                  now_tick);
#endif
      }
      else
      {
        MazeDrive(0.0f, 0.0f, now_tick);
      }
      return 1U;
    }

    manual_turn_brake_dir = 0U;
    manual_turn_brake_until_tick = 0U;
    manual_turn_settle_until_tick = now_tick + MANUAL_TURN_SETTLE_MS;
  }

  if (manual_turn_settle_until_tick != 0U)
  {
    if ((int32_t)(now_tick - manual_turn_settle_until_tick) < 0)
    {
      MazeDrive(0.0f, 0.0f, now_tick);
      return 1U;
    }
    manual_turn_settle_until_tick = 0U;
  }

  if (manual_turn_active == 0U)
  {
    uint8_t req_dir = 0U; /* 1:right, 2:left */

    if (Manual_Turn_Right_Request != 0U)
    {
      req_dir = 1U;
    }
    else if (Manual_Turn_Left_Request != 0U)
    {
      req_dir = 2U;
    }

    if (req_dir == 0U)
    {
      return 0U;
    }

    Manual_Turn_Right_Request = 0U;
    Manual_Turn_Left_Request = 0U;
    manual_turn_active = 1U;
    manual_turn_dir = req_dir;
    manual_turn_start_tick = now_tick;
    manual_turn_start_yaw_valid = 0U;
#if ODOM_OGM_ENABLE
    manual_turn_start_theta_rad = odom_theta_rad;
#else
    manual_turn_start_theta_rad = 0.0f;
#endif
    if (imu_yaw_usable_for_heading() != 0U)
    {
      manual_turn_start_yaw_rad = deg_to_rad(yaw);
      manual_turn_start_yaw_valid = 1U;
    }
  }

  if (manual_turn_active != 0U)
  {
    float turned_rad = 0.0f;
    float turn_target_rad = ((manual_turn_dir == 1U) ? MANUAL_TURN_TARGET_RIGHT_RAD
                                                     : MANUAL_TURN_TARGET_LEFT_RAD) -
                            MANUAL_TURN_FINISH_MARGIN_RAD;
    uint32_t elapsed_ms = now_tick - manual_turn_start_tick;

#if ODOM_OGM_ENABLE
    turned_rad = fabsf(wrap_pi(odom_theta_rad - manual_turn_start_theta_rad));
#endif
    if (manual_turn_start_yaw_valid != 0U &&
        imu_yaw_usable_for_heading() != 0U)
    {
      float yaw_now_rad = deg_to_rad(yaw);
      turned_rad = fabsf(wrap_pi(yaw_now_rad - manual_turn_start_yaw_rad));
    }

    if ((elapsed_ms > MANUAL_TURN_MIN_MS && turned_rad >= turn_target_rad) ||
        (elapsed_ms > MANUAL_TURN_TIMEOUT_MS))
    {
      uint8_t finished_dir = manual_turn_dir;

      manual_turn_active = 0U;
      manual_turn_dir = 0U;
      manual_turn_start_yaw_valid = 0U;
      manual_turn_brake_dir = finished_dir;
      manual_turn_brake_until_tick = now_tick + MANUAL_TURN_COUNTER_BRAKE_MS;
      manual_turn_settle_until_tick = 0U;
      return 1U;
    }

    if (manual_turn_dir == 1U)
    {
#if MAZE_RIGHT_MOTOR_REVERSED
      MazeDrive(-REMOTE_PWM_TURN, REMOTE_PWM_TURN, now_tick);
#else
      MazeDrive(REMOTE_PWM_TURN, -REMOTE_PWM_TURN, now_tick);
#endif
    }
    else
    {
#if MAZE_RIGHT_MOTOR_REVERSED
      MazeDrive(REMOTE_PWM_TURN, -REMOTE_PWM_TURN, now_tick);
#else
      MazeDrive(-REMOTE_PWM_TURN, REMOTE_PWM_TURN, now_tick);
#endif
    }
    return 1U;
  }

  return 0U;
}

void MazeControlTask(uint32_t now_tick)
{
#if MAZE_SIMPLE_RULE_ENABLE
  MazeSimpleControlTask(now_tick);
  return;
#else
  static uint32_t stall_start_tick = 0U;
  static uint32_t escape_state_tick = 0U;
  static uint8_t escape_state = 0U; /* 0: none, 1: back, 2: turn */
  static uint8_t escape_turn_mode = 0U;  /* 0:left, 1:right, 2:back */
  static uint8_t front_lock_dir = 0U;    /* 0:none, 1:right, 2:left, 3:back */
  static uint32_t front_lock_tick = 0U;
  static uint8_t front_block_latched = 0U;
#if ODOM_OGM_ENABLE
  static float loop_anchor_x_m = 0.0f;
  static float loop_anchor_y_m = 0.0f;
  static float loop_turn_accum_rad = 0.0f;
  static float loop_last_theta_rad = 0.0f;
  static uint32_t loop_anchor_tick = 0U;
  static uint8_t loop_theta_valid = 0U;
#endif
  float base_pwm = MAZE_PWM_BASE;
  float drive_base_pwm = base_pwm;
  float front_meas = fordist;
  float right_meas = rightdist;
  float turn_pwm = MAZE_PWM_TURN;
  float right_err = 0.0f;
  float corr = 0.0f;
  float sync_err = 0.0f;
  float sync_corr_pwm = 0.0f;
  float left_pwm = base_pwm;
  float right_pwm = base_pwm;
  uint8_t right_open = 0U;
  uint8_t right_front_open = 0U;
  uint8_t front_emergency = 0U;
  uint8_t front_block = 0U;
  uint8_t left_open = 0U;
  uint8_t right_clear_probe = 0U;
  uint8_t left_clear_probe = 0U;
  uint8_t intersection_right_candidate = 0U;
  uint8_t force_dir_active = 0U;
  uint8_t uturn_block_active = 0U;
  float motion_pulse_abs = fabsf(fdb_A) + fabsf(fdb_B);

  if (maze_force_dir_until_tick != 0U &&
      (int32_t)(now_tick - maze_force_dir_until_tick) >= 0)
  {
    maze_force_dir = 0U;
    maze_force_dir_until_tick = 0U;
  }
  force_dir_active = (maze_force_dir != 0U && maze_force_dir_until_tick != 0U) ? 1U : 0U;
  if (maze_uturn_block_until_tick != 0U &&
      (int32_t)(now_tick - maze_uturn_block_until_tick) >= 0)
  {
    maze_uturn_block_until_tick = 0U;
  }
  uturn_block_active = (maze_uturn_block_until_tick != 0U) ? 1U : 0U;

  if (escape_state != 0U)
  {
    if (escape_state == 1U)
    {
      if ((now_tick - escape_state_tick) < MAZE_ESCAPE_BACK_MS)
      {
        MazeDrive(-MAZE_PWM_BASE, -MAZE_PWM_BASE, now_tick);
        return;
      }
      escape_state = 2U;
      escape_state_tick = now_tick;
    }

    {
      uint32_t escape_turn_ms = (escape_turn_mode == 2U) ? MAZE_TURN_BACK_MS : MAZE_ESCAPE_TURN_MS;
      if ((now_tick - escape_state_tick) < escape_turn_ms)
      {
        if (escape_turn_mode == 1U)
        {
#if MAZE_RIGHT_MOTOR_REVERSED
          MazeDrive(-MAZE_PWM_TURN, MAZE_PWM_TURN, now_tick);
#else
          MazeDrive(MAZE_PWM_TURN, -MAZE_PWM_TURN, now_tick);
#endif
        }
        else
        {
#if MAZE_RIGHT_MOTOR_REVERSED
          MazeDrive(MAZE_PWM_TURN, -MAZE_PWM_TURN, now_tick);
#else
          MazeDrive(-MAZE_PWM_TURN, MAZE_PWM_TURN, now_tick);
#endif
        }
        return;
      }
    }

    escape_state = 0U;
    stall_start_tick = 0U;
    maze_have_right_wall = 0U;
    if (escape_turn_mode == 2U)
    {
      maze_uturn_block_until_tick = now_tick + MAZE_UTURN_COOLDOWN_MS;
      maze_post_uturn_commit_until_tick = now_tick + MAZE_POST_UTURN_COMMIT_MS;
    }
    maze_enter_state(MAZE_STATE_FOLLOW, now_tick);
    maze_last_decide_tick = now_tick;
  }

  if (rightfrontdist > 0.0f && right_meas <= 0.0f)
  {
    right_meas = rightfrontdist;
  }
  else if (rightfrontdist > 0.0f && right_meas > 0.0f)
  {
    right_meas = (right_meas * 0.7f) + (rightfrontdist * 0.3f);
  }

  if (front_meas <= 0.0f && rightfrontdist > 0.0f)
  {
    front_meas = rightfrontdist;
  }
  else if (front_meas > 0.0f &&
           front_meas < MAZE_FRONT_USE_RIGHTFRONT_M &&
           rightfrontdist > 0.0f &&
           rightfrontdist < front_meas)
  {
    /* Only let right-front override front when already very close ahead. */
    front_meas = rightfrontdist;
  }

  if (front_meas > 0.0f && front_meas < MAZE_TURN_NEAR_M)
  {
    turn_pwm = MAZE_TURN_PWM_NEAR;
  }

  right_clear_probe = (rightdist > MAZE_RIGHT_OPEN_STRICT_M &&
                       rightfrontdist > MAZE_RIGHT_FRONT_OPEN_STRICT_M) ? 1U : 0U;
  left_clear_probe = (leftdist > MAZE_LEFT_OPEN_M) ? 1U : 0U;

  if (front_meas == 0.0f || front_meas > MAZE_FRONT_LOCK_RELEASE_M)
  {
    front_lock_dir = 0U;
  }

  if (front_meas > 0.0f && front_meas < MAZE_FRONT_BLOCK_M)
  {
    front_block_latched = 1U;
  }
  else if (front_meas == 0.0f || front_meas > MAZE_FRONT_BLOCK_RELEASE_M)
  {
    front_block_latched = 0U;
  }

  if (fordist <= 0.0f && rightdist <= 0.0f && rightfrontdist <= 0.0f && leftdist <= 0.0f)
  {
    stall_start_tick = 0U;
    MazeDrive(0, 0, now_tick);
    return;
  }

  if (front_meas > 0.0f && front_meas < MAZE_NEAR_SLOW_M)
  {
    drive_base_pwm = base_pwm * MAZE_NEAR_SLOW_SCALE;
  }

  if (right_meas > 0.0f && right_meas < MAZE_RIGHT_WALL_CAPTURE_M)
  {
    maze_have_right_wall = 1U;
  }
  else if (right_meas > MAZE_RIGHT_WALL_LOST_M)
  {
    maze_have_right_wall = 0U;
  }

  if (front_meas > 0.0f && front_meas < MAZE_FRONT_PANIC_M)
  {
    stall_start_tick = 0U;
    if (right_clear_probe != 0U)
    {
      escape_turn_mode = 1U;
    }
    else if (left_clear_probe != 0U)
    {
      escape_turn_mode = 0U;
    }
    else
    {
      float right_side_m = (rightfrontdist > rightdist) ? rightfrontdist : rightdist;
      float left_side_m = leftdist;
      escape_turn_mode = (right_side_m > left_side_m) ? 1U : 0U;
    }
    escape_state = 1U;
    escape_state_tick = now_tick;
    maze_map_pause_active = 0U;
    maze_map_pace_tick = now_tick;
    maze_enter_state(MAZE_STATE_FOLLOW, now_tick);
    maze_last_decide_tick = now_tick;
    MazeDrive(-MAZE_PWM_BASE, -MAZE_PWM_BASE, now_tick);
    return;
  }

  if (((fabsf(maze_prev_left_cmd) > MAZE_STUCK_CMD_PWM) ||
       (fabsf(maze_prev_right_cmd) > MAZE_STUCK_CMD_PWM)) &&
      motion_pulse_abs < MAZE_STUCK_PULSE_EPS &&
      ((front_meas > 0.0f && front_meas < (MAZE_FRONT_BLOCK_M + 0.10f)) ||
       (rightdist > 0.0f && rightdist < MAZE_RIGHT_DANGER_M) ||
       (rightfrontdist > 0.0f && rightfrontdist < MAZE_RIGHT_FRONT_DANGER_M) ||
       (leftdist > 0.0f && leftdist < MAZE_LEFT_DANGER_M) ||
       (leftfrontdist > 0.0f && leftfrontdist < MAZE_LEFT_FRONT_DANGER_M)))
  {
    if (stall_start_tick == 0U)
    {
      stall_start_tick = now_tick;
    }
    else if ((now_tick - stall_start_tick) >= MAZE_STUCK_DETECT_MS)
    {
      stall_start_tick = 0U;
      if (right_clear_probe != 0U)
      {
        escape_turn_mode = 1U;
      }
      else if (left_clear_probe != 0U)
      {
        escape_turn_mode = 0U;
      }
      else
      {
        float right_side_m = (rightfrontdist > rightdist) ? rightfrontdist : rightdist;
        float left_side_m = leftdist;
        escape_turn_mode = (right_side_m > left_side_m) ? 1U : 0U;
      }
      escape_state = 1U;
      escape_state_tick = now_tick;
      maze_map_pause_active = 0U;
      maze_map_pace_tick = now_tick;
      maze_enter_state(MAZE_STATE_FOLLOW, now_tick);
      maze_last_decide_tick = now_tick;
      MazeDrive(-MAZE_PWM_BASE, -MAZE_PWM_BASE, now_tick);
      return;
    }
  }
  else
  {
    stall_start_tick = 0U;
  }

#if ODOM_OGM_ENABLE
  if (loop_theta_valid == 0U)
  {
    loop_last_theta_rad = odom_theta_rad;
    loop_theta_valid = 1U;
  }
  else
  {
    loop_turn_accum_rad += fabsf(wrap_pi(odom_theta_rad - loop_last_theta_rad));
    loop_last_theta_rad = odom_theta_rad;
  }

  if (loop_anchor_tick == 0U)
  {
    loop_anchor_x_m = odom_x_m;
    loop_anchor_y_m = odom_y_m;
    loop_anchor_tick = now_tick;
  }
  else if ((now_tick - loop_anchor_tick) >= MAZE_LOOP_WINDOW_MS)
  {
    float dx = odom_x_m - loop_anchor_x_m;
    float dy = odom_y_m - loop_anchor_y_m;
    float progress_m = sqrtf((dx * dx) + (dy * dy));

    if (progress_m < MAZE_LOOP_MIN_PROGRESS_M &&
        loop_turn_accum_rad > MAZE_LOOP_MIN_TURN_RAD)
    {
      if (left_clear_probe != 0U)
      {
        maze_force_dir = 2U;
      }
      else if (right_clear_probe != 0U)
      {
        maze_force_dir = 1U;
      }
      else
      {
        maze_force_dir = 2U;
      }
      maze_force_dir_until_tick = now_tick + MAZE_FORCE_DIR_HOLD_MS;
      force_dir_active = 1U;
    }

    loop_anchor_x_m = odom_x_m;
    loop_anchor_y_m = odom_y_m;
    loop_turn_accum_rad = 0.0f;
    loop_anchor_tick = now_tick;
  }
#endif

  if (maze_state == MAZE_STATE_PRETURN_STOP)
  {
    if (maze_pending_turn_valid == 0U)
    {
      maze_enter_state(MAZE_STATE_FOLLOW, now_tick);
      maze_last_decide_tick = now_tick;
      MazeDrive(0.0f, 0.0f, now_tick);
      return;
    }

    if ((now_tick - maze_state_tick) < MAZE_INTERSECTION_STOP_MS)
    {
      MazeDrive(0.0f, 0.0f, now_tick);
      return;
    }

    {
      maze_state_t target_state = maze_pending_turn_state;
      maze_enter_state(target_state, now_tick);
      return;
    }
  }

  if (maze_state == MAZE_STATE_TURN_RIGHT ||
      maze_state == MAZE_STATE_TURN_LEFT ||
      maze_state == MAZE_STATE_TURN_BACK)
  {
    uint32_t elapsed_ms = (now_tick - maze_state_tick);
    uint32_t turn_ms = maze_turn_duration_ms(maze_state);
    uint32_t turn_max_ms = turn_ms + MAZE_TURN_MAX_EXTRA_MS;
    float turn_target_rad = maze_turn_target_rad(maze_state);
    float turned_rad = 0.0f;
    uint8_t turn_done_by_angle = 0U;
    uint8_t use_angle_gate = 0U;
    uint8_t keep_turning = 0U;

    if (maze_turn_start_valid != 0U)
    {
#if ODOM_OGM_ENABLE
      float turned_odom = fabsf(wrap_pi(odom_theta_rad - maze_turn_start_theta_rad));
      turned_rad = turned_odom;
      use_angle_gate = 1U;

      if (maze_turn_start_yaw_valid != 0U &&
          imu_yaw_usable_for_heading() != 0U)
      {
        float yaw_now_rad = deg_to_rad(yaw);
        float turned_imu = fabsf(wrap_pi(yaw_now_rad - maze_turn_start_yaw_rad));
        turned_rad = turned_imu;
      }

      if (turned_rad >= turn_target_rad)
      {
        turn_done_by_angle = 1U;
      }
#endif
    }

    if (turn_max_ms < turn_ms)
    {
      turn_max_ms = turn_ms;
    }

    if (elapsed_ms < MAZE_TURN_MIN_MS)
    {
      keep_turning = 1U;
    }
    else if (use_angle_gate != 0U)
    {
      if (turn_done_by_angle == 0U && elapsed_ms < turn_max_ms)
      {
        keep_turning = 1U;
      }
    }
    else if (elapsed_ms < turn_ms)
    {
      keep_turning = 1U;
    }

    if (keep_turning != 0U)
    {
      float turn_cmd_pwm = turn_pwm;
      if (use_angle_gate != 0U &&
          turn_target_rad > 0.2f &&
          turned_rad > (turn_target_rad * MAZE_TURN_SLOWDOWN_RATIO))
      {
        turn_cmd_pwm = MAZE_TURN_PWM_FINAL;
      }

      if (maze_state == MAZE_STATE_TURN_RIGHT)
      {
#if MAZE_RIGHT_MOTOR_REVERSED
        MazeDrive(-turn_cmd_pwm, turn_cmd_pwm, now_tick);
#else
        MazeDrive(turn_cmd_pwm, -turn_cmd_pwm, now_tick);
#endif
      }
      else if (maze_state == MAZE_STATE_TURN_LEFT)
      {
#if MAZE_RIGHT_MOTOR_REVERSED
        MazeDrive(turn_cmd_pwm, -turn_cmd_pwm, now_tick);
#else
        MazeDrive(-turn_cmd_pwm, turn_cmd_pwm, now_tick);
#endif
      }
      else
      {
#if MAZE_RIGHT_MOTOR_REVERSED
        MazeDrive(turn_cmd_pwm, -turn_cmd_pwm, now_tick);
#else
        MazeDrive(-turn_cmd_pwm, turn_cmd_pwm, now_tick);
#endif
      }
      return;
    }

    if (maze_state == MAZE_STATE_TURN_BACK)
    {
      maze_uturn_block_until_tick = now_tick + MAZE_UTURN_COOLDOWN_MS;
      maze_post_uturn_commit_until_tick = now_tick + MAZE_POST_UTURN_COMMIT_MS;
    }
    maze_enter_state(MAZE_STATE_POST_TURN_FORWARD, now_tick);
  }

  if (maze_state == MAZE_STATE_POST_TURN_FORWARD)
  {
    if ((now_tick - maze_state_tick) < MAZE_POST_TURN_FORWARD_MS)
    {
      MazeDrive(drive_base_pwm, drive_base_pwm, now_tick);
      return;
    }
    maze_enter_state(MAZE_STATE_FOLLOW, now_tick);
  }

  if (maze_state == MAZE_STATE_FOLLOW)
  {
    if (rightdist > 0.0f)
    {
      right_open = (rightdist > MAZE_RIGHT_TURN_SIDE_CLEAR_M) ? 1U : 0U;
    }
    else
    {
      right_open = (right_meas > MAZE_RIGHT_OPEN_M) ? 1U : 0U;
    }
    right_front_open = (rightfrontdist > MAZE_RIGHT_FRONT_OPEN_M) ? 1U : 0U;
    front_emergency = (front_meas > 0.0f && front_meas < MAZE_FRONT_EMERGENCY_M) ? 1U : 0U;
    front_block = front_block_latched;
    left_open = (leftdist > MAZE_LEFT_OPEN_M) ? 1U : 0U;
    if (front_emergency == 0U && front_block == 0U)
    {
      maze_deadend_confirm_tick = 0U;
    }

    if (maze_post_uturn_commit_until_tick != 0U)
    {
      if ((int32_t)(now_tick - maze_post_uturn_commit_until_tick) < 0 &&
          (front_meas == 0.0f || front_meas > (MAZE_FRONT_PANIC_M + 0.08f)))
      {
        MazeDrive(drive_base_pwm, drive_base_pwm, now_tick);
        return;
      }
      maze_post_uturn_commit_until_tick = 0U;
    }

#if MAZE_MAP_PACE_ENABLE
    if (maze_map_pause_active != 0U)
    {
      if ((now_tick - maze_map_pace_tick) < MAZE_MAP_STOP_MS)
      {
        MazeDrive(0.0f, 0.0f, now_tick);
        return;
      }
      maze_map_pause_active = 0U;
      maze_map_pace_tick = now_tick;
      maze_last_decide_tick = now_tick - MAZE_DECIDE_INTERVAL_MS;
    }
    else if (front_emergency == 0U &&
             front_block == 0U &&
             (front_meas == 0.0f || front_meas > MAZE_MAP_FRONT_SAFE_M) &&
             (now_tick - maze_map_pace_tick) >= MAZE_MAP_RUN_MS)
    {
      maze_map_pause_active = 1U;
      maze_map_pace_tick = now_tick;
      MazeDrive(0.0f, 0.0f, now_tick);
      return;
    }
#endif

    sync_err = fabsf(fdb_A) - fabsf(fdb_B);
    sync_corr_pwm = clampf(sync_err * MAZE_SYNC_KP_PWM_PER_PULSE,
                           -MAZE_SYNC_CORR_PWM_MAX,
                           MAZE_SYNC_CORR_PWM_MAX);

    if (front_block == 0U &&
        rightdist > 0.0f && rightdist < MAZE_RIGHT_DANGER_M)
    {
      float danger_bias_pwm = MAZE_RIGHT_DANGER_BIAS_PWM;
      if (rightfrontdist > 0.0f && rightfrontdist < MAZE_RIGHT_FRONT_DANGER_M)
      {
        danger_bias_pwm *= 1.45f;
      }
      left_pwm = clampf(drive_base_pwm - danger_bias_pwm, MAZE_PWM_MIN, MAZE_PWM_MAX);
      right_pwm = clampf(drive_base_pwm + danger_bias_pwm, MAZE_PWM_MIN, MAZE_PWM_MAX);
      left_pwm = clampf(left_pwm - sync_corr_pwm, MAZE_PWM_MIN, MAZE_PWM_MAX);
      right_pwm = clampf(right_pwm + sync_corr_pwm, MAZE_PWM_MIN, MAZE_PWM_MAX);
      MazeDrive(left_pwm, right_pwm, now_tick);
      return;
    }

    if (front_emergency != 0U || front_block != 0U)
    {
      uint8_t right_turn_available = (right_clear_probe != 0U) ? 1U : 0U;
      uint8_t left_turn_available = (left_open != 0U) ? 1U : 0U;
      uint8_t rule_dir = 0U;

      /* User requested rule set:
         1) front open + both side walls -> straight (handled outside this block)
         2) front wall + left wall -> turn right
         3) front wall + right wall -> turn left
         4) front wall + both sides open -> turn left
         deadend (front+left+right walls) -> reverse then 90 turn. */
      if (left_turn_available == 0U && right_turn_available == 0U)
      {
        rule_dir = 3U;
      }
      else if (left_turn_available == 0U && right_turn_available != 0U)
      {
        rule_dir = 1U;
      }
      else
      {
        rule_dir = 2U;
      }

      if (maze_rule_turn_candidate_dir != rule_dir)
      {
        maze_rule_turn_candidate_dir = rule_dir;
        maze_rule_turn_candidate_tick = now_tick;
      }

      if ((now_tick - maze_rule_turn_candidate_tick) < MAZE_TURN_RULE_CONFIRM_MS)
      {
        MazeDrive(0.0f, 0.0f, now_tick);
        return;
      }

      maze_rule_turn_candidate_dir = 0U;
      maze_rule_turn_candidate_tick = 0U;
      maze_last_decide_tick = now_tick;

      if (rule_dir == 1U)
      {
        maze_schedule_turn(MAZE_STATE_TURN_RIGHT, now_tick);
      }
      else if (rule_dir == 2U)
      {
        maze_schedule_turn(MAZE_STATE_TURN_LEFT, now_tick);
      }
      else
      {
        float right_side_m = (rightfrontdist > rightdist) ? rightfrontdist : rightdist;
        float left_side_m = leftdist;
        escape_turn_mode = (right_side_m > left_side_m + 0.03f) ? 1U : 0U;
        escape_state = 1U;
        escape_state_tick = now_tick;
        maze_map_pause_active = 0U;
        maze_map_pace_tick = now_tick;
      }
      return;
    }
    else
    {
      maze_rule_turn_candidate_dir = 0U;
      maze_rule_turn_candidate_tick = 0U;
    }

    if (maze_have_right_wall != 0U && right_meas > 0.0f)
    {
      right_err = right_meas - MAZE_RIGHT_TARGET_M;
      corr = clampf(right_err * MAZE_KP_PWM_PER_M, -MAZE_CORR_PWM_MAX, MAZE_CORR_PWM_MAX);
      left_pwm = clampf(drive_base_pwm + corr, MAZE_PWM_MIN, MAZE_PWM_MAX);
      right_pwm = clampf(drive_base_pwm - corr, MAZE_PWM_MIN, MAZE_PWM_MAX);
      left_pwm = clampf(left_pwm - sync_corr_pwm, MAZE_PWM_MIN, MAZE_PWM_MAX);
      right_pwm = clampf(right_pwm + sync_corr_pwm, MAZE_PWM_MIN, MAZE_PWM_MAX);
      MazeDrive(left_pwm, right_pwm, now_tick);
    }
    else
    {
      /* In open space, keep forward crawl and wait to capture a right wall. */
      left_pwm = clampf(drive_base_pwm - sync_corr_pwm, MAZE_PWM_MIN, MAZE_PWM_MAX);
      right_pwm = clampf(drive_base_pwm + sync_corr_pwm, MAZE_PWM_MIN, MAZE_PWM_MAX);
      MazeDrive(left_pwm, right_pwm, now_tick);
    }
    return;
  }

  MazeDrive(0, 0, now_tick);
#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim4)
  {
    if (control_tick_pending < 1000U)
    {
      control_tick_pending++;
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

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
  ssd1306_init(&hi2c1);
  MPU_Init();
  imu_last_reinit_tick = HAL_GetTick();
  if (mpu_dmp_init() == 0U)
  {
    imu_data_valid = 1U;
    imu_last_ok_tick = HAL_GetTick();
  }
  else
  {
    imu_data_valid = 0U;
    imu_last_ok_tick = 0U;
  }

  HAL_TIM_Base_Start_IT(&htim4);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  HAL_UART_Receive_IT(&huart1, rx_buf, 1);
  Lidar_InitSequence();
  lidar_ready = 0U;
  lidar_ready_stable_count = 0U;
  lidar_last_nonzero_tick = HAL_GetTick();

  Load(0, 0);
  pid_setup(&pid_A, pid_k_A, MAX_OUT, MAX_IOUT);
  pid_setup(&pid_B, pid_k_B, MAX_OUT, MAX_IOUT);
  drive_pid_reset_all();
#if ODOM_OGM_ENABLE
  Odom_Reset();
#if OGM_RUNTIME_ENABLE
  OGM_Reset();
#endif
#endif
  /* USER CODE END 2 */

  while (1)
  {
    uint8_t run_control_tick = 0U;

    __disable_irq();
    if (control_tick_pending > 0U)
    {
      control_tick_pending--;
      run_control_tick = 1U;
    }
    __enable_irq();

    if (run_control_tick != 0U)
    {
      ReadSensor();
      Control();
    }

#if ODOM_OGM_ENABLE
    if (lidar_is_ready_for_runtime() != 0U)
    {
      Odom_UpdatePeriodic();
    }
    else
    {
      __disable_irq();
      odom_pulse_accum_left = 0;
      odom_pulse_accum_right = 0;
      __enable_irq();
    }
#endif
    Send_Data_EverySecond();
    Lidar_ForwardTask();
    Lidar_HandleReinitRequest();
#if ODOM_OGM_ENABLE
#if OGM_RUNTIME_ENABLE
    if (lidar_is_ready_for_runtime() != 0U && Maze_Enable != 0U
#if MAZE_TOPO_ENABLE
        && maze_return_align_pending == 0U
#endif
       )
    {
      OGM_UpdatePeriodic();
    }
#endif
#endif
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void ReadSensor(void)
{
  uint32_t now_tick = HAL_GetTick();
  uint8_t imu_rc = 0U;

  fdb_A = Encoder_GetPulseDelta(&htim3, &lastCount_A);
  fdb_B = Encoder_GetPulseDelta(&htim2, &lastCount_B);

  float pulse_avg = (fdb_A + fdb_B) * 0.5f;
  pulse_sum += (int32_t)pulse_avg;
  pulse_sum_A += (int32_t)fdb_A;
  pulse_sum_B += (int32_t)fdb_B;
#if ODOM_OGM_ENABLE
  if (lidar_is_ready_for_runtime() != 0U)
  {
    odom_pulse_accum_left += (int32_t)fdb_A;
    odom_pulse_accum_right += (int32_t)fdb_B;
  }
  else
  {
    odom_pulse_accum_left = 0;
    odom_pulse_accum_right = 0;
  }
#endif

  imu_rc = mpu_dmp_get_data(&pitch, &roll, &yaw);
  if (imu_rc == 0U)
  {
    imu_data_valid = 1U;
    imu_last_ok_tick = now_tick;
  }
  else if ((now_tick - imu_last_ok_tick) > IMU_STALE_MS)
  {
    imu_data_valid = 0U;
    if ((now_tick - imu_last_reinit_tick) >= IMU_REINIT_RETRY_MS)
    {
      imu_last_reinit_tick = now_tick;
      MPU_Init();
      if (mpu_dmp_init() == 0U)
      {
        imu_data_valid = 1U;
        imu_last_ok_tick = now_tick;
      }
    }
  }

  RefreshLidarDistances(now_tick);
}

void Control(void)
{
  uint32_t now_tick = HAL_GetTick();

  Calibration_HandleCommand();
  OGM_HandleCommand();
  MazeTopo_HandleCommand();
  Lidar_HandleScanDumpCommand();
  if (Calibration_Task(now_tick) != 0U)
  {
    return;
  }

  if (lidar_is_ready_for_runtime() == 0U)
  {
    control_prev_maze_enable = 0U;
    manual_turn_active = 0U;
    manual_turn_dir = 0U;
    manual_turn_brake_dir = 0U;
    manual_turn_brake_until_tick = 0U;
    manual_turn_settle_until_tick = 0U;
    Manual_Turn_Left_Request = 0U;
    Manual_Turn_Right_Request = 0U;
    Manual_Turn_Cancel_Request = 0U;
    maze_simple_state = MAZE_SIMPLE_FOLLOW;
    maze_simple_pending_turn_state = MAZE_SIMPLE_FOLLOW;
    maze_simple_pending_turn_valid = 0U;
    maze_simple_cardinal_ref_valid = 0U;
    maze_simple_cardinal_ref_yaw_rad = 0.0f;
    maze_simple_post_lock_phase = 0U;
    maze_simple_post_lock_tick = 0U;
    maze_simple_start_align_phase = 0U;
    maze_simple_start_align_tick = 0U;
    maze_simple_start_align_preserve_map = 0U;
    maze_simple_prefer_turn_dir = 0;
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
    maze_return_align_pending = 0U;
#endif
    maze_centerline_done_this_stop = 0U;
    maze_centerline_done_after_turn = 0U;
    maze_centerline_reset();
#if ODOM_OGM_ENABLE
    maze_simple_start_valid = 0U;
    maze_simple_backup_start_valid = 0U;
    maze_simple_center_start_valid = 0U;
    maze_simple_pre_turn_start_valid = 0U;
#endif
    maze_enter_state(MAZE_STATE_FOLLOW, now_tick);
    maze_last_decide_tick = now_tick;
    maze_prev_left_cmd = 0.0f;
    maze_prev_right_cmd = 0.0f;
    maze_have_right_wall = 0U;
    maze_force_dir = 0U;
    maze_force_dir_until_tick = 0U;
    maze_map_pause_active = 0U;
    maze_map_pace_tick = now_tick;
    maze_uturn_block_until_tick = 0U;
    maze_post_uturn_commit_until_tick = 0U;
    maze_deadend_confirm_tick = 0U;
    maze_intersection_candidate_tick = 0U;
    maze_rule_turn_candidate_dir = 0U;
    maze_rule_turn_candidate_tick = 0U;
    maze_pending_turn_valid = 0U;
    maze_pending_turn_state = MAZE_STATE_FOLLOW;
    Load(0, 0);
    return;
  }

#if MAZE_TILT_ABORT_ENABLE
  if (Maze_Enable != 0U &&
      imu_is_ready_for_runtime() != 0U &&
      imu_attitude_within(MAZE_TILT_ABORT_ROLL_DEG, MAZE_TILT_ABORT_PITCH_DEG) == 0U)
  {
    maze_abort_autonomous("SAFE:TILT\r\n", now_tick);
    return;
  }
#endif

  if (Maze_Enable != 0U)
  {
    if (control_prev_maze_enable == 0U)
    {
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
      if (maze_return_active == 0U && maze_return_align_pending == 0U)
#endif
      {
#if MAZE_SIMPLE_START_ALIGN_ENABLE
        maze_simple_enter(MAZE_SIMPLE_START_ALIGN, now_tick);
#else
        maze_simple_enter(MAZE_SIMPLE_START_STRAIGHT, now_tick);
#endif
      }
      maze_last_decide_tick = now_tick;
      maze_prev_left_cmd = 0.0f;
      maze_prev_right_cmd = 0.0f;
      maze_have_right_wall = 0U;
      maze_centerline_done_this_stop = 0U;
      maze_centerline_done_after_turn = 0U;
      maze_centerline_reset();
    }
    control_prev_maze_enable = 1U;
    manual_turn_active = 0U;
    manual_turn_dir = 0U;
    manual_turn_brake_dir = 0U;
    manual_turn_brake_until_tick = 0U;
    manual_turn_settle_until_tick = 0U;
    Manual_Turn_Left_Request = 0U;
    Manual_Turn_Right_Request = 0U;
    Manual_Turn_Cancel_Request = 0U;
    MazeControlTask(now_tick);
    return;
  }

  if (control_prev_maze_enable != 0U)
  {
    control_prev_maze_enable = 0U;
    maze_simple_state = MAZE_SIMPLE_FOLLOW;
    maze_simple_pending_turn_state = MAZE_SIMPLE_FOLLOW;
    maze_simple_pending_turn_valid = 0U;
    maze_simple_cardinal_ref_valid = 0U;
    maze_simple_cardinal_ref_yaw_rad = 0.0f;
    maze_simple_post_lock_phase = 0U;
    maze_simple_post_lock_tick = 0U;
    maze_simple_start_align_phase = 0U;
    maze_simple_start_align_tick = 0U;
    maze_simple_start_align_preserve_map = 0U;
    maze_simple_prefer_turn_dir = 0;
#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
    maze_return_align_pending = 0U;
#endif
    maze_centerline_done_this_stop = 0U;
    maze_centerline_done_after_turn = 0U;
    maze_centerline_reset();
#if ODOM_OGM_ENABLE
    maze_simple_start_valid = 0U;
    maze_simple_backup_start_valid = 0U;
    maze_simple_center_start_valid = 0U;
    maze_simple_pre_turn_start_valid = 0U;
#endif
    maze_enter_state(MAZE_STATE_FOLLOW, now_tick);
    maze_last_decide_tick = now_tick;
    maze_prev_left_cmd = 0.0f;
    maze_prev_right_cmd = 0.0f;
    maze_have_right_wall = 0U;
    maze_force_dir = 0U;
    maze_force_dir_until_tick = 0U;
    maze_map_pause_active = 0U;
    maze_map_pace_tick = now_tick;
    maze_uturn_block_until_tick = 0U;
    maze_post_uturn_commit_until_tick = 0U;
    maze_deadend_confirm_tick = 0U;
    maze_intersection_candidate_tick = 0U;
    maze_rule_turn_candidate_dir = 0U;
    maze_rule_turn_candidate_tick = 0U;
    maze_pending_turn_valid = 0U;
    maze_pending_turn_state = MAZE_STATE_FOLLOW;
  }

  if (ManualTurnTask(now_tick) != 0U)
  {
    return;
  }

  if ((Fore == 0) && (Back == 0) && (Left == 0) && (Right == 0))
  {
    MazeDrive(0, 0, now_tick);
  }
  else if ((Fore == 1) && (Back == 0) && (Left == 0) && (Right == 0))
  {
    MazeDrive(REMOTE_PWM_BASE, REMOTE_PWM_BASE, now_tick);
  }
  else if ((Fore == 0) && (Back == 1) && (Left == 0) && (Right == 0))
  {
    MazeDrive(-REMOTE_PWM_BASE, -REMOTE_PWM_BASE, now_tick);
  }
  else if ((Fore == 0) && (Back == 0) && (Left == 1) && (Right == 0))
  {
#if MAZE_RIGHT_MOTOR_REVERSED
    MazeDrive(REMOTE_PWM_TURN, -REMOTE_PWM_TURN, now_tick);
#else
    MazeDrive(-REMOTE_PWM_TURN, REMOTE_PWM_TURN, now_tick);
#endif
  }
  else if ((Fore == 0) && (Back == 0) && (Left == 0) && (Right == 1))
  {
#if MAZE_RIGHT_MOTOR_REVERSED
    MazeDrive(-REMOTE_PWM_TURN, REMOTE_PWM_TURN, now_tick);
#else
    MazeDrive(REMOTE_PWM_TURN, -REMOTE_PWM_TURN, now_tick);
#endif
  }
  else
  {
    MazeDrive(0, 0, now_tick);
  }
}

static int selfdbg_cm(float m)
{
  return (m >= 0.0f) ? (int)((m * 100.0f) + 0.5f) :
                       (int)((m * 100.0f) - 0.5f);
}

void Send_Data_EverySecond(void)
{
  static uint32_t lastTick = 0;
#if ODOM_OGM_ENABLE && OGM_RUNTIME_ENABLE && OGM_MAPDBG_ENABLE
  static uint32_t mapdbg_last_report_update_count = 0U;
#endif
  uint32_t currentTick = HAL_GetTick();

  if (currentTick - lastTick >= 1000)
  {
#if ODOM_OGM_ENABLE && OGM_RUNTIME_ENABLE
    uint16_t map_unknown = 0U;
    uint16_t map_free = 0U;
    uint16_t map_occ = 0U;
#endif
    float elapsed_s = (currentTick - lastTick) / 1000.0f;
    lastTick = currentTick;

    speed_mps = ((float)pulse_sum / COUNTS_PER_METER) / elapsed_s;
    left_speed_mps = ((float)pulse_sum_A / COUNTS_PER_METER) / elapsed_s;
    right_speed_mps = ((float)pulse_sum_B / COUNTS_PER_METER) / elapsed_s;
    pulse_sum = 0;
    pulse_sum_A = 0;
    pulse_sum_B = 0;
#if ODOM_OGM_ENABLE && OGM_RUNTIME_ENABLE
    OGM_GetStats(&map_unknown, &map_free, &map_occ);
#endif

#if ODOM_OGM_ENABLE
#if OGM_RUNTIME_ENABLE
    int n = snprintf(Bluetooth_buf, sizeof(Bluetooth_buf),
                     "RPY:%.2f,%.2f,%.2f D:%.2f,%.2f,%.2f,%.2f D8:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f S:%.2f,%.2f P:%.2f,%.2f,%.2f G:%u,%u,%u M:%u,%u SM:%u CM:%u,%d,%d,%d LR:%u\r\n",
                     roll, pitch, yaw,
                     fordist, rightdist, rightfrontdist, leftdist,
                     fordist, rightfrontdist, rightdist, rightreardist,
                     reardist, leftreardist, leftdist, leftfrontdist,
                     left_speed_mps, right_speed_mps,
                     odom_x_m, odom_y_m, odom_theta_rad,
                     (unsigned)map_unknown, (unsigned)map_free, (unsigned)map_occ,
                     (unsigned)Maze_Enable, (unsigned)maze_state,
                     (unsigned)maze_simple_state,
                     (unsigned)maze_centerline_phase,
                     (int)maze_centerline_dir,
                     (int)(maze_centerline_error_m * 100.0f),
                     (int)(maze_centerline_shift_target_m * 100.0f),
                     (unsigned)lidar_ready);
#else
    int n = snprintf(Bluetooth_buf, sizeof(Bluetooth_buf),
                     "RPY:%.2f,%.2f,%.2f D:%.2f,%.2f,%.2f,%.2f D8:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f S:%.2f,%.2f P:%.2f,%.2f,%.2f M:%u,%u SM:%u CM:%u,%d,%d,%d LR:%u\r\n",
                     roll, pitch, yaw,
                     fordist, rightdist, rightfrontdist, leftdist,
                     fordist, rightfrontdist, rightdist, rightreardist,
                     reardist, leftreardist, leftdist, leftfrontdist,
                     left_speed_mps, right_speed_mps,
                     odom_x_m, odom_y_m, odom_theta_rad,
                     (unsigned)Maze_Enable, (unsigned)maze_state,
                     (unsigned)maze_simple_state,
                     (unsigned)maze_centerline_phase,
                     (int)maze_centerline_dir,
                     (int)(maze_centerline_error_m * 100.0f),
                     (int)(maze_centerline_shift_target_m * 100.0f),
                     (unsigned)lidar_ready);
#endif
#else
    int n = snprintf(Bluetooth_buf, sizeof(Bluetooth_buf),
                     "RPY:%.2f,%.2f,%.2f D:%.2f,%.2f,%.2f,%.2f D8:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f S:%.2f,%.2f M:%u,%u SM:%u CM:%u,%d,%d,%d LR:%u\r\n",
                     roll, pitch, yaw,
                     fordist, rightdist, rightfrontdist, leftdist,
                     fordist, rightfrontdist, rightdist, rightreardist,
                     reardist, leftreardist, leftdist, leftfrontdist,
                     left_speed_mps, right_speed_mps,
                     (unsigned)Maze_Enable, (unsigned)maze_state,
                     (unsigned)maze_simple_state,
                     (unsigned)maze_centerline_phase,
                     (int)maze_centerline_dir,
                     (int)(maze_centerline_error_m * 100.0f),
                     (int)(maze_centerline_shift_target_m * 100.0f),
                     (unsigned)lidar_ready);
#endif
    if (n > 0)
    {
      uint16_t tx_len = (n < (int)sizeof(Bluetooth_buf)) ? (uint16_t)n : (uint16_t)(sizeof(Bluetooth_buf) - 1U);
      (void)HAL_UART_Transmit(&huart1, (uint8_t*)Bluetooth_buf, tx_len, 50);
    }

    n = snprintf(Bluetooth_buf, sizeof(Bluetooth_buf),
                 "DRVDBG:FW=%s SG=%d TR=%.1f YE=%.1f YC=%.1f C=%.0f,%.0f L=%.0f,%.0f PID=%.0f,%.0f F=%.1f,%.1f\r\n",
                 FW_TUNE_TAG,
                 (int)drive_dbg_straight_sign,
                 drive_dbg_straight_trim_pwm,
                 drive_dbg_yaw_err_deg,
                 drive_dbg_yaw_corr_pwm,
                 drive_dbg_left_cmd_pwm,
                 drive_dbg_right_cmd_pwm,
                 drive_dbg_load_left_pwm,
                 drive_dbg_load_right_pwm,
                 pid_corr_pwm_A,
                 pid_corr_pwm_B,
                 fdb_A,
                 fdb_B);
    if (n > 0)
    {
      uint16_t tx_len = (n < (int)sizeof(Bluetooth_buf)) ? (uint16_t)n : (uint16_t)(sizeof(Bluetooth_buf) - 1U);
      (void)HAL_UART_Transmit(&huart1, (uint8_t*)Bluetooth_buf, tx_len, 50);
    }

#if ODOM_OGM_ENABLE && OGM_RUNTIME_ENABLE && OGM_MAPDBG_ENABLE
    uint32_t mapdbg_new_updates = ogm_dbg_scan_update_count - mapdbg_last_report_update_count;
    uint8_t mapdbg_skip = ogm_dbg_scan_skip_reason;
    if (mapdbg_new_updates > 0U)
    {
      mapdbg_skip = OGM_SCAN_SKIP_NONE;
    }
    else if (Maze_Enable == 0U)
    {
      mapdbg_skip = OGM_SCAN_SKIP_IDLE;
    }
    mapdbg_last_report_update_count = ogm_dbg_scan_update_count;
    n = snprintf(Bluetooth_buf, sizeof(Bluetooth_buf),
                 "MAPDBG:USED=%u SKIP=%u RAW=%u FILT=%u FAR=%u MSK=%u AGE=%lu UPD=%lu NEW=%lu\r\n",
                 (unsigned)((mapdbg_new_updates > 0U) ? 1U : 0U),
                 (unsigned)mapdbg_skip,
                 (unsigned)ogm_dbg_scan_raw_valid_bins,
                 (unsigned)ogm_dbg_scan_filtered_bins,
                 (unsigned)ogm_dbg_scan_far_bins,
                 (unsigned)ogm_dbg_scan_masked_count,
                 (unsigned long)((ogm_dbg_scan_tick == 0U) ? 0U : (currentTick - ogm_dbg_scan_tick)),
                 (unsigned long)ogm_dbg_scan_update_count,
                 (unsigned long)mapdbg_new_updates);
    if (n > 0)
    {
      uint16_t tx_len = (n < (int)sizeof(Bluetooth_buf)) ? (uint16_t)n : (uint16_t)(sizeof(Bluetooth_buf) - 1U);
      (void)HAL_UART_Transmit(&huart1, (uint8_t*)Bluetooth_buf, tx_len, 50);
    }
#endif

#if SELFDBG_ENABLE
    {
      uint32_t state_age = currentTick - maze_state_tick;
      uint32_t age_f = (fordist_last_valid_tick == 0U) ? 9999U : (currentTick - fordist_last_valid_tick);
      uint32_t age_rf = (rightfrontdist_last_valid_tick == 0U) ? 9999U : (currentTick - rightfrontdist_last_valid_tick);
      uint32_t age_r = (rightdist_last_valid_tick == 0U) ? 9999U : (currentTick - rightdist_last_valid_tick);
      uint32_t age_rr = (rightreardist_last_valid_tick == 0U) ? 9999U : (currentTick - rightreardist_last_valid_tick);
      uint32_t age_b = (reardist_last_valid_tick == 0U) ? 9999U : (currentTick - reardist_last_valid_tick);
      uint32_t age_lr = (leftreardist_last_valid_tick == 0U) ? 9999U : (currentTick - leftreardist_last_valid_tick);
      uint32_t age_l = (leftdist_last_valid_tick == 0U) ? 9999U : (currentTick - leftdist_last_valid_tick);
      uint32_t age_lf = (leftfrontdist_last_valid_tick == 0U) ? 9999U : (currentTick - leftfrontdist_last_valid_tick);
      uint8_t right_open_self = maze_simple_right_open();
      uint8_t left_open_self = maze_simple_left_open();
      uint8_t front_clear_self = maze_simple_front_clear(fordist);
      uint8_t front_block_self = maze_simple_front_blocked(fordist);
      uint8_t right_probe_self = maze_simple_right_probe_candidate(currentTick, fordist);
      uint8_t right_hard_block_self = maze_simple_right_hard_blocked(currentTick);
      uint8_t wait_timeout_self =
        (Maze_Enable != 0U && state_age >= MAZE_SIMPLE_DECISION_WAIT_MAX_MS) ? 1U : 0U;
      uint8_t map_run_self =
        (Maze_Enable != 0U && maze_simple_state_allows_mapping() != 0U) ? 1U : 0U;
      uint8_t map_stop_self =
        (Maze_Enable != 0U && maze_simple_state_allows_stop_mapping() != 0U) ? 1U : 0U;
      int8_t side_guard_self = maze_simple_manhattan_side_guard(currentTick, fordist);

      if (selfdbg_side_open_suppressed != 0U)
      {
        right_open_self = 0U;
        left_open_self = 0U;
        right_probe_self = 0U;
      }

      n = snprintf(Bluetooth_buf, sizeof(Bluetooth_buf),
                   "SELF:PF=%s SA=%lu AG=%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu OPEN=%u,%u,%u,%u DIAG=%u,%u,%u,%u MAP=%u,%u GU=%d BP=%u BT=%d BE=%u WT=%u TD=%d TP=%d TB=%u PAR=%d,%d,%d,%d,%d,%d,%d,%d,%d,%u SAFE=%d,%d,%u\r\n",
                   MAZE_PROFILE_NAME,
                   (unsigned long)state_age,
                   (unsigned long)age_f,
                   (unsigned long)age_rf,
                   (unsigned long)age_r,
                   (unsigned long)age_rr,
                   (unsigned long)age_b,
                   (unsigned long)age_lr,
                   (unsigned long)age_l,
                   (unsigned long)age_lf,
                   (unsigned)right_open_self,
                   (unsigned)left_open_self,
                   (unsigned)front_clear_self,
                   (unsigned)front_block_self,
                   (unsigned)right_probe_self,
                   (unsigned)right_hard_block_self,
                   (unsigned)selfdbg_side_open_suppressed,
                   (unsigned)lidar_ranges_all_zero(),
                   (unsigned)map_run_self,
                   (unsigned)map_stop_self,
                   (int)side_guard_self,
                   (unsigned)selfdbg_backup_stop_pending,
                   (int)selfdbg_backup_turn_dir,
                   (unsigned)selfdbg_backup_turn_extra_active,
                   (unsigned)wait_timeout_self,
                   (int)selfdbg_topo_decision_dir,
                   (int)selfdbg_topo_pending_dir,
                   (unsigned)selfdbg_topo_block_reason,
                   selfdbg_cm(MAZE_SIMPLE_RIGHT_GAP_ENTRY_M),
                   selfdbg_cm(MAZE_SIMPLE_LEFT_GAP_ENTRY_M),
                   selfdbg_cm(MAZE_SIMPLE_RIGHT_CENTER_FRONT_MIN_M),
                   selfdbg_cm(MAZE_SIMPLE_LEFT_CENTER_FRONT_MIN_M),
                   selfdbg_cm(MAZE_SIMPLE_RIGHT_OPEN_M),
                   selfdbg_cm(MAZE_SIMPLE_LEFT_OPEN_M),
                   selfdbg_cm(MAZE_SIMPLE_APPROACH_FRONT_MIN_M),
                   selfdbg_cm(MAZE_SIMPLE_BACKUP_CENTER_M),
                   selfdbg_cm(MAZE_SIMPLE_FORWARD_SIDE_DANGER_M),
                   (unsigned)(MAZE_SIMPLE_POST_TURN_SIDE_IGNORE_MS / 100U),
                   (int)(MAZE_TILT_ABORT_ROLL_DEG + 0.5f),
                   (int)(MAZE_TILT_ABORT_PITCH_DEG + 0.5f),
                   (unsigned)MAZE_PROFILE);
      if (n > 0)
      {
      uint16_t tx_len = (n < (int)sizeof(Bluetooth_buf)) ? (uint16_t)n : (uint16_t)(sizeof(Bluetooth_buf) - 1U);
      (void)HAL_UART_Transmit(&huart1, (uint8_t*)Bluetooth_buf, tx_len, 50);
    }

    if (selfdbg_backup_align_phase != 0U ||
        selfdbg_topo_block_reason == 4U)
    {
      n = snprintf(Bluetooth_buf, sizeof(Bluetooth_buf),
                   "BALDBG:PH=%u AGE=%lu YE=%.1f REF=%.1f Y=%.1f PWM=%.0f DB=%.1f MAX=%u\r\n",
                   (unsigned)selfdbg_backup_align_phase,
                   (unsigned long)selfdbg_backup_align_age_ms,
                   selfdbg_backup_align_err_deg,
                   selfdbg_backup_align_ref_deg,
                   yaw,
                   selfdbg_backup_align_pwm,
                   MAZE_SIMPLE_BACKUP_ALIGN_DEADBAND_DEG,
                   (unsigned)MAZE_SIMPLE_BACKUP_ALIGN_MAX_MS);
      if (n > 0)
      {
        uint16_t tx_len = (n < (int)sizeof(Bluetooth_buf)) ? (uint16_t)n : (uint16_t)(sizeof(Bluetooth_buf) - 1U);
        (void)HAL_UART_Transmit(&huart1, (uint8_t*)Bluetooth_buf, tx_len, 50);
      }
    }
  }
#endif

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE
    n = snprintf(Bluetooth_buf, sizeof(Bluetooth_buf),
                 "RETDBG:PH=%s PN=%s RA=%c ACT=%u DONE=%u PATH=%u APATH=%u AST=%u AFB=%u REM=%u HOME=%u HD=%u DIR=%u DEC=%c AGE=%lu\r\n",
                 MazeReturn_PhaseName(maze_return_phase),
                 MazeReturn_PlanName(maze_return_plan),
                 MazeReturn_ActionChar(maze_return_last_action),
                 (unsigned)maze_return_active,
                 (unsigned)maze_return_done,
                 (unsigned)maze_topo_path_count,
#if MAZE_ASTAR_ENABLE
                 (unsigned)maze_astar_path_count,
                 (unsigned)maze_return_astar_active,
                 (unsigned)maze_return_astar_fallback,
#else
                 0U,
                 0U,
                 0U,
#endif
                 (unsigned)maze_return_remaining,
                 (unsigned)maze_return_home_pending,
                 (unsigned)maze_return_home_drive_started,
                 (unsigned)maze_return_target_dir,
                 MazeTopo_DecisionChar(maze_return_latched_decision),
                 (unsigned long)((maze_return_phase_tick != 0U) ?
                   (currentTick - maze_return_phase_tick) : 0U));
    if (n > 0)
    {
      uint16_t tx_len = (n < (int)sizeof(Bluetooth_buf)) ? (uint16_t)n : (uint16_t)(sizeof(Bluetooth_buf) - 1U);
      (void)HAL_UART_Transmit(&huart1, (uint8_t*)Bluetooth_buf, tx_len, 50);
    }
#endif

#if ODOM_OGM_ENABLE && MAZE_TOPO_ENABLE && MAZE_ASTAR_ENABLE
    if (maze_return_active != 0U && maze_return_astar_active != 0U)
    {
      maze_astar_debug_t astdbg;
      if (MazeAstar_DebugSnapshot(&astdbg) != 0U)
      {
        n = snprintf(Bluetooth_buf, sizeof(Bluetooth_buf),
                     "ASTDBG:C=%d,%d PR=%u N=%u,%d,%d ND=%u T=%u,%d,%d TD=%u G=%u,%d,%d GD=%u GR=%u H=%u DIR=%u ACT=%c DD=%c DG=%u BLK=%u\r\n",
                     astdbg.cur_x,
                     astdbg.cur_y,
                     (unsigned)astdbg.progress_i,
                     (unsigned)astdbg.nearest_i,
                     astdbg.nearest_x,
                     astdbg.nearest_y,
                     (unsigned)astdbg.nearest_d2,
                     (unsigned)astdbg.target_i,
                     astdbg.target_x,
                     astdbg.target_y,
                     (unsigned)astdbg.target_d2,
                     (unsigned)astdbg.gate_i,
                     astdbg.gate_x,
                     astdbg.gate_y,
                     (unsigned)astdbg.gate_d2,
                     (unsigned)astdbg.gate_ready,
                     (unsigned)astdbg.astar_heading,
                     (unsigned)astdbg.next_dir,
                     MazeTopo_DecisionChar(astdbg.next_decision),
                     MazeTopo_DecisionChar(maze_astar_last_dynamic_decision),
                     (unsigned)maze_astar_last_dynamic_goal,
                     (unsigned)maze_astar_temp_block_count);
      }
      else
      {
        n = snprintf(Bluetooth_buf, sizeof(Bluetooth_buf),
                     "ASTDBG:INVALID\r\n");
      }
      if (n > 0)
      {
        uint16_t tx_len = (n < (int)sizeof(Bluetooth_buf)) ? (uint16_t)n : (uint16_t)(sizeof(Bluetooth_buf) - 1U);
        (void)HAL_UART_Transmit(&huart1, (uint8_t*)Bluetooth_buf, tx_len, 50);
      }
    }
#endif

    ssd1306_display_metrics(fordist,
                            rightdist,
                            rightfrontdist,
                            leftdist,
                            roll,
                            pitch,
                            yaw);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    BlueToothControl(rx_buf[0]);
  }
}
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

#ifdef  USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
