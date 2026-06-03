#include "bluetooth.h"

extern UART_HandleTypeDef huart1;

#define BT_CMD_RETURN_START 0xA0U
#define BT_CMD_RETURN_CLEAR 0xA1U

int connect = 0; /* 1: connected, 0: disconnected */
uint8_t Fore = 0, Back = 0, Left = 0, Right = 0;
uint8_t Maze_Enable = 0;
volatile uint8_t Calib_Start_Request = 0U;
volatile uint8_t Calib_Stop_Request = 0U;
volatile uint8_t Map_Dump_Request = 0U;
volatile uint8_t Map_Clear_Request = 0U;
volatile uint8_t Lidar_Scan_Dump_Request = 0U;
volatile uint8_t Lidar_Reinit_Request = 0U;
volatile uint8_t Topo_Dump_Request = 0U;
volatile uint8_t Topo_Return_Request = 0U;
volatile uint8_t Topo_Return_Clear_Request = 0U;
volatile uint8_t Manual_Turn_Left_Request = 0U;
volatile uint8_t Manual_Turn_Right_Request = 0U;
volatile uint8_t Manual_Turn_Cancel_Request = 0U;
uint8_t rx_buf[2];

static uint8_t bt_hex_nibble(uint8_t data, uint8_t *value)
{
    if (data >= '0' && data <= '9')
    {
        *value = (uint8_t)(data - '0');
        return 1U;
    }
    if (data >= 'A' && data <= 'F')
    {
        *value = (uint8_t)(data - 'A' + 10U);
        return 1U;
    }
    if (data >= 'a' && data <= 'f')
    {
        *value = (uint8_t)(data - 'a' + 10U);
        return 1U;
    }
    if (data <= 0x0FU)
    {
        *value = data;
        return 1U;
    }
    return 0U;
}

void JudgeConnect(void)
{
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_SET)
    {
        connect = 1;
    }
    else
    {
        connect = 0;
    }
}

void BlueToothControl(uint8_t Bluetooth_data)
{
    static uint8_t hex_cmd_state = 0U;
    uint8_t nibble = 0U;

    if (Bluetooth_data == '\r' || Bluetooth_data == '\n')
    {
        hex_cmd_state = 0U;
        HAL_UART_Receive_IT(&huart1, rx_buf, 1);
        return;
    }

    if (Bluetooth_data == 0x00U && hex_cmd_state == 0U)
    {
        HAL_UART_Receive_IT(&huart1, rx_buf, 1);
        return;
    }

    /* Accept text sequences such as "0x06", "10", and "11" sent as ASCII bytes. */
    if (hex_cmd_state == 0U)
    {
        if (Bluetooth_data == '0')
        {
            hex_cmd_state = 1U;
            HAL_UART_Receive_IT(&huart1, rx_buf, 1);
            return;
        }
        else if (Bluetooth_data == '1')
        {
            hex_cmd_state = 4U;
            HAL_UART_Receive_IT(&huart1, rx_buf, 1);
            return;
        }
    }
    else if (hex_cmd_state == 1U)
    {
        if (Bluetooth_data == 'x' || Bluetooth_data == 'X')
        {
            hex_cmd_state = 2U;
            HAL_UART_Receive_IT(&huart1, rx_buf, 1);
            return;
        }
        else if (bt_hex_nibble(Bluetooth_data, &nibble) != 0U)
        {
            Bluetooth_data = nibble;
        }
        hex_cmd_state = 0U;
    }
    else if (hex_cmd_state == 2U)
    {
        if (Bluetooth_data == '0')
        {
            hex_cmd_state = 3U;
            HAL_UART_Receive_IT(&huart1, rx_buf, 1);
            return;
        }
        hex_cmd_state = 0U;
    }
    else if (hex_cmd_state == 3U)
    {
        if (bt_hex_nibble(Bluetooth_data, &nibble) != 0U)
        {
            Bluetooth_data = nibble;
        }
        hex_cmd_state = 0U;
    }
    else if (hex_cmd_state == 4U)
    {
        if (Bluetooth_data == '0')
        {
            Bluetooth_data = BT_CMD_RETURN_START;
        }
        else if (Bluetooth_data == '1')
        {
            Bluetooth_data = BT_CMD_RETURN_CLEAR;
        }
        hex_cmd_state = 0U;
    }

    switch (Bluetooth_data)
    {
        case 0x00: /* stop */
            Maze_Enable = 0;
            Fore = 0; Back = 0; Left = 0; Right = 0;
            Calib_Start_Request = 0U;
            Calib_Stop_Request = 0U;
            Map_Dump_Request = 0U;
            Map_Clear_Request = 0U;
            Manual_Turn_Left_Request = 0U;
            Manual_Turn_Right_Request = 0U;
            Manual_Turn_Cancel_Request = 1U;
            break;
        case 0x01: /* forward */
            Maze_Enable = 0;
            Fore = 1; Back = 0; Left = 0; Right = 0;
            Calib_Start_Request = 0U;
            Calib_Stop_Request = 0U;
            Map_Dump_Request = 0U;
            Map_Clear_Request = 0U;
            Manual_Turn_Left_Request = 0U;
            Manual_Turn_Right_Request = 0U;
            Manual_Turn_Cancel_Request = 1U;
            break;
        case 0x02: /* backward */
            Maze_Enable = 0;
            Fore = 0; Back = 1; Left = 0; Right = 0;
            Calib_Start_Request = 0U;
            Calib_Stop_Request = 0U;
            Map_Dump_Request = 0U;
            Map_Clear_Request = 0U;
            Manual_Turn_Left_Request = 0U;
            Manual_Turn_Right_Request = 0U;
            Manual_Turn_Cancel_Request = 1U;
            break;
        case 0x03: /* turn right (one-shot 90 deg request) */
            Maze_Enable = 0;
            Fore = 0; Back = 0; Left = 0; Right = 0;
            Calib_Start_Request = 0U;
            Calib_Stop_Request = 0U;
            Map_Dump_Request = 0U;
            Map_Clear_Request = 0U;
            Manual_Turn_Right_Request = 1U;
            Manual_Turn_Left_Request = 0U;
            Manual_Turn_Cancel_Request = 0U;
            break;
        case 0x04: /* turn left (one-shot 90 deg request) */
            Maze_Enable = 0;
            Fore = 0; Back = 0; Left = 0; Right = 0;
            Calib_Start_Request = 0U;
            Calib_Stop_Request = 0U;
            Map_Dump_Request = 0U;
            Map_Clear_Request = 0U;
            Manual_Turn_Left_Request = 1U;
            Manual_Turn_Right_Request = 0U;
            Manual_Turn_Cancel_Request = 0U;
            break;
        case 0x05: /* maze mode (right-hand rule) */
        case '5':
            Maze_Enable = 1;
            Fore = 0; Back = 0; Left = 0; Right = 0;
            Calib_Start_Request = 0U;
            Calib_Stop_Request = 0U;
            Map_Dump_Request = 0U;
            Map_Clear_Request = 0U;
            Topo_Return_Request = 0U;
            Topo_Return_Clear_Request = 0U;
            Lidar_Reinit_Request = 1U;
            Manual_Turn_Left_Request = 0U;
            Manual_Turn_Right_Request = 0U;
            Manual_Turn_Cancel_Request = 1U;
            break;
        case 0x06: /* one-key odom calibration start */
        case '6':
        case 'C':
        case 'c':
            Calib_Start_Request = 1U;
            Calib_Stop_Request = 0U;
            if (Maze_Enable == 0U &&
                Fore == 0U && Back == 0U && Left == 0U && Right == 0U)
            {
                Maze_Enable = 0;
                Map_Dump_Request = 0U;
                Map_Clear_Request = 0U;
                Manual_Turn_Left_Request = 0U;
                Manual_Turn_Right_Request = 0U;
                Manual_Turn_Cancel_Request = 1U;
            }
            break;
        case 0x07: /* one-key odom calibration stop/abort (binary cmd only) */
        case '7':
            Maze_Enable = 0;
            Fore = 0; Back = 0; Left = 0; Right = 0;
            Calib_Stop_Request = 1U;
            Calib_Start_Request = 0U;
            Map_Dump_Request = 0U;
            Map_Clear_Request = 0U;
            Manual_Turn_Left_Request = 0U;
            Manual_Turn_Right_Request = 0U;
            Manual_Turn_Cancel_Request = 1U;
            break;
        case 0x08: /* map dump */
        case '8':
        case 'D':
        case 'd':
            Maze_Enable = 0;
            Fore = 0; Back = 0; Left = 0; Right = 0;
            Calib_Start_Request = 0U;
            Calib_Stop_Request = 0U;
            Map_Dump_Request = 1U;
            Manual_Turn_Left_Request = 0U;
            Manual_Turn_Right_Request = 0U;
            Manual_Turn_Cancel_Request = 1U;
            break;
        case 0x09: /* map clear */
        case '9':
        case 'R':
        case 'r':
            Maze_Enable = 0;
            Fore = 0; Back = 0; Left = 0; Right = 0;
            Calib_Start_Request = 0U;
            Calib_Stop_Request = 0U;
            Map_Clear_Request = 1U;
            Manual_Turn_Left_Request = 0U;
            Manual_Turn_Right_Request = 0U;
            Manual_Turn_Cancel_Request = 1U;
            break;
        case 'S': /* 360 scan dump */
        case 's':
            Maze_Enable = 0;
            Fore = 0; Back = 0; Left = 0; Right = 0;
            Calib_Start_Request = 0U;
            Calib_Stop_Request = 0U;
            Lidar_Scan_Dump_Request = 1U;
            Manual_Turn_Left_Request = 0U;
            Manual_Turn_Right_Request = 0U;
            Manual_Turn_Cancel_Request = 1U;
            break;
        case 'T': /* topology dump */
        case 't':
            Maze_Enable = 0;
            Fore = 0; Back = 0; Left = 0; Right = 0;
            Calib_Start_Request = 0U;
            Calib_Stop_Request = 0U;
            Topo_Dump_Request = 1U;
            Manual_Turn_Left_Request = 0U;
            Manual_Turn_Right_Request = 0U;
            Manual_Turn_Cancel_Request = 1U;
            break;
        case BT_CMD_RETURN_START: /* mark exit and start return-to-start replay */
        case 'E':
        case 'e':
        case 'H':
        case 'h':
            Fore = 0; Back = 0; Left = 0; Right = 0;
            Calib_Start_Request = 0U;
            Calib_Stop_Request = 0U;
            Topo_Return_Request = 1U;
            Manual_Turn_Left_Request = 0U;
            Manual_Turn_Right_Request = 0U;
            Manual_Turn_Cancel_Request = 1U;
            break;
        case BT_CMD_RETURN_CLEAR: /* clear return replay state */
        case 'Q':
        case 'q':
            Maze_Enable = 0;
            Fore = 0; Back = 0; Left = 0; Right = 0;
            Calib_Start_Request = 0U;
            Calib_Stop_Request = 0U;
            Topo_Return_Clear_Request = 1U;
            Manual_Turn_Left_Request = 0U;
            Manual_Turn_Right_Request = 0U;
            Manual_Turn_Cancel_Request = 1U;
            break;
        default:
            break;
    }

    HAL_UART_Receive_IT(&huart1, rx_buf, 1);
}
