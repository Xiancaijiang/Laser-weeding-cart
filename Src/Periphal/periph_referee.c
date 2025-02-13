/*
 *  Project      : Infantry_Neptune
 * 
 *  file         : referee_periph.c
 *  Description  : This document contains the data receiving and sending of the referee system
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 10:10:43
 */

#include "periph_referee.h"
#include "periph_remote.h"

#define PACKET_SIZE 30 

UART_HandleTypeDef* Const_Referee_UART_HANDLER          = &huart6;

const uint16_t Const_Referee_TX_BUFF_LEN            = 300;
const uint16_t Const_Referee_RX_BUFF_LEN            = 300;
const uint16_t Const_Referee_REMOTE_OFFLINE_TIME    = 1000;

uint8_t Referee_TxData[Const_Referee_TX_BUFF_LEN];
uint8_t Referee_RxData[Const_Referee_RX_BUFF_LEN];
Referee_RefereeDataTypeDef Referee_RefereeData;

const uint8_t PARSE_FAILED = 0, PARSE_SUCCEEDED = 1;


/********** REFEREE CMD PARSER FUNCTION **********/


uint8_t P_ext_game_status(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_game_status_t *struct_ptr = data_ptr;
    
    referee->game_type = struct_ptr->game_type;
    referee->game_progress = struct_ptr->game_progress;
    referee->stage_remain_time = struct_ptr->stage_remain_time;
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_result(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_game_result_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_robot_HP(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_game_robot_HP_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_dart_status(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_dart_status_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_event_data(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_event_data_t *struct_ptr = data_ptr;
    
    referee->event_type = struct_ptr->event_type;
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_supply_projectile_action(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_supply_projectile_action_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_referee_warning(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_referee_warning_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_dart_remaining_time(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_dart_remaining_time_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_robot_status(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_game_robot_status_t *struct_ptr = data_ptr;
    
    referee->robot_level = struct_ptr->robot_level;
    referee->remain_HP = struct_ptr->remain_HP;
    referee->max_chassis_power = struct_ptr->chassis_power_limit;
    referee->mains_power_gimbal_output = struct_ptr->mains_power_gimbal_output;
    referee->mains_power_chassis_output = struct_ptr->mains_power_chassis_output;//底盘功率上限
    referee->mains_power_shooter_output = struct_ptr->mains_power_shooter_output;
    referee->shooter_heat0_cooling_rate = struct_ptr->shooter_id1_17mm_cooling_rate;
    referee->shooter_heat1_cooling_rate = struct_ptr->shooter_id2_17mm_cooling_rate;
    referee->shooter_heat0_cooling_limit = struct_ptr->shooter_id1_17mm_cooling_limit;
    referee->shooter_heat1_cooling_limit = struct_ptr->shooter_id2_17mm_cooling_limit;
    referee->shooter_heat0_speed_limit = struct_ptr->shooter_id1_17mm_speed_limit;
    referee->shooter_heat1_speed_limit = struct_ptr->shooter_id2_17mm_speed_limit;
    
    if (referee->robot_id != struct_ptr->robot_id) {
        referee->robot_id = struct_ptr->robot_id;
        referee->client_id = Referee_GetClientIDByRobotID(referee->robot_id);
    }
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_power_heat_data(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_power_heat_data_t *struct_ptr = data_ptr;
    referee->chassis_volt = struct_ptr->chassis_volt;
    referee->chassis_current = struct_ptr->chassis_current;
    referee->chassis_power = struct_ptr->chassis_power;
    referee->chassis_power_buffer = struct_ptr->chassis_power_buffer;
    referee->shooter_heat0 = struct_ptr->shooter_heat0;
    referee->shooter_heat1 = struct_ptr->shooter_heat1;
    referee->mobile_shooter_heat2 = struct_ptr->mobile_shooter_heat2;
    
    // Referee_DrawingTimeBaseCallback();
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_game_robot_pos(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_game_robot_pos_t *struct_ptr = data_ptr;
    
    referee->x = struct_ptr->x;
    referee->y = struct_ptr->y;
    referee->z = struct_ptr->z;
    referee->yaw = struct_ptr->yaw;
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_buff(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_buff_t *struct_ptr = data_ptr;
    
    referee->power_rune_buff = struct_ptr->power_rune_buff;
    
    return PARSE_SUCCEEDED;
}

uint8_t P_aerial_robot_energy(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    // aerial_robot_energy_t *struct_ptr = data_ptr;

    return PARSE_SUCCEEDED;
}

uint8_t P_ext_robot_hurt(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_robot_hurt_t *struct_ptr = data_ptr;

    // Hurt Callback
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_shoot_data(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    ext_shoot_data_t *struct_ptr = data_ptr;
	
		referee->bullet_freq = struct_ptr->bullet_freq;
		referee->bullet_speed = struct_ptr->bullet_speed; 
		referee->bullet_type = struct_ptr->bullet_type;
    // Shoot Callback
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_bullet_remaining(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_bullet_remaining_t *struct_ptr = data_ptr;
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_rfid_status(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_rfid_status_t *struct_ptr = data_ptr;
    
    return PARSE_SUCCEEDED;
}

uint8_t P_ext_dart_cmd(Referee_RefereeDataTypeDef* referee, void *data_ptr) {
    //ext_dart_client_cmd_t *struct_ptr = data_ptr;
        
    return PARSE_SUCCEEDED;
}


/********** END OF REFEREE CMD PARSER FUNCTION **********/


const uint16_t Const_Referee_FRAME_HEADER_SOF       = 0xA5;     // 裁判系统指令帧头长度
const Referee_RobotAndClientIDTypeDef   // 机器人ID及对应客户端ID，0表示无对应客户端
    HERO_RED        = {1,   0x0101},    // 英雄(红)
    ENGINEER_RED    = {2,   0x0102},    // 工程(红)
    INFANTRY3_RED   = {3,   0x0103},    // 步兵3(红)
    INFANTRY4_RED   = {4,   0x0104},    // 步兵4(红)
    INFANTRY5_RED   = {5,   0x0105},    // 步兵5(红)
    AERIAL_RED      = {6,   0x0106},    // 空中(红)
    SENTRY_RED      = {7,   0},         // 哨兵(红)
    HERO_BLUE       = {101, 0x0165},    // 英雄(蓝)
    ENGINEER_BLUE   = {102, 0x0166},    // 工程(蓝)
    INFANTRY3_BLUE  = {103, 0x0167},    // 步兵3(蓝)
    INFANTRY4_BLUE  = {104, 0x0168},    // 步兵4(蓝)
    INFANTRY5_BLUE  = {105, 0x0169},    // 步兵5(蓝)
    AERIAL_BLUE     = {106, 0x016A},    // 空中(蓝)
    SENTRY_BLUE     = {107, 0};         // 哨兵(蓝)
        
const uint16_t Const_Referee_CMD_NUM                = 20;       // 裁判系统指令个数（不含交互指令）
const Referee_RefereeCmdTypeDef Const_Referee_CMD_LIST[Const_Referee_CMD_NUM] = {           // 裁判系统消息命令ID列表
    {0x0001,    11, &P_ext_game_status},                // 比赛状态数据，1Hz 周期发送
    {0x0002,    1,  &P_ext_game_result},                // 比赛结果数据，比赛结束后发送
    {0x0003,    28, &P_ext_game_robot_HP},              // 比赛机器人血量数据，1Hz 周期发送
    {0x0004,    3,  &P_ext_dart_status},                // 飞镖发射状态，飞镖发射时发送
    {0x0005,    11, NULL},                              // （未使用）人工智能挑战赛加成与惩罚区状态，1Hz周期发送
    {0x0101,    4,  &P_ext_event_data},                 // 场地事件数据，事件改变后发送
    {0x0102,    3,  &P_ext_supply_projectile_action},   // 场地补给站动作标识数据，动作改变后发送
    {0x0103,    2,  NULL},                              // （已废弃）请求补给站补弹数据，由参赛队发送，上限 10Hz。（RM 对抗赛尚未开放）
    {0x0104,    2,  &P_ext_referee_warning},            // 裁判警告数据，警告发生后发送
    {0x0105,    1,  &P_ext_dart_remaining_time},        // 飞镖发射口倒计时，1Hz周期发送
    {0x0201,    15, &P_ext_game_robot_status},          // 机器人状态数据，10Hz 周期发送
    {0x0202,    14, &P_ext_power_heat_data},            // 实时功率热量数据，50Hz 周期发送
    {0x0203,    16, &P_ext_game_robot_pos},             // 机器人位置数据，10Hz 发送
    {0x0204,    1,  &P_ext_buff},                       // 机器人增益数据，1Hz 周期发送
    {0x0205,    3,  &P_aerial_robot_energy},            // 空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送
    {0x0206,    1,  &P_ext_robot_hurt},                 // 伤害状态数据，伤害发生后发送
    {0x0207,    6,  &P_ext_shoot_data},                 // 实时射击数据，子弹发射后发送
    {0x0208,    2,  &P_ext_bullet_remaining},           // 弹丸剩余发射数，仅空中机器人，哨兵机器人以及ICRA机器人发送，1Hz周期发送
    {0x0209,    4,  &P_ext_rfid_status},                // 机器人RFID状态，1Hz周期发送
    {0x020A,    12, &P_ext_dart_cmd}                    // 飞镖机器人客户端指令书，10Hz周期发送
};

const Referee_RefereeCmdTypeDef Const_Referee_CMD_INTERACTIVE       = {0x0301, 8, NULL};    // 机器人间交互数据，发送方触发发送
// 注：这里的6是交互数据帧头的长度，因为交互数据帧是不定长的
//const uint16_t Const_Referee_DATA_CMD_ID_CLIENT_CUSTOM_DATA       = 0xD180;               // （已废弃）客户端自定义数据内容ID
const uint16_t Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND    = 0x0200;               // 机器人间交互数据内容ID下界
const uint16_t Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND    = 0x02FF;               // 机器人间交互数据内容ID上界
const uint16_t Const_Referee_DATA_INTERACTIVE_DATA_MAX_LENGTH       = 113 - 1;              // 机器人间交互数据内容最大长度
const uint16_t Const_Referee_GRAPHIC_BUFFER_MAX_LENGTH              = 21;                   // 图形缓冲区最大长度
const Referee_RefereeCmdTypeDef Const_Referee_DATA_CMD_ID_LIST[6]   = {                     // 裁判系统交互数据内容ID
    {0x0100,    2,      NULL},              // 客户端删除图形
    {0x0101,    15,     NULL},              // 客户端绘制一个图形
    {0x0102,    30,     NULL},              // 客户端绘制二个图形
    {0x0103,    75,     NULL},              // 客户端绘制五个图形
    {0x0104,    105,    NULL},              // 客户端绘制七个图形
    {0x0110,    45,     NULL}               // 客户端绘制字符图形
};

graphic_data_struct_t Referee_dummyGraphicCmd = {{0x00, 0x00, 0x00}, Draw_OPERATE_NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


/**
  * @brief      获取裁判系统数据对象的指针
  * @param      无
  * @retval     指针指向裁判系统数据对象
  */
Referee_RefereeDataTypeDef* Referee_GetRefereeDataPtr() {
    return &Referee_RefereeData;
}


/**
  * @brief      重置裁判系统数据对象
  * @param      无
  * @retval     无
  */
void Referee_ResetRefereeData() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    Referee_RefereeStateEnum state = referee->state;        // backup state
    uint32_t last_update_time = referee->last_update_time;  // backup time
    memset(referee, 0, sizeof(Referee_RefereeDataTypeDef));
    referee->state = state;
    referee->last_update_time = last_update_time;
}


/**
  * @brief      初始化裁判系统
  * @param      无
  * @retval     无
  */
void Referee_InitReferee() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    Referee_ResetRefereeData();
    referee->last_update_time = HAL_GetTick();
    Uart_InitUartDMA(Const_Referee_UART_HANDLER);
    Uart_ReceiveDMA(Const_Referee_UART_HANDLER, Referee_RxData, Const_Referee_RX_BUFF_LEN);
}


/**
  * @brief      通过机器人ID获取对应客户端ID
  * @param      robot_id: 机器人ID
  * @retval     客户端ID
  */
uint16_t Referee_GetClientIDByRobotID(uint8_t robot_id) {
    if (robot_id == 7 || robot_id == 107) return 0;
    if ((robot_id >= 1 && robot_id <= 6) || (robot_id >= 101 && robot_id <= 106)) 
        return robot_id + 0x100;
    return 0;
}


/**
  * @brief      裁判系统交互数据发送函数（阻塞）
  * @param      data_cmd_id: 数据内容ID
  * @param      receiver_ID: 接受者ID
  * @param      interactive_data: 交互数据帧
  * @param      interactive_data_length: 交互数据帧长度
  * @retval     无
  */
void Referee_SendInteractiveData(uint16_t data_cmd_id, uint16_t receiver_ID, const uint8_t *interactive_data, uint16_t interactive_data_length) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    static uint8_t seq = 0;
    uint8_t *buf = Referee_TxData;
    buf[0] = Const_Referee_FRAME_HEADER_SOF;
    
    uint16_t *data_length_ptr = (void *) (buf + 1);
    *data_length_ptr = interactive_data_length + Const_Referee_CMD_INTERACTIVE.data_length;
    
    uint8_t *seq_ptr = (void *) (buf + 3);
    *seq_ptr = seq;   // not obvious in doc
    seq = (seq + 1) % 256;

    uint8_t *crc8_ptr = (void *) (buf + 4);
    *crc8_ptr = CRC_GetCRC8CheckSum(buf, 4, CRC8_INIT);
    
    buf[5] = 0x01;
    buf[6] = 0x03;
    
    ext_student_interactive_header_data_t *header = (void *) (buf + 7);
    header->data_cmd_id  = data_cmd_id;
    header->receiver_ID  = receiver_ID;
    header->sender_ID    = (uint16_t)referee->robot_id;
    
    memcpy(buf + 5 + Const_Referee_CMD_INTERACTIVE.data_length, interactive_data, interactive_data_length);
    
    uint16_t *crc16_ptr = (void *) (buf + 5 + 2 + *data_length_ptr);
    *crc16_ptr = CRC_GetCRC16CheckSum(buf, 5 + 2 + *data_length_ptr, CRC16_INIT);
    
    uint16_t tx_size = 5 + 2 + *data_length_ptr + 2;
    Uart_SendMessage(Const_Referee_UART_HANDLER, buf, tx_size, 100);
}



/**
  * @brief      （已废弃）设置客户端自定义数据LED
  * @param      led_no: 客户端自定义数据LED序号
  * @param      led_state: 客户端自定义数据LED状态（1为绿，0为红）
  */
/* 
void Referee_SetClientCustomDataLED(uint8_t led_no, uint8_t led_state) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    if (led_no > 5) return;
    if (led_state) referee->custom_data.masks |= 1 << led_no;
    else referee->custom_data.masks &= ~(1 << led_no);
}
*/


/**
  * @brief      （已废弃）客户端自定义数据发送函数
  * @param      无
  * @retval     无
  */
/*
void Referee_SendClientCustomData() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_CLIENT_CUSTOM_DATA, referee->client_id, 
                                (void *) &(referee->custom_data), sizeof(referee->custom_data));
}
*/


/**
  * @brief      机器人间交互数据发送函数
  * @param      data_cmd_id: 数据内容ID
  * @param      receiver_ID: 接受者ID
  * @param      data: 数据帧
  * @param      data_length: 数据帧长度
  * @retval     无
  */
void Referee_SendRobotCustomData(uint16_t data_cmd_id, uint16_t receiver_ID, const uint8_t *data, uint16_t data_length) {
    if (data_cmd_id < Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND || 
        data_cmd_id > Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND)  return;                   // wrong data cmd id
    if (receiver_ID == 0 || (receiver_ID > 10 && receiver_ID < 110) || receiver_ID > 107) return;   // wrong receiver id
    if (data_length > Const_Referee_DATA_INTERACTIVE_DATA_MAX_LENGTH) return;                       // interactive data too long
    Referee_SendInteractiveData(data_cmd_id, receiver_ID, data, data_length);
}


/**
  * @brief      （已废弃）客户端自定义图形发送函数
  * @param      无
  * @retval     无
  */
/*
void Referee_SendClientGraphicDraw() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_CLIENT_GRAPHIC_DRAW, referee->client_id, 
                                (void *) &(referee->graphic_draw), sizeof(referee->graphic_draw));
}
*/


/**
  * @brief      发送客户端自定义图形命令组
  * @param      graph: 数组包括指定数量个图形命令
  * @param      mode: 发送模式，1、2、3、4对应1、2、5、7个一组
  * @retval     无
  */
void Referee_SendDrawingCmd(graphic_data_struct_t graph[], uint8_t mode) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    if (mode == 0 || mode >= 5) return;
    
    uint8_t buf[120], cellsize = sizeof(graphic_data_struct_t);
    if (mode >= 1) {
        memcpy(buf, graph, cellsize);
    }
    if (mode >= 2) {
        memcpy(buf + cellsize, graph + 1, cellsize);
    }
    if (mode >= 3) {
        memcpy(buf + cellsize * 2, graph + 2, cellsize);
        memcpy(buf + cellsize * 3, graph + 3, cellsize);
        memcpy(buf + cellsize * 4, graph + 4, cellsize);
    }
    if (mode >= 4) {
        memcpy(buf + cellsize * 5, graph + 5, cellsize);
        memcpy(buf + cellsize * 6, graph + 6, cellsize);
    }
    
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_LIST[mode].cmd_id, referee->client_id, 
                                buf, Const_Referee_DATA_CMD_ID_LIST[mode].data_length);
}


/**
  * @brief      发送客户端自定义图形显示字符串命令
  * @param      pgraph: 指针指向显示字符串图形命令
  * @param      str: 字符串（定长为30）
  * @retval     无
  */
void Referee_SendDrawingStringCmd(graphic_data_struct_t *pgraph, const uint8_t str[]) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    
    ext_client_custom_character_t struct_data;
    memcpy(&struct_data.grapic_data_struct, pgraph, sizeof(graphic_data_struct_t));
    memcpy(&struct_data.data, str, Const_Referee_DATA_CMD_ID_LIST[5].data_length - sizeof(graphic_data_struct_t));
    
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_LIST[5].cmd_id, referee->client_id, 
                                (void *) &struct_data, Const_Referee_DATA_CMD_ID_LIST[5].data_length);
}


/**
  * @brief      客户端自定义图形缓冲区是否为空
  * @param      无
  * @retval     1为空，0为非空
  */
uint8_t Referee_IsDrawingBufferEmpty() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    return referee->graphic_buf_len == 0;
}


/**
  * @brief      客户端自定义图形缓冲区刷写函数
  * @param      无
  * @retval     无
  */
void Referee_DrawingBufferFlush() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    if (Referee_IsDrawingBufferEmpty()) return;
    uint8_t cur = 0;
    while (cur + 7 < referee->graphic_buf_len) {
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 4);
        cur += 7;
    }
    uint8_t remain = referee->graphic_buf_len - cur;
    if (remain > 5) {
        for (int i = remain; i < 7; ++i)
            Referee_DrawingBufferPushDummy();
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 4);
    }
    else if (remain > 2) {
        for (int i = remain; i < 5; ++i)
            Referee_DrawingBufferPushDummy();
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 3);
    }
    else if (remain == 2) {
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 2);
    }
    else if (remain == 1) {
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 1);
    }
    referee->graphic_buf_len = 0;
}


/**
  * @brief      将空图形命令加入客户端自定义图形缓冲区（占位用）
  * @param      无
  * @retval     无
  */
void Referee_DrawingBufferPushDummy() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    memcpy(referee->graphic_buf + referee->graphic_buf_len, &Referee_dummyGraphicCmd, sizeof(graphic_data_struct_t));
    ++referee->graphic_buf_len;
}


/**
  * @brief      将图形命令加入客户端自定义图形缓冲区
  * @param      pgraph: 指针指向图形命令
  * @retval     无
  */
void Referee_DrawingBufferPush(graphic_data_struct_t *pgraph) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    memcpy(referee->graphic_buf + referee->graphic_buf_len, pgraph, sizeof(graphic_data_struct_t));
    ++referee->graphic_buf_len;
    if (referee->graphic_buf_len >= 7) {
        Referee_DrawingBufferFlush();
    }
}


/**
  * @brief      客户端自定义图形时基函数
  * @param      无
  * @retval     无
  */
void Referee_DrawingTimeBaseCallback() {
    static uint8_t tick = 0;
    ++tick;
    if (tick == 2) {
        tick = 0;
        Referee_DrawingBufferFlush();
    }
}


/**
  * @brief      打包图形命令
  * @param      详见协议及头文件定义
  * @retval     是否合法（1为是，0为否）
  */
uint32_t Referee_PackGraphicData(graphic_data_struct_t *pgraph, uint32_t graph_id, 
                                 Draw_OperateType operate_type, Draw_GraphicType graphic_type, uint8_t layer,
                                 Draw_Color color, uint16_t start_angle, uint16_t end_angle, 
                                 uint8_t width, uint16_t start_x, uint16_t start_y,
                                 uint16_t radius, uint16_t end_x, uint16_t end_y) 
{
    if (graph_id > 0xffffff) return PARSE_FAILED;
    pgraph->graphic_name[0] = graph_id & 0xff;
    pgraph->graphic_name[1] = (graph_id >> 8) & 0xff;
    pgraph->graphic_name[2] = (graph_id >> 16) & 0xff;
    
    pgraph->operate_type = (uint8_t) operate_type;
    pgraph->graphic_type = (uint8_t) graphic_type;
    
    if (layer > 9) return PARSE_FAILED;
    pgraph->layer = layer;
    
    pgraph->color = (uint8_t) color;
    
    if (start_angle > 0x7ff || end_angle > 0x7ff) return PARSE_FAILED;
    pgraph->start_angle = start_angle;
    pgraph->end_angle = end_angle;
    
    pgraph->width = width;
    
    if (start_x > 0x7ff || start_x > 0x7ff || radius > 0x3ff || end_x > 0x7ff || end_y > 0x7ff) return PARSE_FAILED;
    pgraph->start_x = start_x;
    pgraph->start_y = start_y;
    pgraph->radius = radius;
    pgraph->end_x = end_x;
    pgraph->end_y = end_y;
    
    return PARSE_SUCCEEDED;
}


/**
  * @brief      打包显示浮点数图形命令
  * @param      详见协议及头文件定义
  * @retval     是否合法（1为是，0为否）
  */
uint32_t Referee_PackFloatGraphicData(graphic_data_struct_t *pgraph, uint32_t graph_id, 
                                      Draw_OperateType operate_type, uint8_t layer,
                                      Draw_Color color, uint16_t font_size, uint16_t decimal_digit, 
                                      uint8_t width, uint16_t start_x, uint16_t start_y, float value)
{
    Referee_GraphicDataConverterUnion conv;
    conv.int_data = (int32_t) (value * 1000.0f);
    uint16_t radius = (conv.ui32_data) & 0x3ff;
    uint16_t end_x = (conv.ui32_data >> 10) & 0x7ff;
    uint16_t end_y = (conv.ui32_data >> 21) & 0x7ff;
    return Referee_PackGraphicData(pgraph, graph_id, operate_type, Draw_TYPE_FLOAT, layer, color, font_size, 
                                   decimal_digit, width, start_x, start_y, radius, end_x, end_y);
}


/**
  * @brief      打包显示整数图形命令
  * @param      详见协议及头文件定义
  * @retval     是否合法（1为是，0为否）
  */
uint32_t Referee_PackIntGraphicData(graphic_data_struct_t *pgraph, uint32_t graph_id, 
                                    Draw_OperateType operate_type, uint8_t layer,
                                    Draw_Color color, uint16_t font_size,
                                    uint8_t width, uint16_t start_x, uint16_t start_y, int value)
{
    Referee_GraphicDataConverterUnion conv;
    conv.int_data = value;
    uint16_t radius = (conv.ui32_data) & 0x3ff;
    uint16_t end_x = (conv.ui32_data >> 10) & 0x7ff;
    uint16_t end_y = (conv.ui32_data >> 21) & 0x7ff;
    return Referee_PackGraphicData(pgraph, graph_id, operate_type, Draw_TYPE_INT, layer, color, font_size, 
                                   0, width, start_x, start_y, radius, end_x, end_y);
}


/**
  * @brief      打包显示字符串图形命令
  * @param      详见协议及头文件定义
  * @retval     是否合法（1为是，0为否）
  */
uint32_t Referee_PackStringGraphicData(graphic_data_struct_t *pgraph, uint32_t graph_id, 
                                       Draw_OperateType operate_type, uint8_t layer,
                                       Draw_Color color, uint16_t font_size, uint8_t length,
                                       uint8_t width, uint16_t start_x, uint16_t start_y)
{
    if (length > Const_Referee_DATA_CMD_ID_LIST[5].data_length - sizeof(graphic_data_struct_t)) return PARSE_FAILED;
    return Referee_PackGraphicData(pgraph, graph_id, operate_type, Draw_TYPE_STRING, layer, color, font_size, 
                                   length, width, start_x, start_y, 0, 0, 0);
}


/********** REFEREE CUSTOM GRAPHIC DRAWING FUNCTION **********/


/**
  * @brief      绘图函数，清空指定图层
  * @param      layer: 图层号（0~9）
  * @retval     无
  */
void Draw_ClearLayer(uint8_t layer) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    Referee_DrawingBufferFlush();
    uint8_t buf[2];
    buf[0] = 1;
    buf[1] = layer;
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_LIST[0].cmd_id, referee->client_id, 
                                buf, Const_Referee_DATA_CMD_ID_LIST[0].data_length);
}


/**
  * @brief      绘图函数，清空全部
  * @param      无
  * @retval     无
  */
void Draw_ClearAll() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    //Referee_DrawingBufferFlush();
    referee->graphic_buf_len = 0;   // 直接抛弃缓冲区中的绘图指令
    uint8_t buf[2];
    buf[0] = 2;
    buf[1] = 0;
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_LIST[0].cmd_id, referee->client_id, 
                                buf, Const_Referee_DATA_CMD_ID_LIST[0].data_length);
}


/**
  * @brief      绘图函数，清空指定图形
  * @param      graph_id: 图形ID
  * @retval     无
  */
void Draw_Delete(uint32_t graph_id) {
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_DELETE, (Draw_GraphicType) 0, 0,
                                (Draw_Color) 0, 0, 0, 0, 0, 0, 0, 0, 0) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，画直线（新增）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_AddLine(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, 
                  uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y) 
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_ADD, Draw_TYPE_LINE, layer, color, 
                                0, 0, width, start_x, start_y, 0, end_x, end_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，画直线（修改）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_ModifyLine(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, 
                     uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y) 
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, Draw_TYPE_LINE, layer, color, 
                                0, 0, width, start_x, start_y, 0, end_x, end_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，画矩形（新增）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_AddRectangle(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, 
                       uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y) 
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_ADD, Draw_TYPE_RECTANGLE, layer, color, 
                                0, 0, width, start_x, start_y, 0, end_x, end_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，画矩形（修改）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_ModifyRectangle(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, 
                          uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y) 
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, Draw_TYPE_RECTANGLE, layer, color, 
                                0, 0, width, start_x, start_y, 0, end_x, end_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，画圆（新增）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_AddCircle(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, 
                    uint16_t center_x, uint16_t center_y, uint16_t radius) 
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_ADD, Draw_TYPE_CIRCLE, layer, color, 
                                0, 0, width, center_x, center_y, radius, 0, 0) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，画圆（修改）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_ModifyCircle(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, 
                       uint16_t center_x, uint16_t center_y, uint16_t radius) 
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, Draw_TYPE_CIRCLE, layer, color, 
                                0, 0, width, center_x, center_y, radius, 0, 0) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，画椭圆（新增）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_AddEllipse(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, 
                     uint16_t center_x, uint16_t center_y, uint16_t radius_x, uint16_t radius_y) 
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_ADD, Draw_TYPE_ELLIPSE, layer, color, 
                                0, 0, width, center_x, center_y, 0, radius_x, radius_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，画椭圆（修改）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_ModifyEllipse(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, 
                        uint16_t center_x, uint16_t center_y, uint16_t radius_x, uint16_t radius_y) 
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, Draw_TYPE_ELLIPSE, layer, color, 
                                0, 0, width, center_x, center_y, 0, radius_x, radius_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，画圆弧（新增）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_AddArc(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t start_angle, uint16_t end_angle, 
                 uint8_t width, uint16_t center_x, uint16_t center_y, uint16_t radius_x, uint16_t radius_y)  
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_ADD, Draw_TYPE_ARC, layer, color, 
                                start_angle, end_angle, width, center_x, center_y, 0, radius_x, radius_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，画圆弧（修改）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_ModifyArc(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t start_angle, uint16_t end_angle, 
                    uint8_t width, uint16_t center_x, uint16_t center_y, uint16_t radius_x, uint16_t radius_y) 
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, Draw_TYPE_ARC, layer, color, 
                                start_angle, end_angle, width, center_x, center_y, 0, radius_x, radius_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，显示浮点数（新增）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_AddFloat(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t font_size, uint16_t decimal_digit, 
                   uint8_t width, uint16_t start_x, uint16_t start_y, float value)  
{
    graphic_data_struct_t graph;
    if (Referee_PackFloatGraphicData(&graph, graph_id, Draw_OPERATE_ADD, layer, color, 
                                     font_size, decimal_digit, width, start_x, start_y, value) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，显示浮点数（修改）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_ModifyFloat(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t font_size, uint16_t decimal_digit, 
                      uint8_t width, uint16_t start_x, uint16_t start_y, float value) 
{
    graphic_data_struct_t graph;
    if (Referee_PackFloatGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, layer, color, 
                                     font_size, decimal_digit, width, start_x, start_y, value) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，显示整数（新增）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_AddInt(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t font_size,
                 uint8_t width, uint16_t start_x, uint16_t start_y, int value)  
{
    graphic_data_struct_t graph;
    if (Referee_PackIntGraphicData(&graph, graph_id, Draw_OPERATE_ADD, layer, color, 
                                   font_size, width, start_x, start_y, value) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，显示整数（修改）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_ModifyInt(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t font_size,
                    uint8_t width, uint16_t start_x, uint16_t start_y, int value) 
{
    graphic_data_struct_t graph;
    if (Referee_PackIntGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, layer, color, 
                                   font_size, width, start_x, start_y, value) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      绘图函数，显示字符串（新增）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_AddString(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t font_size, 
                    uint8_t width, uint16_t start_x, uint16_t start_y, const char str[])  
{
    graphic_data_struct_t graph;
    Referee_DrawingBufferFlush();
    uint8_t len = strlen(str);
    if (Referee_PackStringGraphicData(&graph, graph_id, Draw_OPERATE_ADD, layer, color, 
                                      font_size, len, width, start_x, start_y) != PARSE_SUCCEEDED)
        return;
    uint8_t buf[35];
    memcpy(buf, str, len);
    Referee_SendDrawingStringCmd(&graph, buf);
}


/**
  * @brief      绘图函数，显示字符串（修改）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_ModifyString(uint32_t graph_id, uint8_t layer, Draw_Color color, uint16_t font_size,
                       uint8_t width, uint16_t start_x, uint16_t start_y, const char str[]) 
{
    graphic_data_struct_t graph;
    Referee_DrawingBufferFlush();
    uint8_t len = strlen(str);
    if (Referee_PackStringGraphicData(&graph, graph_id, Draw_OPERATE_MODIFY, layer, color, 
                                      font_size, len, width, start_x, start_y) != PARSE_SUCCEEDED)
        return;
    uint8_t buf[35];
    memcpy(buf, str, len);
    Referee_SendDrawingStringCmd(&graph, buf);
}


/********** END OF REFEREE CUSTOM GRAPHIC DRAWING FUNCTION **********/


/**
  * @brief      判断裁判系统是否离线
  * @param      无
  * @retval     是否离线（1为是，0为否）
  */
uint8_t Referee_IsRefereeOffline() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    uint32_t now = HAL_GetTick();
    if ((now - referee->last_update_time) > Const_Referee_REMOTE_OFFLINE_TIME)
        referee->state = Referee_STATE_LOST;
    return referee->state == Referee_STATE_LOST;
}


/**
  * @brief      机器人间交互数据解析函数
  * @param      data: 数据帧
  * @param      data_length: 数据帧长度
  * @retval     解析结果（0为失败，1为成功）
  */
uint8_t Referee_ParseRobotCustomData(uint8_t* data, uint16_t data_length) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    
    //if (data_length != Const_Referee_CMD_INTERACTIVE.data_length) return PARSE_FAILED;      // wrong data length
    
    ext_student_interactive_header_data_t *header_struct_ptr = (void *) data;
    if (header_struct_ptr->data_cmd_id < Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND || 
        header_struct_ptr->data_cmd_id > Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND) 
        return PARSE_FAILED;    // wrong data cmd id
    if (header_struct_ptr->receiver_ID != referee->robot_id) return PARSE_FAILED;           // wrong receiver id
    
    //uint8_t interactive_data_ptr = data + Const_Referee_CMD_INTERACTIVE.data_length;
    
    // Interactive Data Recieve Callback
    
    return PARSE_SUCCEEDED;
}


/**
  * @brief      裁判系统数据解析函数
  * @param      cmd_id: 命令ID
  * @param      data: 数据帧
  * @param      data_length: 数据帧长度
  * @retval     解析结果（0为失败，1为成功）
  */
uint8_t Referee_ParseRefereeCmd(uint16_t cmd_id, uint8_t* data, uint16_t data_length) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    
    if (cmd_id == Const_Referee_CMD_INTERACTIVE.cmd_id) return Referee_ParseRobotCustomData(data, data_length);
    
    for (int i = 0; i < Const_Referee_CMD_NUM; ++i) {
        if (cmd_id == Const_Referee_CMD_LIST[i].cmd_id) {
            //if (data_length != Const_Referee_CMD_LIST[i].data_length) return PARSE_FAILED;  // wrong data length
            if (Const_Referee_CMD_LIST[i].parse_func == NULL) return PARSE_FAILED;          // unsupported cmd
            return (*(Const_Referee_CMD_LIST[i].parse_func))(referee, data);                // parse cmd
        }
    }
    
    return PARSE_FAILED;    // unknown cmd
}


/**
  * @brief      裁判系统串口数据解码函数
  * @param      buff: 数据缓冲区
  * @param      rxdatalen: 数据长度
  * @retval     无
  */
void Referee_DecodeRefereeData(uint8_t* buff, uint16_t rxdatalen) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    
    referee->state              = Referee_STATE_PENDING;    // 加锁防止互斥读写
    referee->last_update_time   = HAL_GetTick();            // 时间戳
    
    if (buff[0] != Const_Referee_FRAME_HEADER_SOF) {
        referee->state          = Referee_STATE_ERROR;
        return;
    }
    
//    if (!CRC_VerifyCRC8CheckSum(buff, 5)) {
//        referee->state          = Referee_STATE_ERROR;
//        return;
//    }
//    
    uint16_t data_length = (uint16_t) buff[2] << 8 | buff[1];
    uint8_t seq = buff[3];
    if (seq == 0) {
        //referee->state          = Referee_STATE_ERROR;
        //return;
    }
//    if (!CRC_VerifyCRC16CheckSum(buff, data_length + 9)) {
//        referee->state          = Referee_STATE_ERROR;
//        return;
//    }
    
    uint16_t cmd_id = (uint16_t) buff[6] << 8 | buff[5];
//    if (!Referee_ParseRefereeCmd(cmd_id, buff + 7, data_length)) {
//        referee->state          = Referee_STATE_ERROR;
//        return;
//    }
    
    referee->state              = Referee_STATE_CONNECTED;  // 解锁
}


/**
  * @brief      裁判系统串口回调函数
  * @param      huart: 指针指向串口句柄
  * @retval     无
  */
 dh_remotedata get_target;
uint16_t Calculate_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i]; // 直接 XOR 低字节，因为多项式是反转的
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)              // 检查最低位
                crc = (crc >> 1) ^ 0xA001; // 右移并应用多项式
            else
                crc >>= 1; // 只右移
        }
    }
    return crc;
}


// ???????
typedef struct {
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
} TargetData;

// ??????
TargetData get_targe;

void DaoHang_Receive() {

    if (Referee_RxData[0] == 0xA4) {
        if (sizeof(Referee_RxData) >= 25) {
            // ???????
            uint32_t temp;

            // ?? linear_x
            temp = (Referee_RxData[1] << 0) | (Referee_RxData[2] << 8) | (Referee_RxData[3] << 16) | (Referee_RxData[4] << 24);
            get_targe.linear_x = *(float*)&temp;
    
            // ?? linear_y
					
            temp = (Referee_RxData[5] << 0) | (Referee_RxData[6] << 8) | (Referee_RxData[7] << 16) | (Referee_RxData[8] << 24);
            get_targe.linear_y = *(float*)&temp;
    
            // ?? linear_z
            temp = (Referee_RxData[9] << 0) | (Referee_RxData[10] << 8) | (Referee_RxData[11] << 16) | (Referee_RxData[12] << 24);
            get_targe.linear_z = *(float*)&temp;
    
            // ?? angular_x
            temp = (Referee_RxData[13] << 0) | (Referee_RxData[14] << 8) | (Referee_RxData[15] << 16) | (Referee_RxData[16] << 24);
            get_targe.angular_x = *(float*)&temp;
    
            // ?? angular_y
            temp = (Referee_RxData[17] << 0) | (Referee_RxData[18] << 8) | (Referee_RxData[19] << 16) | (Referee_RxData[20] << 24);
            get_targe.angular_y = *(float*)&temp;
    
            // ?? angular_z
            temp = (Referee_RxData[21] << 0) | (Referee_RxData[22] << 8) | (Referee_RxData[23] << 16) | (Referee_RxData[24] << 24);
            get_targe.angular_z = *(float*)&temp;
    
      
    } else {
        printf("Invalid header. Expected 0xA4, got 0x%X\n", Referee_RxData[0]);
    }
		}
	}
void Referee_RXCallback(UART_HandleTypeDef* huart) {
    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle dbus data dbus_buf from DMA */

    uint16_t rxdatalen = Const_Referee_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
//    for (uint16_t i = 0; i < rxdatalen; i++){
//        if (Referee_RxData[i] == Const_Referee_FRAME_HEADER_SOF){
//            Referee_DecodeRefereeData(Referee_RxData + i, rxdatalen - i);
//        }
//    }
//    Referee_DecodeRefereeData(Referee_RxData, rxdatalen);
   // if(rxdatalen==7){
		DaoHang_Receive();
		//}
    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_Referee_RX_BUFF_LEN);
        //HAL_DMA_Start(huart->hdmarx,(uint32_t)&huart->Instance->DR,(uint32_t)Referee_RxData,Const_Referee_RX_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}
