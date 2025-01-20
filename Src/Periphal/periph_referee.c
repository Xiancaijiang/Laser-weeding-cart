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
    referee->mains_power_chassis_output = struct_ptr->mains_power_chassis_output;//���̹�������
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


const uint16_t Const_Referee_FRAME_HEADER_SOF       = 0xA5;     // ����ϵͳָ��֡ͷ����
const Referee_RobotAndClientIDTypeDef   // ������ID����Ӧ�ͻ���ID��0��ʾ�޶�Ӧ�ͻ���
    HERO_RED        = {1,   0x0101},    // Ӣ��(��)
    ENGINEER_RED    = {2,   0x0102},    // ����(��)
    INFANTRY3_RED   = {3,   0x0103},    // ����3(��)
    INFANTRY4_RED   = {4,   0x0104},    // ����4(��)
    INFANTRY5_RED   = {5,   0x0105},    // ����5(��)
    AERIAL_RED      = {6,   0x0106},    // ����(��)
    SENTRY_RED      = {7,   0},         // �ڱ�(��)
    HERO_BLUE       = {101, 0x0165},    // Ӣ��(��)
    ENGINEER_BLUE   = {102, 0x0166},    // ����(��)
    INFANTRY3_BLUE  = {103, 0x0167},    // ����3(��)
    INFANTRY4_BLUE  = {104, 0x0168},    // ����4(��)
    INFANTRY5_BLUE  = {105, 0x0169},    // ����5(��)
    AERIAL_BLUE     = {106, 0x016A},    // ����(��)
    SENTRY_BLUE     = {107, 0};         // �ڱ�(��)
        
const uint16_t Const_Referee_CMD_NUM                = 20;       // ����ϵͳָ���������������ָ�
const Referee_RefereeCmdTypeDef Const_Referee_CMD_LIST[Const_Referee_CMD_NUM] = {           // ����ϵͳ��Ϣ����ID�б�
    {0x0001,    11, &P_ext_game_status},                // ����״̬���ݣ�1Hz ���ڷ���
    {0x0002,    1,  &P_ext_game_result},                // ����������ݣ�������������
    {0x0003,    28, &P_ext_game_robot_HP},              // ����������Ѫ�����ݣ�1Hz ���ڷ���
    {0x0004,    3,  &P_ext_dart_status},                // ���ڷ���״̬�����ڷ���ʱ����
    {0x0005,    11, NULL},                              // ��δʹ�ã��˹�������ս���ӳ���ͷ���״̬��1Hz���ڷ���
    {0x0101,    4,  &P_ext_event_data},                 // �����¼����ݣ��¼��ı����
    {0x0102,    3,  &P_ext_supply_projectile_action},   // ���ز���վ������ʶ���ݣ������ı����
    {0x0103,    2,  NULL},                              // ���ѷ��������󲹸�վ�������ݣ��ɲ����ӷ��ͣ����� 10Hz����RM �Կ�����δ���ţ�
    {0x0104,    2,  &P_ext_referee_warning},            // ���о������ݣ����淢������
    {0x0105,    1,  &P_ext_dart_remaining_time},        // ���ڷ���ڵ���ʱ��1Hz���ڷ���
    {0x0201,    15, &P_ext_game_robot_status},          // ������״̬���ݣ�10Hz ���ڷ���
    {0x0202,    14, &P_ext_power_heat_data},            // ʵʱ�����������ݣ�50Hz ���ڷ���
    {0x0203,    16, &P_ext_game_robot_pos},             // ������λ�����ݣ�10Hz ����
    {0x0204,    1,  &P_ext_buff},                       // �������������ݣ�1Hz ���ڷ���
    {0x0205,    3,  &P_aerial_robot_energy},            // ���л���������״̬���ݣ�10Hz ���ڷ��ͣ�ֻ�п��л��������ط���
    {0x0206,    1,  &P_ext_robot_hurt},                 // �˺�״̬���ݣ��˺���������
    {0x0207,    6,  &P_ext_shoot_data},                 // ʵʱ������ݣ��ӵ��������
    {0x0208,    2,  &P_ext_bullet_remaining},           // ����ʣ�෢�����������л����ˣ��ڱ��������Լ�ICRA�����˷��ͣ�1Hz���ڷ���
    {0x0209,    4,  &P_ext_rfid_status},                // ������RFID״̬��1Hz���ڷ���
    {0x020A,    12, &P_ext_dart_cmd}                    // ���ڻ����˿ͻ���ָ���飬10Hz���ڷ���
};

const Referee_RefereeCmdTypeDef Const_Referee_CMD_INTERACTIVE       = {0x0301, 8, NULL};    // �����˼佻�����ݣ����ͷ���������
// ע�������6�ǽ�������֡ͷ�ĳ��ȣ���Ϊ��������֡�ǲ�������
//const uint16_t Const_Referee_DATA_CMD_ID_CLIENT_CUSTOM_DATA       = 0xD180;               // ���ѷ������ͻ����Զ�����������ID
const uint16_t Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND    = 0x0200;               // �����˼佻����������ID�½�
const uint16_t Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND    = 0x02FF;               // �����˼佻����������ID�Ͻ�
const uint16_t Const_Referee_DATA_INTERACTIVE_DATA_MAX_LENGTH       = 113 - 1;              // �����˼佻������������󳤶�
const uint16_t Const_Referee_GRAPHIC_BUFFER_MAX_LENGTH              = 21;                   // ͼ�λ�������󳤶�
const Referee_RefereeCmdTypeDef Const_Referee_DATA_CMD_ID_LIST[6]   = {                     // ����ϵͳ������������ID
    {0x0100,    2,      NULL},              // �ͻ���ɾ��ͼ��
    {0x0101,    15,     NULL},              // �ͻ��˻���һ��ͼ��
    {0x0102,    30,     NULL},              // �ͻ��˻��ƶ���ͼ��
    {0x0103,    75,     NULL},              // �ͻ��˻������ͼ��
    {0x0104,    105,    NULL},              // �ͻ��˻����߸�ͼ��
    {0x0110,    45,     NULL}               // �ͻ��˻����ַ�ͼ��
};

graphic_data_struct_t Referee_dummyGraphicCmd = {{0x00, 0x00, 0x00}, Draw_OPERATE_NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


/**
  * @brief      ��ȡ����ϵͳ���ݶ����ָ��
  * @param      ��
  * @retval     ָ��ָ�����ϵͳ���ݶ���
  */
Referee_RefereeDataTypeDef* Referee_GetRefereeDataPtr() {
    return &Referee_RefereeData;
}


/**
  * @brief      ���ò���ϵͳ���ݶ���
  * @param      ��
  * @retval     ��
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
  * @brief      ��ʼ������ϵͳ
  * @param      ��
  * @retval     ��
  */
void Referee_InitReferee() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    Referee_ResetRefereeData();
    referee->last_update_time = HAL_GetTick();
    Uart_InitUartDMA(Const_Referee_UART_HANDLER);
    Uart_ReceiveDMA(Const_Referee_UART_HANDLER, Referee_RxData, Const_Referee_RX_BUFF_LEN);
}


/**
  * @brief      ͨ��������ID��ȡ��Ӧ�ͻ���ID
  * @param      robot_id: ������ID
  * @retval     �ͻ���ID
  */
uint16_t Referee_GetClientIDByRobotID(uint8_t robot_id) {
    if (robot_id == 7 || robot_id == 107) return 0;
    if ((robot_id >= 1 && robot_id <= 6) || (robot_id >= 101 && robot_id <= 106)) 
        return robot_id + 0x100;
    return 0;
}


/**
  * @brief      ����ϵͳ�������ݷ��ͺ�����������
  * @param      data_cmd_id: ��������ID
  * @param      receiver_ID: ������ID
  * @param      interactive_data: ��������֡
  * @param      interactive_data_length: ��������֡����
  * @retval     ��
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
  * @brief      ���ѷ��������ÿͻ����Զ�������LED
  * @param      led_no: �ͻ����Զ�������LED���
  * @param      led_state: �ͻ����Զ�������LED״̬��1Ϊ�̣�0Ϊ�죩
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
  * @brief      ���ѷ������ͻ����Զ������ݷ��ͺ���
  * @param      ��
  * @retval     ��
  */
/*
void Referee_SendClientCustomData() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_CLIENT_CUSTOM_DATA, referee->client_id, 
                                (void *) &(referee->custom_data), sizeof(referee->custom_data));
}
*/


/**
  * @brief      �����˼佻�����ݷ��ͺ���
  * @param      data_cmd_id: ��������ID
  * @param      receiver_ID: ������ID
  * @param      data: ����֡
  * @param      data_length: ����֡����
  * @retval     ��
  */
void Referee_SendRobotCustomData(uint16_t data_cmd_id, uint16_t receiver_ID, const uint8_t *data, uint16_t data_length) {
    if (data_cmd_id < Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_LBOUND || 
        data_cmd_id > Const_Referee_DATA_CMD_ID_INTERACTIVE_DATA_UBOUND)  return;                   // wrong data cmd id
    if (receiver_ID == 0 || (receiver_ID > 10 && receiver_ID < 110) || receiver_ID > 107) return;   // wrong receiver id
    if (data_length > Const_Referee_DATA_INTERACTIVE_DATA_MAX_LENGTH) return;                       // interactive data too long
    Referee_SendInteractiveData(data_cmd_id, receiver_ID, data, data_length);
}


/**
  * @brief      ���ѷ������ͻ����Զ���ͼ�η��ͺ���
  * @param      ��
  * @retval     ��
  */
/*
void Referee_SendClientGraphicDraw() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_CLIENT_GRAPHIC_DRAW, referee->client_id, 
                                (void *) &(referee->graphic_draw), sizeof(referee->graphic_draw));
}
*/


/**
  * @brief      ���Ϳͻ����Զ���ͼ��������
  * @param      graph: �������ָ��������ͼ������
  * @param      mode: ����ģʽ��1��2��3��4��Ӧ1��2��5��7��һ��
  * @retval     ��
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
  * @brief      ���Ϳͻ����Զ���ͼ����ʾ�ַ�������
  * @param      pgraph: ָ��ָ����ʾ�ַ���ͼ������
  * @param      str: �ַ���������Ϊ30��
  * @retval     ��
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
  * @brief      �ͻ����Զ���ͼ�λ������Ƿ�Ϊ��
  * @param      ��
  * @retval     1Ϊ�գ�0Ϊ�ǿ�
  */
uint8_t Referee_IsDrawingBufferEmpty() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    return referee->graphic_buf_len == 0;
}


/**
  * @brief      �ͻ����Զ���ͼ�λ�����ˢд����
  * @param      ��
  * @retval     ��
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
  * @brief      ����ͼ���������ͻ����Զ���ͼ�λ�������ռλ�ã�
  * @param      ��
  * @retval     ��
  */
void Referee_DrawingBufferPushDummy() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    memcpy(referee->graphic_buf + referee->graphic_buf_len, &Referee_dummyGraphicCmd, sizeof(graphic_data_struct_t));
    ++referee->graphic_buf_len;
}


/**
  * @brief      ��ͼ���������ͻ����Զ���ͼ�λ�����
  * @param      pgraph: ָ��ָ��ͼ������
  * @retval     ��
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
  * @brief      �ͻ����Զ���ͼ��ʱ������
  * @param      ��
  * @retval     ��
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
  * @brief      ���ͼ������
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     �Ƿ�Ϸ���1Ϊ�ǣ�0Ϊ��
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
  * @brief      �����ʾ������ͼ������
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     �Ƿ�Ϸ���1Ϊ�ǣ�0Ϊ��
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
  * @brief      �����ʾ����ͼ������
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     �Ƿ�Ϸ���1Ϊ�ǣ�0Ϊ��
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
  * @brief      �����ʾ�ַ���ͼ������
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     �Ƿ�Ϸ���1Ϊ�ǣ�0Ϊ��
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
  * @brief      ��ͼ���������ָ��ͼ��
  * @param      layer: ͼ��ţ�0~9��
  * @retval     ��
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
  * @brief      ��ͼ���������ȫ��
  * @param      ��
  * @retval     ��
  */
void Draw_ClearAll() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    //Referee_DrawingBufferFlush();
    referee->graphic_buf_len = 0;   // ֱ�������������еĻ�ͼָ��
    uint8_t buf[2];
    buf[0] = 2;
    buf[1] = 0;
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_LIST[0].cmd_id, referee->client_id, 
                                buf, Const_Referee_DATA_CMD_ID_LIST[0].data_length);
}


/**
  * @brief      ��ͼ���������ָ��ͼ��
  * @param      graph_id: ͼ��ID
  * @retval     ��
  */
void Draw_Delete(uint32_t graph_id) {
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_DELETE, (Draw_GraphicType) 0, 0,
                                (Draw_Color) 0, 0, 0, 0, 0, 0, 0, 0, 0) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      ��ͼ��������ֱ�ߣ�������
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ��������ֱ�ߣ��޸ģ�
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ�����������Σ�������
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ�����������Σ��޸ģ�
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ��������Բ��������
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ��������Բ���޸ģ�
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ����������Բ��������
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ����������Բ���޸ģ�
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ��������Բ����������
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ��������Բ�����޸ģ�
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ��������ʾ��������������
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ��������ʾ���������޸ģ�
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ��������ʾ������������
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ��������ʾ�������޸ģ�
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ��������ʾ�ַ�����������
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      ��ͼ��������ʾ�ַ������޸ģ�
  * @param      ���Э�鼰ͷ�ļ�����
  * @retval     ��
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
  * @brief      �жϲ���ϵͳ�Ƿ�����
  * @param      ��
  * @retval     �Ƿ����ߣ�1Ϊ�ǣ�0Ϊ��
  */
uint8_t Referee_IsRefereeOffline() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    uint32_t now = HAL_GetTick();
    if ((now - referee->last_update_time) > Const_Referee_REMOTE_OFFLINE_TIME)
        referee->state = Referee_STATE_LOST;
    return referee->state == Referee_STATE_LOST;
}


/**
  * @brief      �����˼佻�����ݽ�������
  * @param      data: ����֡
  * @param      data_length: ����֡����
  * @retval     ���������0Ϊʧ�ܣ�1Ϊ�ɹ���
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
  * @brief      ����ϵͳ���ݽ�������
  * @param      cmd_id: ����ID
  * @param      data: ����֡
  * @param      data_length: ����֡����
  * @retval     ���������0Ϊʧ�ܣ�1Ϊ�ɹ���
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
  * @brief      ����ϵͳ�������ݽ��뺯��
  * @param      buff: ���ݻ�����
  * @param      rxdatalen: ���ݳ���
  * @retval     ��
  */
void Referee_DecodeRefereeData(uint8_t* buff, uint16_t rxdatalen) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    
    referee->state              = Referee_STATE_PENDING;    // ������ֹ�����д
    referee->last_update_time   = HAL_GetTick();            // ʱ���
    
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
    
    referee->state              = Referee_STATE_CONNECTED;  // ����
}


/**
  * @brief      ����ϵͳ���ڻص�����
  * @param      huart: ָ��ָ�򴮿ھ��
  * @retval     ��
  */
 dh_remotedata get_target;
uint16_t Calculate_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i]; // ֱ�� XOR ���ֽڣ���Ϊ����ʽ�Ƿ�ת��
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)              // ������λ
                crc = (crc >> 1) ^ 0xA001; // ���Ʋ�Ӧ�ö���ʽ
            else
                crc >>= 1; // ֻ����
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
