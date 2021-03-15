#include <SimpleFOC.h>
#include <SimpleCAN.h>

#define NODE_BITS 6
#define COMMAND_BITS 5

#define CAN_ID_FROM(NODE_ID, COMMAND_ID) ((NODE_ID << COMMAND_BITS) + COMMAND_ID)
#define NODE_ID_FROM(CAN_ID) (CAN_ID >> COMMAND_BITS)
#define COMMAND_ID_FROM(CAN_ID) (CAN_ID & ((1 << COMMAND_BITS) -1))



void can_cmd_send_canopen_nmt(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_heartbeat(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_estop(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_target(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_error(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_reserved_2(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_pid_velocity(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_pid_velocity_2(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_pid_angle(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_pid_angle_2(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_pid_current_d(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_pid_current_d_2(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_pid_current_q(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_pid_current_q_2(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_reserved_3(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_reserved_4(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_reserved_5(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_reserved_6(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_lpf(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_motor(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_motor_limits(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_controller(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_dq(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_phase(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_driver(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_reserved_7(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_reserved_8(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_reserved_9(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_reserved_10(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_send_user_adc(can_message_t* txMessage, uint16_t nodeId);
void can_cmd_send_user_2(can_message_t* txMessage, uint16_t nodeId);
void can_cmd_send_user_3(can_message_t* txMessage, uint16_t nodeId);
void can_cmd_send_message_not_implemented(can_message_t* txMessage, uint16_t nodeId, uint16_t commandId);


void can_cmd_set_canopen_nmt(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_heartbeat(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_estop(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_target(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_error(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_reserved_2(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_pid_velocity(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_pid_velocity_2(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_pid_angle(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_pid_angle_2(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_pid_current_d(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_pid_current_d_2(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_pid_current_q(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_pid_current_q_2(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_reserved_3(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_reserved_4(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_reserved_5(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_reserved_6(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_lpf(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_motor(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_motor_limits(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_controller(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_dq(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_phase(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_driver(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_reserved_7(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_reserved_8(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_reserved_9(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_reserved_10(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor);
void can_cmd_set_user_adc(can_message_t* rxMessage, uint16_t nodeId);
void can_cmd_set_user_2(can_message_t* rxMessage, uint16_t nodeId);
void can_cmd_set_user_3(can_message_t* rxMessage, uint16_t nodeId);


// CAN commands fit in the last 5 bits of identifier 
enum CAN_COMMANDS {
    CMD_CANOPEN_NMT     = 0x00,
    CMD_HEARTBEAT       = 0x01,
    CMD_ESTOP           = 0x02,
    CMD_TARGET          = 0x03,
    CMD_ERROR           = 0x04,
    CMD_RESERVED_2      = 0x05,
    CMD_PID_VELOCITY    = 0x06,
    CMD_PID_VELOCITY_2  = 0x07,
    CMD_PID_ANGLE       = 0x08,
    CMD_PID_ANGLE_2     = 0x09,
    CMD_PID_CURRENT_D   = 0x0A,
    CMD_PID_CURRENT_D_2 = 0x0B,
    CMD_PID_CURRENT_Q   = 0x0C,
    CMD_PID_CURRENT_Q_2 = 0x0D,
    CMD_RESERVED_3      = 0x0E,
    CMD_RESERVED_4      = 0x0F,
    CMD_RESERVED_5      = 0x10,
    CMD_RESERVED_6      = 0x11,
    CMD_LPF             = 0x12,
    CMD_MOTOR           = 0x13,
    CMD_MOTOR_LIMITS    = 0x14,
    CMD_CONTROLLER      = 0x15,
    CMD_DQ              = 0x16,
    CMD_PHASE           = 0x17,
    CMD_DRIVER          = 0x18,
    CMD_RESERVED_7      = 0x19,
    CMD_RESERVED_8      = 0x1A,
    CMD_RESERVED_9      = 0x1B,
    CMD_RESERVED_10     = 0x1C,
    CMD_USER_ADC        = 0x1D,
    CMD_USER_2          = 0x1E,
    CMD_USER_3          = 0x1F,
};

enum CAN_ERRORS {
    ERROR_CMD_NOT_IMPLEMENTED=0xFF,
};

uint16_t pack_half_float(float f);
float unpack_half_float(uint16_t n);

class SimpleFOCProfile {

public:
    SimpleFOCProfile(uint8_t nodeId, BLDCMotor* motor);
    void handleDataFrame(can_message_t* dataMessage);
    void handleRemoteFrame(can_message_t* remoteMessage, can_message_t* txMessage);
    
private:
    BLDCMotor* _motor;
    uint16_t _nodeId;

};