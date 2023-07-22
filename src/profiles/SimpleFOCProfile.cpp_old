#include "SimpleFOCProfile.h"

uint16_t data1;
uint16_t data2;
uint16_t data3;

uint16_t rxData1;
uint16_t rxData2;
uint16_t rxData3;
uint16_t rxData4;

uint16_t pack_half_float(float f)
{
    uint32_t t = *(uint32_t*)&f;
    // man bits = 10; but we keep 11 for rounding
    uint16_t man = (t & 0x007FFFFF) >> 12;
    int16_t  exp = (t & 0x7F800000) >> 23;
    bool     sgn = (t & 0x80000000);

    // handle 0
    if ((t & 0x7FFFFFFF) == 0)
    {
        return sgn?0x8000:0x0000;
    }
    // denormalized float32 does not fit in float16
    if (exp == 0x00)
    {
        return sgn?0x8000:0x0000;
    }
    // handle inf & NAN
    if (exp == 0x00FF)
    {
        if (man) return 0xFE00; // NAN
        return sgn?0xFC00:0x7C00; // -INF : INF
    }
    // normal numbers
    exp = exp - 127 + 15;
    if (exp > 30) // overflow does not fit => INF
    {
        return sgn?0xFC00:0x7C00;
    }
    if (exp < -38) // subnormal not possible => zero
    {
        return sgn?0x8000:0x0000;
    }
    if (exp <= 0) // subnormal
    {
        man >>= (exp+14);
        // rounding
        man++;
        man >>= 1;
        return sgn?0x8000:0x0000 | man;
    }
    // normal
    // todo rounding
    exp <<= 10;
    man++;
    man >>= 1;
    return sgn?0x8000:0x0000 | exp | man;
}

float unpack_half_float(uint16_t n)
{
    uint16_t sgn, man;
    int exp;
    double f;

    sgn = (n & 0x8000) > 0;
    exp = (n & 0x7C00) >> 10;
    man = (n & 0x03FF);

    // ZERO
    if ((n & 0x7FFF) == 0)
    {
        return sgn?-0:0;
    }
    // NAN & INF
    if (exp == 0x001F)
    {
        if (man == 0) return sgn?-INFINITY:INFINITY;
        else return NAN;
    }
    // SUBNORMAL/NORMAL
    if (exp == 0)  f = 0;
    else           f = 1;
    // PROCESS MANTISSE
    for (int i=9; i>=0; i--)
    {
        f *= 2;
        if (man & (1<<i)) f = f + 1;
    }
    f = f * pow(2.0, exp-25);
    if (exp == 0)
    {
        f = f * pow(2.0, -13); // 5.96046447754e-8;
    }
    return sgn?-f:f;
}

void can_cmd_send_pid(can_message_t* txMessage, uint16_t nodeId, uint16_t commandId, PIDController* controller) { 
    
    data1 = pack_half_float(controller->P);
    data2 = pack_half_float(controller->I);
    data3 = pack_half_float(controller->D);
    memcpy(&txMessage->data[0], &data1 , 2);
    memcpy(&txMessage->data[2], &data2 , 2);
    memcpy(&txMessage->data[4], &data3 , 2);
    txMessage->id  = CAN_ID_FROM(nodeId, commandId);
    txMessage->dlc = 6;
    txMessage->isStandard = true;
    txMessage->isRTR = false;
}

WEAK void can_cmd_send_canopen_nmt(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_CANOPEN_NMT);
}

WEAK void can_cmd_send_heartbeat(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {

}

WEAK void can_cmd_send_estop(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    byte enable = !motor->enabled;
    txMessage->id  = CAN_ID_FROM(nodeId, CMD_ESTOP);
    txMessage->dlc = 1;
    txMessage->data[0] = enable;
    txMessage->isStandard = true;
    txMessage->isRTR = false;
}

WEAK void can_cmd_send_target(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    uint16_t target = pack_half_float(motor->target);
    Serial.print("send target "); Serial.println(motor->target);
    txMessage->id  = CAN_ID_FROM(nodeId, CMD_ESTOP);
    txMessage->dlc = 2;
    txMessage->isStandard = true;
    txMessage->isRTR = false;
    memcpy(&txMessage->data[0], &target, 2);
}

WEAK void can_cmd_send_error(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {

}

WEAK void can_cmd_send_reserved_2(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_CANOPEN_NMT);
}

WEAK void can_cmd_send_pid_velocity(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_pid(txMessage, nodeId, CMD_PID_VELOCITY, &motor->PID_velocity);
}

WEAK void can_cmd_send_pid_velocity_2(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {

}

WEAK void can_cmd_send_pid_angle(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_pid(txMessage, nodeId, CMD_PID_ANGLE, &motor->P_angle);
}

WEAK void can_cmd_send_pid_angle_2(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {

}

WEAK void can_cmd_send_pid_current_d(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_pid(txMessage, nodeId, CMD_PID_CURRENT_D, &motor->PID_current_d);
}

WEAK void can_cmd_send_pid_current_d_2(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {

}

WEAK void can_cmd_send_pid_current_q(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_pid(txMessage, nodeId, CMD_PID_CURRENT_Q, &motor->PID_current_q);
}

WEAK void can_cmd_send_pid_current_q_2(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {

}

WEAK void can_cmd_send_reserved_3(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_CANOPEN_NMT);
}

WEAK void can_cmd_send_reserved_4(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_CANOPEN_NMT);
}

WEAK void can_cmd_send_reserved_5(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_CANOPEN_NMT);
}

WEAK void can_cmd_send_reserved_6(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_CANOPEN_NMT);
}

WEAK void can_cmd_send_lpf(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {

}

WEAK void can_cmd_send_motor(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {

}

WEAK void can_cmd_send_motor_limits(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {

}

WEAK void can_cmd_send_controller(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {

}

WEAK void can_cmd_send_dq(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {

}

WEAK void can_cmd_send_phase(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {

}

WEAK void can_cmd_send_driver(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {

}

WEAK void can_cmd_send_reserved_7(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_CANOPEN_NMT);
}

WEAK void can_cmd_send_reserved_8(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_CANOPEN_NMT);
}

WEAK void can_cmd_send_reserved_9(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_CANOPEN_NMT);
}

WEAK void can_cmd_send_reserved_10(can_message_t* txMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_CANOPEN_NMT);
}

WEAK void can_cmd_send_user_adc(can_message_t* txMessage, uint16_t nodeId) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_CANOPEN_NMT);
}

WEAK void can_cmd_send_user_2(can_message_t* txMessage, uint16_t nodeId) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_CANOPEN_NMT);
}

WEAK void can_cmd_send_user_3(can_message_t* txMessage, uint16_t nodeId) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_CANOPEN_NMT);
}


/**
 * Set Functions (after receiving data frame)
 */
WEAK void can_cmd_set_canopen_nmt(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_heartbeat(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_estop(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    if (rxMessage->data[0] == 0) {
        motor->enable();
    } else {
        motor->disable();
    }
}

WEAK void can_cmd_set_target(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    uint16_t target;
    // Serial.print("set target "); Serial.println(target);
    memcpy(&target, &rxMessage->data[0], 2);
    // Serial.print("set target "); Serial.println(rx[0]);
    motor->target = unpack_half_float(target);
    Serial.println(motor->target);

}

WEAK void can_cmd_set_error(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_reserved_2(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

void can_cmd_set_pid(can_message_t* rxMessage, uint16_t nodeId, PIDController* controller) {

    if (rxMessage->dlc >= 6) {
        memcpy(&rxData1, &rxMessage->data[0],2);
        memcpy(&rxData2, &rxMessage->data[2],2);
        memcpy(&rxData3, &rxMessage->data[4],2);
        controller->P = unpack_half_float(rxData1);
        controller->I = unpack_half_float(rxData2);
        controller->D = unpack_half_float(rxData3);
    }
}

WEAK void can_cmd_set_pid_velocity(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_set_pid(rxMessage, nodeId, &motor->PID_velocity);
}

WEAK void can_cmd_set_pid_velocity_2(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_pid_angle(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_set_pid(rxMessage, nodeId, &motor->P_angle);
}

WEAK void can_cmd_set_pid_angle_2(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_pid_current_d(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_set_pid(rxMessage, nodeId, &motor->PID_current_d);
}

WEAK void can_cmd_set_pid_current_d_2(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_pid_current_q(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    can_cmd_set_pid(rxMessage, nodeId, &motor->PID_current_q);
}

WEAK void can_cmd_set_pid_current_q_2(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_reserved_3(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_reserved_4(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_reserved_5(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_reserved_6(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_lpf(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_motor(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_motor_limits(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_controller(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_dq(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_phase(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_driver(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_reserved_7(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_reserved_8(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_reserved_9(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_reserved_10(can_message_t* rxMessage, uint16_t nodeId, BLDCMotor* motor) {
    
}

WEAK void can_cmd_set_user_adc(can_message_t* rxMessage, uint16_t nodeId) {
    
}

WEAK void can_cmd_set_user_2(can_message_t* rxMessage, uint16_t nodeId) {
    
}

WEAK void can_cmd_set_user_3(can_message_t* rxMessage, uint16_t nodeId) {
    
}

WEAK void can_cmd_send_message_not_implemented(can_message_t* txMessage, uint16_t nodeId, uint16_t commandId ) {
    txMessage->id = CAN_ID_FROM(nodeId, commandId);
    txMessage->dlc = 3;
    txMessage->data[0] = ERROR_CMD_NOT_IMPLEMENTED;
    txMessage->data[1] = commandId >> 8;
    txMessage->data[2] = commandId & 0xFF;
    txMessage->isRTR = false;
    txMessage->isStandard = true;
}

/* can be overriden - the default implementation does not know what pins for ADC */
WEAK void can_cmd_user_adc(can_message_t* txMessage, uint16_t nodeId) {
    can_cmd_send_message_not_implemented(txMessage, nodeId, CMD_USER_ADC);
}

SimpleFOCProfile::SimpleFOCProfile(uint8_t nodeId, BLDCMotor* motor) {
    _motor = motor;
    _nodeId = nodeId;
}

void SimpleFOCProfile::handleDataFrame(can_message_t* dataMessage) {
    uint16_t node_id = NODE_ID_FROM(dataMessage->id);  
    uint16_t command_id = COMMAND_ID_FROM(dataMessage->id);
       
       switch (command_id) {
           case CMD_CANOPEN_NMT: 
            can_cmd_set_canopen_nmt(dataMessage, _nodeId, _motor);
            break;
        case CMD_HEARTBEAT: 
            can_cmd_set_heartbeat(dataMessage, _nodeId, _motor);
            break;
        case CMD_ESTOP: 
            can_cmd_set_estop(dataMessage, _nodeId, _motor);
            break;
        case CMD_TARGET: 
            can_cmd_set_target(dataMessage, _nodeId, _motor);
            break;
        case CMD_ERROR: 
            can_cmd_set_error(dataMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_2: 
            can_cmd_set_reserved_2(dataMessage, _nodeId, _motor);
            break;
        case CMD_PID_VELOCITY:
            can_cmd_set_pid_velocity(dataMessage, _nodeId, _motor);
            break;
        case CMD_PID_VELOCITY_2: 
            can_cmd_set_pid_velocity_2(dataMessage, _nodeId, _motor);
            break;
        case CMD_PID_ANGLE: 
            can_cmd_set_pid_angle(dataMessage, _nodeId, _motor);
            break;
        case CMD_PID_ANGLE_2: 
            can_cmd_set_pid_angle_2(dataMessage, _nodeId, _motor);
            break;
        case CMD_PID_CURRENT_D: 
            can_cmd_set_pid_current_d(dataMessage, _nodeId, _motor);
            break;
        case CMD_PID_CURRENT_D_2: 
            can_cmd_set_pid_current_d_2(dataMessage, _nodeId, _motor);
            break;
        case CMD_PID_CURRENT_Q: 
            can_cmd_set_pid_current_q(dataMessage, _nodeId, _motor);
            break;
        case CMD_PID_CURRENT_Q_2: 
            can_cmd_set_pid_current_q_2(dataMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_3:
            can_cmd_set_reserved_3(dataMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_4: 
            can_cmd_set_reserved_4(dataMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_5: 
            can_cmd_set_reserved_5(dataMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_6: 
            can_cmd_set_reserved_6(dataMessage, _nodeId, _motor);
            break;
        case CMD_LPF: 
            can_cmd_set_lpf(dataMessage, _nodeId, _motor);
            break;
        case CMD_MOTOR: 
            can_cmd_set_motor(dataMessage, _nodeId, _motor);
            break;
        case CMD_MOTOR_LIMITS: 
            can_cmd_set_motor_limits(dataMessage, _nodeId, _motor);
            break;
        case CMD_CONTROLLER: 
            can_cmd_set_controller(dataMessage, _nodeId, _motor);
            break;
        case CMD_DQ: 
            can_cmd_set_dq(dataMessage, _nodeId, _motor);
            break;
        case CMD_PHASE: 
            can_cmd_set_phase(dataMessage, _nodeId, _motor);
            break;
        case CMD_DRIVER: 
            can_cmd_set_driver(dataMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_7: 
            can_cmd_set_reserved_7(dataMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_8: 
            can_cmd_set_reserved_8(dataMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_9: 
            can_cmd_set_reserved_9(dataMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_10: 
            can_cmd_set_reserved_10(dataMessage, _nodeId, _motor);
            break;
        case CMD_USER_ADC: 
            can_cmd_set_user_adc(dataMessage, _nodeId);
            break;
        case CMD_USER_2: 
            can_cmd_set_user_2(dataMessage, _nodeId);
            break;
        case CMD_USER_3: 
            can_cmd_set_user_3(dataMessage, _nodeId);
            break;
        default:
            // can_cmd_send_message_not_implemented(txMessage, _nodeId, command);
            break;


       }
}

void SimpleFOCProfile::handleRemoteFrame(can_message_t* rtrMessage, can_message_t* txMessage) {
    // TODO: What to do with RTR not for is e.g. node_id != _nodeId?
    uint16_t node_id = NODE_ID_FROM(rtrMessage->id);  
    uint16_t command_id = COMMAND_ID_FROM(rtrMessage->id);

    // Serial.print("ZZZZ "); Serial.println(rtrMessage->id);
    // Serial.print("ZZZZ "); Serial.println(((1 << COMMAND_BITS) -1));
    // Serial.print("ZZZZ "); Serial.println(command);
    // can_message_t txMessage = {};

    
    switch (command_id) {
        case CMD_CANOPEN_NMT: 
            can_cmd_send_canopen_nmt(txMessage, _nodeId, _motor);
            break;
        case CMD_HEARTBEAT: 
            can_cmd_send_heartbeat(txMessage, _nodeId, _motor);
            break;
        case CMD_ESTOP: 
            can_cmd_send_estop(txMessage, _nodeId, _motor);
            break;
        case CMD_TARGET: 
            can_cmd_send_target(txMessage, _nodeId, _motor);
            break;
        case CMD_ERROR: 
            can_cmd_send_error(txMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_2: 
            can_cmd_send_reserved_2(txMessage, _nodeId, _motor);
            break;
        case CMD_PID_VELOCITY:
            can_cmd_send_pid_velocity(txMessage, _nodeId, _motor);
            break;
        case CMD_PID_VELOCITY_2: 
            can_cmd_send_pid_velocity_2(txMessage, _nodeId, _motor);
            break;
        case CMD_PID_ANGLE: 
            can_cmd_send_pid_angle(txMessage, _nodeId, _motor);
            break;
        case CMD_PID_ANGLE_2: 
            can_cmd_send_pid_angle_2(txMessage, _nodeId, _motor);
            break;
        case CMD_PID_CURRENT_D: 
            can_cmd_send_pid_current_d(txMessage, _nodeId, _motor);
            break;
        case CMD_PID_CURRENT_D_2: 
            can_cmd_send_pid_current_d_2(txMessage, _nodeId, _motor);
            break;
        case CMD_PID_CURRENT_Q: 
            can_cmd_send_pid_current_q(txMessage, _nodeId, _motor);
            break;
        case CMD_PID_CURRENT_Q_2: 
            can_cmd_send_pid_current_q_2(txMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_3:
            can_cmd_send_reserved_3(txMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_4: 
            can_cmd_send_reserved_4(txMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_5: 
            can_cmd_send_reserved_5(txMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_6: 
            can_cmd_send_reserved_6(txMessage, _nodeId, _motor);
            break;
        case CMD_LPF: 
            can_cmd_send_lpf(txMessage, _nodeId, _motor);
            break;
        case CMD_MOTOR: 
            can_cmd_send_motor(txMessage, _nodeId, _motor);
            break;
        case CMD_MOTOR_LIMITS: 
            can_cmd_send_motor_limits(txMessage, _nodeId, _motor);
            break;
        case CMD_CONTROLLER: 
            can_cmd_send_controller(txMessage, _nodeId, _motor);
            break;
        case CMD_DQ: 
            can_cmd_send_dq(txMessage, _nodeId, _motor);
            break;
        case CMD_PHASE: 
            can_cmd_send_phase(txMessage, _nodeId, _motor);
            break;
        case CMD_DRIVER: 
            can_cmd_send_driver(txMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_7: 
            can_cmd_send_reserved_7(txMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_8: 
            can_cmd_send_reserved_8(txMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_9: 
            can_cmd_send_reserved_9(txMessage, _nodeId, _motor);
            break;
        case CMD_RESERVED_10: 
            can_cmd_send_reserved_10(txMessage, _nodeId, _motor);
            break;
        case CMD_USER_ADC: 
            can_cmd_send_user_adc(txMessage, _nodeId);
            break;
        case CMD_USER_2: 
            can_cmd_send_user_2(txMessage, _nodeId);
            break;
        case CMD_USER_3: 
            can_cmd_send_user_3(txMessage, _nodeId);
            break;
        default:
            can_cmd_send_message_not_implemented(txMessage, _nodeId, command_id);
            break;
    }
}