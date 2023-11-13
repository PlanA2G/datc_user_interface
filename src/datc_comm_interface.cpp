/**
 * @file datc_comm_interface.cpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief
 * @version 1.0
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "datc_comm_interface.hpp"

const uint16_t kFreq = 50;

DatcCommInterface::DatcCommInterface(int argc, char **argv) {

}

DatcCommInterface::~DatcCommInterface() {
    flag_stop_ = true;
    usleep(1000);

    releaseTcp();
    modbusRelease();
}

bool DatcCommInterface::init(char *port_name, uint16_t slave_address) {
    if (!modbusInit(port_name, slave_address)) {
        return false;
    }

    COUT("DATC ros interface init.");

    start();
    return true;
}

void DatcCommInterface::initTcp(const string addr, uint16_t socket_port) {
    flag_tcp_stop_ = false;

    tcp_server_ = new TcpServer(socket_port);
    tcp_thread_ = std::thread(bind(&DatcCommInterface::recvCommand, this));

    is_socket_connected_ = true;
}

void DatcCommInterface::releaseTcp() {
    flag_tcp_stop_ = true;
    usleep(100);

    if (tcp_thread_.joinable()) {
        tcp_thread_.join();
    }

    tcp_server_->~TcpServer();
    is_socket_connected_ = false;
}

void DatcCommInterface::sendStatus() {
    DatcStatus status = getDatcStatus();

    Json::Value json;

    json["states"]     = status.states;
    json["motor_pos"]  = status.motor_pos;
    json["motor_vel"]  = status.motor_vel;
    json["motor_cur"]  = status.motor_cur;
    json["finger_pos"] = status.finger_pos;
    json["voltage"]    = status.voltage;

    unique_lock<mutex> lg(mutex_tcp_);

    MessageManager<Json::Value>::getInstance().pushToAllClientQueue(json);
}

void DatcCommInterface::recvCommand() {
    auto checkValueFn = [] (const Json::Value json, string str) {
        if (json.isMember(str)) {
            return true;
        } else {
            COUT("[Error] \"" + str + "\" must be entered.");
            return false;
        }
    };

    string cmd_str     = "command";
    string value_1_str = "value_1";
    string value_2_str = "value_2";

    Json::Value json;

    while (!flag_tcp_stop_) {
        mutex_tcp_.lock();

        if (MessageManager<Json::Value>::getInstance().tryPopFromWokerQueue(json)) {
            mutex_tcp_.unlock();

            if (!json.isMember(cmd_str)) {
                continue;
            }

            switch ((DATC_COMMAND) json[cmd_str].asUInt()) {
                case DATC_COMMAND::MOTOR_ENABLE:
                    motorEnable();
                    break;

                case DATC_COMMAND::MOTOR_STOP:
                    motorStop();
                    break;

                case DATC_COMMAND::MOTOR_DISABLE:
                    motorDisable();
                    break;

                case DATC_COMMAND::MOTOR_POSITION_CONTROL:
                    checkValueFn(json, value_1_str);
                    checkValueFn(json, value_2_str);
                    motorPosCtrl(json[value_1_str].asInt(), json[value_2_str].asUInt());
                    break;

                case DATC_COMMAND::MOTOR_VELOCITY_CONTROL:
                    checkValueFn(json, value_1_str);
                    checkValueFn(json, value_2_str);
                    motorVelCtrl(json[value_1_str].asInt(), json[value_2_str].asUInt());
                    break;

                case DATC_COMMAND::MOTOR_CURRENT_CONTROL:
                    checkValueFn(json, value_1_str);
                    checkValueFn(json, value_2_str);
                    motorCurCtrl(json[value_1_str].asInt(), json[value_2_str].asUInt());
                    break;

                case DATC_COMMAND::CHANGE_MODBUS_ADDRESS:
                    checkValueFn(json, value_1_str);
                    setModbusAddr(json[value_1_str].asUInt());
                    break;

                case DATC_COMMAND::GRIPPER_INITIALIZE:
                    grpInitialize();
                    break;

                case DATC_COMMAND::GRIPPER_OPEN:
                    grpOpen();
                    break;

                case DATC_COMMAND::GRIPPER_CLOSE:
                    grpClose();
                    break;

                case DATC_COMMAND::SET_FINGER_POSITION:
                    checkValueFn(json, value_1_str);
                    setFingerPos(json[value_1_str].asUInt());
                    break;

                case DATC_COMMAND::VACUUM_GRIPPER_ON:
                    vacuumGrpOn();
                    break;

                case DATC_COMMAND::VACUUM_GRIPPER_OFF:
                    vacuumGrpOff();
                    break;

                case DATC_COMMAND::SET_MOTOR_TORQUE:
                    checkValueFn(json, value_1_str);
                    setMotorTorque(json[value_1_str].asUInt());
                    break;

                case DATC_COMMAND::SET_MOTOR_SPEED:
                    checkValueFn(json, value_1_str);
                    setMotorSpeed(json[value_1_str].asUInt());
                    break;

                default:
                    COUT("Error: Undefined command.");
            }
        } else {
            mutex_tcp_.unlock();
        }

        usleep(10000);
    }
}

// Main loop
void DatcCommInterface::run() {
    std::thread thread_recv([&] () {
        double period = 1 / (double) kFreq;

        timespec time_prev, time_current;
        clock_gettime(CLOCK_MONOTONIC, &time_prev);

        while(!flag_stop_) {
            clock_gettime(CLOCK_MONOTONIC, &time_current);

            double dt = (time_current.tv_sec - time_prev.tv_sec) + ((time_current.tv_nsec - time_prev.tv_nsec) * 0.000000001);

            if(dt >= period) {
                if (mbc_.getConnectionState()) {
                    readDatcData();

                    if (is_socket_connected_) {
                        sendStatus();
                    }
                }

                time_prev = time_current;
            }
        }
    });

    while(!flag_stop_) {

    }

    motorDisable();
    modbusRelease();
}

