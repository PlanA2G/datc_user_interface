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
    modbusRelease();
    flag_stop_ = true;
}

bool DatcCommInterface::init(char *port_name, uint16_t slave_address) {
    if (!modbusInit(port_name, slave_address)) {
        return false;
    }

    COUT("DATC ros interface init.");

    start();
    return true;
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

