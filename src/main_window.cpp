/**
 * @file main_window.cpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief Implementation for the qt gui.
 * @version 1.0
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "main_window.hpp"

using namespace Qt;

namespace gripper_ui {

MainWindow::MainWindow(int argc, char **argv, bool &success, QWidget *parent): QMainWindow(parent) {
    ui.setupUi(this);

    //setWindowIcon(QIcon(":/images/icon.png"));
    datc_interface_ = new DatcCommInterface(argc, argv);

    // Button mapping
    QObject::connect(ui.pushButton_cmd_enable  , SIGNAL(clicked()), this, SLOT(datcEnable()));
    QObject::connect(ui.pushButton_cmd_disable , SIGNAL(clicked()), this, SLOT(datcDisable()));
    QObject::connect(ui.pushButton_set_position, SIGNAL(clicked()), this, SLOT(datcFingerPosCtrl()));

    QObject::connect(ui.pushButton_cmd_initialize    , SIGNAL(clicked()), this, SLOT(datcInit()));
    QObject::connect(ui.pushButton_cmd_grp_open      , SIGNAL(clicked()), this, SLOT(datcOpen()));
    QObject::connect(ui.pushButton_cmd_grp_close     , SIGNAL(clicked()), this, SLOT(datcClose()));
    QObject::connect(ui.pushButton_cmd_grp_vacuum_on , SIGNAL(clicked()), this, SLOT(datcVacuumGrpOn()));
    QObject::connect(ui.pushButton_cmd_grp_vacuum_off, SIGNAL(clicked()), this, SLOT(datcVacuumGrpOff()));

    QObject::connect(ui.pushButton_set_torque_ratio, SIGNAL(clicked()), this, SLOT(datcSetTorque()));

    // Modbus RTU related btn
    QObject::connect(ui.pushButton_modbus_start, SIGNAL(clicked()), this, SLOT(initModbus()));
    QObject::connect(ui.pushButton_modbus_stop , SIGNAL(clicked()), this, SLOT(releaseModbus()));
    QObject::connect(ui.pushButton_modbus_slave_change  , SIGNAL(clicked()), this, SLOT(changeSlaveAddress()));
    QObject::connect(ui.pushButton_modbus_set_slave_addr, SIGNAL(clicked()), this, SLOT(setSlaveAddr()));

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(WIN64) || defined(_WIN64) || defined(__WIN64__)
    ui.lineEdit_serial_port->setText("COM4");
#else
    ui.lineEdit_serial_port->setText("/dev/ttyUSB0");
#endif

    timer_ = new QTimer(this);
    connect(timer_, SIGNAL(timeout()), this, SLOT(timerCallback()));
    timer_->start(100); // msec

    datc_interface_->start();
    success = true;
}

MainWindow::~MainWindow() {
    if(datc_interface_ != NULL) {
        datc_interface_->~DatcCommInterface();
    }

    if(timer_ != NULL) {
        delete timer_;
    }
}

void MainWindow::timerCallback() {
    DatcStatus datc_status = datc_interface_->getDatcStatus();

    // Display
    ui.lineEdit_monitor_position-> setText(QString::number((double) datc_status.finger_pos / 100 , 'f', 1) + " %");
    ui.lineEdit_monitor_current -> setText(QString::number(datc_status.motor_cur, 'd', 0) + " mA");
    ui.lineEdit_monitor_mode    -> setText(QString::fromStdString(datc_status.status_str));

    // Comm. status check
    const bool is_modbus_connected = datc_interface_->getConnectionState();

    ui.pushButton_modbus_start->setEnabled(!is_modbus_connected);
    ui.pushButton_modbus_stop ->setEnabled(is_modbus_connected);
    ui.pushButton_modbus_set_slave_addr->setEnabled(is_modbus_connected);
    ui.pushButton_modbus_slave_change->setEnabled(is_modbus_connected);

    if (is_modbus_connected) {

    } else {

    }
}

// Enable Disable
void MainWindow::datcEnable() {
    datc_interface_->motorEnable();
}

void MainWindow::datcDisable() {
    datc_interface_->motorDisable();
}

// Datc control
void MainWindow::datcFingerPosCtrl() {
    datc_interface_->setFingerPos(ui.doubleSpinBox_grpPos_ctrl_range_1->value() * 100);
}

void MainWindow::datcInit() {
    datc_interface_->grpInitialize();
}

void MainWindow::datcOpen() {
    datc_interface_->grpOpen();
}

void MainWindow::datcClose() {
    datc_interface_->grpClose();
}

void MainWindow::datcVacuumGrpOn() {
    datc_interface_->vacuumGrpOn();
}

void MainWindow::datcVacuumGrpOff() {
    datc_interface_->vacuumGrpOff();
}

void MainWindow::datcSetTorque() {
    datc_interface_->setMotorTorque(ui.spinBox_motor_torque_ratio->value());
}

void MainWindow::datcSetSpeed() {

}

// Modbus RTU related
void MainWindow::initModbus() {
    COUT("--------------------------------------------");
    COUT("[INFO] Port: " + ui.lineEdit_serial_port->text().toStdString());
    COUT("[INFO] Slave address #" + ui.spinBox_slave_addr->text().toStdString());
    COUT("--------------------------------------------");

    const char* port = ui.lineEdit_serial_port->text().toStdString().c_str();
    uint16_t slave_addr = ui.spinBox_slave_addr->value();

    if (datc_interface_->init(port, slave_addr)) {

    } else {
        COUT("[ERROR] Port name or slave address invlaid !");
    }
}

void MainWindow::releaseModbus() {
    datc_interface_->modbusRelease();
}

void MainWindow::changeSlaveAddress() {

}

void MainWindow::setSlaveAddr() {

}

#ifndef RCLCPP__RCLCPP_HPP_
// TCP comm. related functions
void MainWindow::openTcpComm() {
    string addr          = "192.168.5.5";
    uint16_t socket_port = 12345;

    datc_interface_->initTcp(addr, socket_port);
}

void MainWindow::closeTcpComm() {
    datc_interface_->releaseTcp();
}
#endif

} // end of namespace
