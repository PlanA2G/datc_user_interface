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
    QObject::connect(ui.pushButton_cmd_enable  , SIGNAL(clicked()), this, SLOT(pushButton_cmdEnableCallback()));
    QObject::connect(ui.pushButton_cmd_disable , SIGNAL(clicked()), this, SLOT(pushButton_cmdDisableCallback()));
    QObject::connect(ui.pushButton_set_position, SIGNAL(clicked()), this, SLOT(pushButton_grpPosCtrlCallback()));

    QObject::connect(ui.pushButton_cmd_initialize    , SIGNAL(clicked()), this, SLOT(pushButton_grpInitCallback()));
    QObject::connect(ui.pushButton_cmd_grp_open      , SIGNAL(clicked()), this, SLOT(pushButton_grpOpenCallback()));
    QObject::connect(ui.pushButton_cmd_grp_close     , SIGNAL(clicked()), this, SLOT(pushButton_grpCloseCallback()));
    QObject::connect(ui.pushButton_cmd_grp_vacuum_on , SIGNAL(clicked()), this, SLOT(pushButton_vacuumGrpOnCallback()));
    QObject::connect(ui.pushButton_cmd_grp_vacuum_off, SIGNAL(clicked()), this, SLOT(pushButton_vacuumGrpOffCallback()));

    QObject::connect(ui.pushButton_set_torque_ratio, SIGNAL(clicked()), this, SLOT(pushButton_setTorqueCallback()));

    if (argc >= 0) {
//        string address_str = argv[2];
        string port = "/dev/ttyUSB0";
//        uint slave_address = stoi(address_str);

        COUT("--------------------------------------");
        COUT("[INFO] Port: ");
//        cout << "[INFO] Slave address #" << slave_address << endl;
        COUT("--------------------------------------");

        if (datc_interface_->init("/dev/ttyUSB0", 1)) {
            timer_ = new QTimer(this);
            connect(timer_, SIGNAL(timeout()), this, SLOT(timerCallback()));
            timer_->start(100); // msec
            success = true;
        } else {
            COUT("[ERROR] Port name or slave address invlaid !");
        }
    } else {
        COUT("--------------------------------------------");
        COUT("[ERROR] Port name & Slave address required !");
        COUT("--------------------------------------------");
    }
}

MainWindow::~MainWindow() {
    if(datc_interface_ != NULL) {
        delete datc_interface_;
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
}

void MainWindow::pushButton_cmdEnableCallback() {
    datc_interface_->motorEnable();
}

void MainWindow::pushButton_cmdDisableCallback() {
    datc_interface_->motorDisable();
}

void MainWindow::pushButton_grpPosCtrlCallback() {
    datc_interface_->setFingerPos(ui.doubleSpinBox_grpPos_ctrl_range_1->value() * 100);
}

void MainWindow::pushButton_grpInitCallback() {
    datc_interface_->grpInitialize();
}

void MainWindow::pushButton_grpOpenCallback() {
    datc_interface_->grpOpen();
}

void MainWindow::pushButton_grpCloseCallback() {
    datc_interface_->grpClose();
}

void MainWindow::pushButton_vacuumGrpOnCallback() {
    datc_interface_->vacuumGrpOn();
}

void MainWindow::pushButton_vacuumGrpOffCallback() {
    datc_interface_->vacuumGrpOff();
}

void MainWindow::pushButton_setTorqueCallback() {
    datc_interface_->setMotorTorque(ui.spinBox_motor_torque_ratio->value());
}

void MainWindow::pushButton_setSpeedCallback() {

}

}   // end of namespace
