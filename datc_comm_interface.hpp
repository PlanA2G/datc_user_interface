/**
 * @file datc_comm_interface.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief
 * @version 1.0
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef DATC_COMM_INTERFACE_HPP
#define DATC_COMM_INTERFACE_HPP

#include "datc_ctrl.hpp"
#include <thread>
#include <QThread>
#include <boost/asio.hpp>
#include <jsoncpp/json/json.h>
#include "socket/tcp_manager.hpp"

using namespace std;
using namespace boost::asio;
using namespace boost::asio::ip;

class DatcCommInterface : public QThread, public DatcCtrl {
    Q_OBJECT

public:
    DatcCommInterface(int argc, char **argv);
    virtual ~DatcCommInterface();

Q_SIGNALS:
    void rosShutdown();

public:
    bool init(char *port_name, uint16_t slave_address);
    void initTcp(const string addr, uint16_t socket_port);
    void releaseTcp();

private:
    void run();
    void sendStatus();
    void recvCommand();

    bool flag_stop_ = false;

    // TCP socket related variables
    TcpServer *tcp_server_;
    std::thread tcp_thread_;

    bool flag_tcp_stop_       = false;
    bool is_socket_connected_ = false;

    mutex mutex_tcp_;

};

#endif // DATC_COMM_INTERFACE_HPP
