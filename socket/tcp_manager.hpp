/**
 * @file tcp_manager.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief : Individual sockets are created to handle the client's TCP communication
 * connection, and messages sent and received through each connected socket
 * @details  1. Socket read operation: The socket asynchronously receives the client's
 * robot command message (json) from the read handler and accumulates it in the worker
 * queue of the message manager, which is a singleton instance. If there are messages
 * accumulated in the queue in the Tcp Receive Handler, an independent thread running
 * concurrently in KCRController, it receives data from the worker queue, parses it
 * into command data, sends the command information to the control interface, and
 * performs a thread sleep for 1 second.
 *
 * 2. Socket write operation: Data to be transmitted for each client is accumulated
 * in the client queue through the socket ID stored when connecting the socket to
 * the message manager's client queue map for each robot update cycle. In a thread
 * that runs independently in the Tcp Manager, messages generated in the message
 * manager are taken out as they are accumulated in the queue, parsed into json
 * format, and sent to clients. Since clients have already been identified through
 * the socket ID given when connected in the message handler's client queue map,
 * it is delivered only to connected clients, and disconnected clients are not
 * delivered because their ID is deleted from the client queue map when the socket
 * is disconnected.
 * @version 1.0
 * @date 2022-12-29
 *
 * @copyright Copyright (c) 2022 KorasRobotics All Rights Reserved.
 *
 */

#ifndef TCP_MANAGER_HPP
#define TCP_MANAGER_HPP

#include <boost/asio.hpp>
#include "message_manager.hpp"

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(WIN64) || defined(_WIN64) || defined(__WIN64__)
#include "../lib/json.h"
#else
#include <jsoncpp/json/json.h>
#endif

using namespace std;
using namespace tcp_communication;

namespace tcp_communication {

/**
 * @brief Performs reading and writing of messages sent and received by communication.
 */
class TcpSocket {
    static constexpr int MAX_BUFFER = 1024; /**< Maximum size of buffer */
public:
    /**
     * @brief Constructor of the TcpSocket object, initiate message handler instance
     * and socket instance managed by io_service
     *
     * @param[in] io_service Io_service that mananges each socket established
     */
    TcpSocket(boost::asio::io_service &io_service);

    /**
     * @brief Destroy the TcpSocket object
     */
    ~TcpSocket();

public:
    /**
     * @brief Get function of socket object
     *
     * @return boost::asio::ip::tcp::socket Socket object
     */
    boost::asio::ip::tcp::socket &getSocket();

    /**
     * @brief Stores the socket's ID in the message handler's client queue map,
     * and starts parallel reading/writing of the socket's messages.
     */
    void start();

    /**
     * @brief Closes the socket connection and deletes the ID associated with
     * the client from the message handler's client queue map.
     */
    void close();

    /**
     * @brief The message (Json Value type) to be delivered to the clients is taken
     * out of the client queue map and the same message is sent to the connected clients.
     */
    void writeHandler();

    /**
     * @brief After parsing the data received from the client into a json buffer,
     * it puts it in the worker queue and reads it again asynchronously. (async read)
     *
     * @param[in] err Boost async read error code
     * @param[in] bytes_transferred Transferred bytes
     */
    void readHandler(const boost::system::error_code& err, size_t bytes_transferred);

private:
    /**
     * @brief get function of socket object
     *
     * @param[out] json Json::Value object for parsing data
     *
     * @return true If succeeds to parse received data
     * @return false If fails to parse received data
     */
    bool parseJsonFromBuffer(Json::Value &json);

private:
    MessageHandler<Json::Value> &message_handler_; /**< Message handler */
    boost::asio::ip::tcp::socket socket_;          /**< Tcp socket */
    string recevied_;                              /**< std::string for json parsing */
    char buffer_[MAX_BUFFER];                      /**< Received data buffer */
};

/**
 * @brief Handles opening and closing of connections of all sockets connected by
 * TCP communication through ports.
 */
class TcpServer {
public:
    /**
     * @brief Constructor of the TcpServer object, connection accept start
     *
     * @param[in] port Port Number
     */
    TcpServer(const int port = 8421);

    /**
     * @brief Destroy the TcpServer object
     */
    ~TcpServer();

public:
    /**
     * @brief Asynchronously executes a connection operation.
     */
    void startAccept();

     /**
     * @brief Start connected socket communication.
     *
     * @param[in] socket Tcpsocket that is establised from io_service
     * @param[in] err Connection error code
     */
    void acceptHandler(TcpSocket *socket, const boost::system::error_code& err);

private:
    boost::asio::io_service io_service_;      /**< Input/output service */
    boost::asio::ip::tcp::acceptor acceptor_; /**< Detect TCP connection from client */
};
} // namespace tcp_comm
#endif
