/**
 * @file message_manager.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief : Processes messages sent/received through socket communication.
 * @version 1.0
 * @date 2022-12-29
 *
 * @copyright Copyright (c) 2022 KorasRobotics All Rights Reserved.
 *
 */

#ifndef MESSAGE_MANAGER_HPP
#define MESSAGE_MANAGER_HPP

#include <vector>
#include <algorithm>
#include <unordered_map>

#include "concurrent_queue.hpp"

using namespace std;

namespace tcp_communication {

/**
 * @brief A class for handling messages sent/received via socket communication
 *
 * @details Communication messages to be delivered through TCP communication are
 * processed in the worker queue, and received communication messages are processed
 * in the client queue. In particular, the client queue map assigns a different
 * id to each connection in order to create a 1:1 correspondence to multiple TCP
 * communication connections, and stores entities composed of queues for dedicated
 * communication with the assigned id. Both client queues and worker queues use
 * concurrent queues.
 */
template<typename Data>
class MessageHandler {
public:
    /**
     * @brief If there is no identical TCP connection id in the message handler,
     * a new entity is created with the corresponding id as a key for message reception.
     *
     * @param[in] id An identified key of client queue map
     *
     * @return true If a new entity of client queue map is created
     * @return false If client queue map has the same id
     */
    bool createClientQueue(uint32_t id) {
        if (to_client_queue_map_.find(id) != to_client_queue_map_.end()) {
            return false;
        }

        to_client_queue_map_.insert(make_pair(id, ConcurrentQueue<Data>()));

        return true;
    }

    /**
     * @brief If the id exists in the client queue map, the corresponding entity is deleted.
     *
     * @param[in] id ID of the entity to be deleted
     *
     * @return true If corresponding entity is successfully deleted.
     * @return false If there is no entity corresponding ID of the client queue map to delete
     */
    bool deleteClientQueue(uint32_t id) {
        auto itr = to_client_queue_map_.find(id);

        if (itr == to_client_queue_map_.end()) {
            return false;
        }

        to_client_queue_map_.erase(itr);

        return true;
    }

    /**
     * @brief A function that stores received messages in a worker queue
     *
     * @param[in] data Data to be stored in the worker queue
     */
    void pushToWorkerQueue(Data const &data) {
        to_worker_queue_.push(data);
    }

    /**
     * @brief A function that tries to pop data from worker queue if queue is not empty
     *
     * @param[out] data Data to be tried to be popped off the worker queue
     *
     * @return true If data is popped off from worker queue
     * @return false If worker queue is empty
     */
    bool tryPopFromWokerQueue(Data &data) {
        if (to_worker_queue_.empty()) {
            return false;
        }
        return to_worker_queue_.tryPop(data);
    }

    /**
     * @brief Stores data in the client queue of the client map entity corresponding
     * to all ids in the message handler for the message to be delivered.
     *
     * @param[in] data Queue data to be stored in the client map entity
     * corresponding to all ids in the message handler.
     */
    void pushToAllClientQueue(Data const &data) {
        auto ids = getAllClientId();

        for (auto id : ids) {
            pushToClientQueue(id, data);
        }
    }

    /**
     * @brief A function that stores messages to be delivered to the client queue.
     *
     * @param[in] id ID stored in client queue map
     * @param[in] data Data to be stored in entity corresponding ID of client queue map
     *
     * @return true If data is pushed into entity corresponding ID of client queue map
     * @return false If there is no entity corresponding ID of client queue map
     */
    bool pushToClientQueue(uint32_t id, Data const &data) {
        auto itr = to_client_queue_map_.find(id);

        if (itr == to_client_queue_map_.end()) {
            return false;
        }

        // original
         itr->second.push(data);

        // modified
//        if (itr->second.empty()) {
//            itr->second.push(data);
//        } else {
//            itr->second.clear();
//            itr->second.push(data);
//        }

        return true;
    }

    /**
     * @brief A function that pulls out the first data stored in the client queue
     *
     * @param[in] id ID of client queue map
     * @param[out] data Data to be tried to be popped off
     *
     * @return true If data is popped off from the client queue of the corresponding ID
     * @return false If the data is empty in the client queue
     */
    bool tryPopFromClientQueue(uint32_t id, Data &data) {
        auto itr = to_client_queue_map_.find(id);

        if (itr == to_client_queue_map_.end() || itr->second.empty()) {
            return false;
        }

        return itr->second.tryPop(data);
    }

    /**
     * @brief Returns all IDs that exist in the client queue map
     *
     * @return vector<uint32_t> A vector of all IDs that exist in the client queue map
     */
    vector<uint32_t> getAllClientId() {
        vector<uint32_t> ids;
        for (auto itr = to_client_queue_map_.begin(); itr != to_client_queue_map_.end(); itr++) {
            ids.push_back(itr->first);
        }
        return ids;
    }

private:
    ConcurrentQueue<Data> to_worker_queue_; /**< Worker queue to send data */
    unordered_map<uint32_t, ConcurrentQueue<Data>> to_client_queue_map_; /**< Client queue for storing data */
};

/**
 * @brief Singleton class that inherits Message Handler, creates only one instance,
 * and returns the corresponding pointer.
 */
template<typename Data>
class MessageManager : public MessageHandler<Data> {
private:
    /**
     * @brief Construct a new MessageManager object
     */
    MessageManager() {}
public:
    /**
     * @brief MessageManager class can have only one object or instance at a time.
     *
     * @return MessageManager instance
     */
    static MessageManager &getInstance() {
        static MessageManager *_instance = nullptr;
        if ( _instance == nullptr ) {
            _instance = new MessageManager();
        }
        return *_instance;
    }
};
} // namespace tcp_comm
#endif
