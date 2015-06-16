#pragma once

#include "publisher.hpp"
#include "subscriber.hpp"
#include <stdint.h>

namespace ros
{
/**
 * \brief roscpp's interface for creating subscribers, publishers, etc.
 *
 * This class is used for writing nodes. It provides a RAII interface
 * to this process' node, in that when the first NodeHandle is
 * created, it instantiates everything necessary for this node, and
 * when the last NodeHandle goes out of scope it shuts down the node.
 *
 * NodeHandle uses reference counting internally, and coying a
 * NodeHandle is very lightweight.
 *
 * You must call one of the ros::init functions prior to instantiating
 * this class.
 *
 * The most widely used methods are:
 *   - Setup:
 *    - ros::init()
 *   - Publish / subscribe messaging:
 *    - advertise()
 *    - subscribe()
 *   - RPC services:
 *    - advertiseService()
 *    - serviceClient()
 *    - ros::service::call()
 *   - Parameters:
 *    - getParam()
 *    - setParam()
 */
class NodeHandle
{
public:
	/**
	 * \brief Constructor
	 *
	 * When a NodeHandle is constructd, it checks to see if the global
	 * node state has already been started. If so, it increments a
	 * global reference count. If not, it starts the node with
	 * ros::start() and sets the reference count to 1.
	 *
	 * \param ns namespace
	 * \param remappings  Remappings for this NodeHandle.
	 */
	NodeHandle(const char *ns = "",  const char *remappings = "");

	/**
	 * When a NodeHandle is destroyed, it decrements a global reference
	 * count by 1, and if the reference count is now 0, shuts down the
	 * node.
	 */
	~NodeHandle();

	/**
	 * \brief Advertise a topic, simple version
	 *
	 * This call connects to the master to publicize that the node will
	 * be publishing messages on the given topic. This method returns
	 * a Publisher that allows you to publish a message on this topic.
	 *
	 * This version of advertise is a templated convenience function, and
	 * can be used like so.
	 *
	 * ros::Publisher pub = handle.advertise<std_msgs::Empty("my_topic", 1);
	 *
	 * \param topic Topic to advertise on
	 *
	 * \param queue_size Maximum number of outgoing messages to be
	 * queued for delivery to subscribers
	 *
	 * \param latch (optional) If true, the last message published on
	 * this topic will be saved and sent to new subscribers when they
	 * connect
	 *
	 * \return On success, a Publishder that, when it goes out of scope,
	 * will automatically release a reference on the advertisement. On
	 * failure, an empty Publisher.
	 *
	 * \throws InvalidNameException If the topic name begins with a
	 * tilde, or is an otherwise invalid graph resource name, or is an
	 * otherwise invalid graph resource name
	 */
	template <class M>
	Publisher advertise(const char * topic, uint32_t queue_size, bool latch = false) {
		return Publisher();
	}

	template <class M>
	Subscriber subscribe(const char * topic, uint32_t queue_size, void (*callback)(const M* msg)) {
		return Subscriber();
	}

	template <class M, class T>
	Subscriber subscribe(const char * topic, uint32_t queue_size, void (T::*fp)(M), T * obj) {
		return Subscriber();
	}

};

}
