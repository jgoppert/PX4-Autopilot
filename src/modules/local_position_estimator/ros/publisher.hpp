#pragma once

namespace ros
{

	class NodeHandle;

	/**
	 * \brief Manages an advertisement on a specific topic.
	 *
	 * A Publisher should always be created through a call to NodeHandle::advertise(), or copied from one
	 * that was. Once all copies of a specific
	 * Publisher go out of scope, any subscriber status callbacks associated with that handle will stop
	 * being called.  Once all Publishers for a given topic go out of scope the topic will be unadvertised.
	 */
	class Publisher
	{
	public:
		Publisher();
		~Publisher();
		/**
		 * \brief Publish a message on the topic associated with this Publisher.
		 *
		 * This version of publish will allow fast intra-process message-passing in the future,
		 * so you may not mutate the message after it has been passed in here (since it will be
		 * passed directly into a callback function)
		 *
		 */
		template <typename M>
		void publish(const M & message) const {
		}
	private:
		const char * _topic;
		NodeHandle * _node_handle;
	};

}
