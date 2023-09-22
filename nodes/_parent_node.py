#!/usr/bin/env python3

"""
* Node Parent Class *
authors: Lukas Beißner, Jannik Eichmann, Lukas Stefer
copyright: L2J Passing EMR Ltd.
"""

import numpy as np
import rospy as rp


class Node(object):
    def __init__(self, name: str):
        rp.init_node(name, anonymous=True)

    def create_publisher(self, topic_name: str, msg, rate: np.uint8 = 30):
        publisher = rp.Publisher(topic_name, msg, queue_size=rate)
        publishing_rate = rp.Rate(rate)
        print(f"\n[TOPIC] {self.__class__.__name__} < {topic_name} > initialized ...")
        return publisher, publishing_rate

    def publish(self, msg, publisher: rp.Publisher, rate: rp.Rate = None):
        publisher.publish(msg)
        if rate is not None:
            rate.sleep()

    def create_subscriber(self, topic_name: str, msg, callback: callable):
        subscriber = rp.Subscriber(topic_name, msg, callback)
        return subscriber

    def create_service_server(self, service_name: str, srv, handle: callable):
        service_server = rp.Service(service_name, srv, handle)
        print(
            f"\n[SERVICE] {self.__class__.__name__} < {service_name} > ready and listening ..."
        )
        return service_server

    def create_service_client(self, service_name: str, srv, *args, **kwargs):
        rp.wait_for_service(service_name)
        service = rp.ServiceProxy(service_name, srv)
        return service(*args, **kwargs)

    def spin(self):
        """
        Method to keep the ros node active until the is_shutdown() flag is true. This method needs to be called after creating:
        * subscriber nodes
        * service server nodes
        """
        rp.spin()
