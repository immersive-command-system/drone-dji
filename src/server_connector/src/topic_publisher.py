#!/usr/bin/env python
import rospy
import roslibpy
from std_srvs.srv import Trigger

class TopicPublisher:

    def __init__(self, namespace, connection):
        self.namespace = namespace
        self.connection = connection
        self.services = []
        self.topics = []
        self.location = 0
        self.advertise_basic_services_topics()

    def advertise_basic_services_topics(self):
        self.advertise_service('/shutdown', 'std_srvs/Trigger'
                               , self.shutdown, include_namespace=True)

    def get_topics(self):
        return rospy.get_published_topics(self.namespace)

    def advertise_service(self, service_name, service_type, handler, include_namespace=False):
        if include_namespace:
            service_name = self.namespace + service_name
        service = roslibpy.Service(self.connection,
                                   service_name,
                                   service_type)
        service.advertise(handler)
        self.services.append(service)

    def publish_topic(self, topic_name, topic_type, topic_type_class, publish_function, include_namespace=False):
        if include_namespace:
            topic_name = self.namespace + topic_name
        publisher = roslibpy.Topic(self.connection, topic_name, topic_type)
        self.topics.append(publisher)

        def publish(data):
            publish_function(publisher, data)

        rospy.Subscriber(topic_name, topic_type_class, publish)

    # ALL PUBLISHERS

    # ALL SERVICE CALLBACKS
    def set_speed(self, request, response):
        rospy.loginfo(request)
        set_speed_service = rospy.ServiceProxy(self.namespace + '/dji_sdk/mission_waypoint_setSpeed', MissionWpSetSpeed)
        local_response = set_speed_service(request.get('basic_mode'), request.get('custom_mode'))
        response['mode_sent'] = local_response.mode_sent
        rospy.loginfo(local_response.mode_sent)
        return True

    def get_speed(self, request, response):
        rospy.loginfo(request)
        get_speed_service = rospy.ServiceProxy(self.namespace + '/dji_sdk/mission_waypoint_getSpeed', MissionWpGetSpeed)
        local_response = get_speed_service(request.get('basic_mode'), request.get('custom_mode'))
        response['mode_sent'] = local_response.mode_sent
        rospy.loginfo(local_response.mode_sent)
        return True

    def shutdown(self, request, response):
        rospy.loginfo(request)
        shutdown_service = rospy.ServiceProxy('/shutdown', Trigger)
        shutdown_service_response = shutdown_service()
        response['success'] = shutdown_service_response.success
        response['message'] = shutdown_service_response.message
        return True

    def unpublish(self):
        for service in self.services:
            service.unadvertise()
        for topic in self.topics:
            topic.unadvertise()