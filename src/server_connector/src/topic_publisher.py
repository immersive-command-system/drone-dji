#!/usr/bin/env python
import rospy
import roslibpy
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix
from dji_sdk.srv import MissionWpSetSpeed
from dji_sdk.srv import MissionWpGetSpeed
from dji_sdk.srv import MissionWpAction
from dji_sdk.srv import DroneTaskControl

class TopicPublisher:

    def __init__(self, namespace, connection):
        self.namespace = namespace
        self.connection = connection
        self.services = []
        self.topics = []
        self.location = 0
        self.advertise_basic_services_topics()

    def advertise_basic_services_topics(self):
        self.advertise_service('/dji_sdk/mission_waypoint_setSpeed', 'dji_sdk/MissionWpSetSpeed',
                               self.set_speed, include_namespace=True)
        self.advertise_service('/dji_sdk/mission_waypoint_getSpeed', 'dji_sdk/MissionWpGetSpeed',
                               self.get_speed, include_namespace=True)
        self.advertise_service('/dji_sdk/mission_waypoint_action', 'dji_sdk/MissionWpAction',
                               self.mission_waypoint_action, include_namespace=True)
        self.advertise_service('/dji_sdk/drone_task_control', 'dji_sdk/DroneTaskControl',
                               self.drone_task_control, include_namespace=True)
        self.publish_topic('/dji_sdk/gps_position', 'sensor_msgs/NavSatFix', NavSatFix
                           , self.gps_publisher, include_namespace=True)
        self.advertise_service('/shutdown', 'std_srvs/Trigger',
                               self.shutdown, include_namespace=True)

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
    def gps_publisher(self, publisher, data):
        publisher.publish({'latitude': data.latitude, 'longitude': data.longitude, 'altitude': data.altitude})

        # DONE FOR TAKEOFF LOCATION
        self.location = data

    # ALL SERVICE CALLBACKS
    def set_speed(self, request, response):
        rospy.loginfo(request)
        set_speed_service = rospy.ServiceProxy(self.namespace + '/dji_sdk/mission_waypoint_setSpeed',
                                               MissionWpSetSpeed)
        local_response = set_speed_service(request.get('speed'))
        response['result'] = local_response.result
        rospy.loginfo(local_response.result)
        return True

    def get_speed(self, request, response):
        rospy.loginfo(request)
        get_speed_service = rospy.ServiceProxy(self.namespace + '/dji_sdk/mission_waypoint_getSpeed',
                                               MissionWpGetSpeed)
        local_response = get_speed_service()
        response['speed'] = local_response.speed
        rospy.loginfo(local_response.speed)
        return True

    def mission_waypoint_action(self, request, response):
        rospy.loginfo(request)
        mission_waypoint_action_service = rospy.ServiceProxy(self.namespace + '/dji_sdk/mission_waypoint_action',
                                                             MissionWpAction)
        local_response = mission_waypoint_action_service(request.get('action'))
        response['result'] = local_response.result
        rospy.loginfo(local_response.result)
        return True

    def drone_task_control(self, request, response):
        rospy.loginfo(request)
        drone_task_control_service = rospy.ServiceProxy(self.namespace + '/dji_sdk/drone_task_control',
                                                        DroneTaskControl)
        local_response = drone_task_control_service(request.get('task'))
        response['result'] = local_response.result
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
