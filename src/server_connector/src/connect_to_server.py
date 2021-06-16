#!/usr/bin/env python
import rospy
import importlib
from connection_manager import ConnectionManager

if __name__ == '__main__':
    rospy.init_node('server_connector')
    rospy.loginfo('Starting!')

    server_ip = rospy.get_param('server_ip', '54.161.15.175')
    server_port = 9090
    name = rospy.get_param('name', 'hexacopter')
    rospy.loginfo('Starting with name ' + name + ' to ip ' + server_ip)
    connection_manager = ConnectionManager(server_ip, server_port, name)
    connection_manager.attempt_connection()

    setup_modules = rospy.get_param('setup_modules')
    for module in setup_modules:
        imported = importlib.import_module(module)
        imported.setup(connection_manager)

    r = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        connection_manager.check_connection()
        r.sleep()

    connection_manager.stop_connection()