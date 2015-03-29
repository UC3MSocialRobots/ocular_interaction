#!/usr/bin/env python
# :version:      0.0.0
# :copyright:    Copyright (C) 2014 Universidad Carlos III de Madrid.
#                Todos los derechos reservados.
# :license       LASR_UC3M v1.0, ver LICENCIA.txt

# Este programa es software libre: puede redistribuirlo y/o modificarlo
# bajo los terminos de la Licencia Academica Social Robotics Lab - UC3M
# publicada por la Universidad Carlos III de Madrid, tanto en su version 1.0
# como en una version posterior.

# Este programa se distribuye con la intencion de que sea util,
# pero SIN NINGUNA GARANTIA. Para mas detalles, consulte la
# Licencia Academica Social Robotics Lab - UC3M version 1.0 o posterior.

# Usted ha recibido una copia de la Licencia Academica Social
# Robotics Lab - UC3M en el fichero LICENCIA.txt, que tambien se encuentra
# disponible en <URL a la LASR_UC3Mv1.0>.

"""
Node that translates log messages from Interaction Manager to ROS Log messages.
"""
import roslib
roslib.load_manifest('ocular_interaction')
import rospy
from rospy import (logdebug, loginfo, logwarn, logerr, logfatal)

from ocular_interaction import ocular_action_translators as tr
from dialog_manager_msgs.msg import ActionMsg


_DEFAULT_NAME = 'dialog_logger_node'
_ACTOR_NAME = 'ros_logger'
_ACTION_NAME = 'log'

_LOGGERS = {'logdebug': logdebug,
            'loginfo': loginfo,
            'logwarn': logwarn,
            'logerr': logerr,
            'logfatal': logfatal}


def __check_action_msg(action_msg):
    """ Validate an action_msg. """
    if action_msg.name != _ACTION_NAME:
        logdebug("DM Logger Node: Action name is {}. I Only process action: {}"
                 .format(action_msg.name, _ACTION_NAME))
        return False
    if action_msg.actor != _ACTOR_NAME:
        logdebug("DM Logger Node: action {} is for actor {}. Discarding"
                 .format(action_msg.name, action_msg.actor))
        return False
    if not action_msg.args:
        logdebug("DM Logger Node: Action Message has empty args.")
        return False
    return True


def callback(action_msg):
    """ Check if action_msg is valid and send it to the loggers. """
    if not __check_action_msg(action_msg):
        return
    msg = tr.action_args_to_dict(action_msg)
    logger = _LOGGERS[msg['logger']]
    logger(msg['message'])


if __name__ == '__main__':
    try:
        rospy.init_node(_DEFAULT_NAME)
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))
        rospy.Subscriber('im_action', ActionMsg, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
