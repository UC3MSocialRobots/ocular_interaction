#!/usr/bin/env python

"""
:author: Victor Gonzalez-Pacheco (VGonPa) vgonzale@ing.uc3m.es

This module provides Action Translators for OCULAR messages
"""

import roslib
roslib.load_manifest('ocular_interaction')
from rospy import logdebug, logwarn
from ocular.msg import EventHandler


def should_process_action(action_msg, actor_name, action_name):
    """
    Check if an action should be processed by an action translator.

    :param action_msg: The action message to be checked
    :param actor_name: Name of the expected actor checked against action.actor
    :param action_name: Name of the expected action checked against action.name
    :return: True if action.name and action.actor are the expected ones
    :rtype: bool
    """
    if action_msg.name != action_name:
        logdebug("DM Logger Node: Action name is {}. I Only process action: {}"
                 .format(action_msg.name, action_name))
        return False
    if action_msg.actor != actor_name:
        logdebug("DM Logger Node: action {} is for actor {}. Discarding"
                 .format(action_msg.name, action_msg.actor))
        return False
    if not action_msg.args:
        logdebug("DM Logger Node: Action Message has empty args.")
        return False
    return True


def action_args_to_dict(action):
    """Return action.args in form of a dictionary."""
    if not action.args:
        return None
    d = {arg.name: arg.value for arg in action.args}
    return d


def action_to_event_handler(action):
    """Convert a dialog_manager_msgs/ActionMsg to ocular/EventHandlermsg."""
    d = action_args_to_dict(action)
    logwarn("Received event from iwaki: %s", d)
    return EventHandler(hand=d.get('hand', None),
                        event=d.get('event', None),
                        last_event=d.get('last_event', None))
