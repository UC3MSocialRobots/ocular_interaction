#!/usr/bin/env python

"""
:author: Victor Gonzalez-Pacheco (VGonPa) vgonzale@ing.uc3m.es

This module provides Action Translators for OCULAR messages
"""

import roslib
roslib.load_manifest('ocular_interaction')

from ocular.msg import EventHandler


def action_args_to_dict(action):
    """ Return action.args in form of a dictionary. """
    if not action.args:
        return None
    d = {arg.name: arg.value for arg in action.args}
    return d


def action_to_event_handler(action):
    """ Convert a dialog_manager_msgs/ActionMsg to ocular/EventHandler.msg."""
    d = action_args_to_dict(action)
    return EventHandler(hand=d.get('hand', None),
                        event=d.get('event', None),
                        last_event=d.get('last_event', None))
