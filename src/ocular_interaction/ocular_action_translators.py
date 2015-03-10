#!/usr/bin/env python

"""
:author: Victor Gonzalez-Pacheco (VGonPa) vgonzale@ing.uc3m.es

This module provides Action Translators for OCULAR messages
"""

import roslib
roslib.load_manifest('ocular_interaction')

from ocular.msg import EventHandler


def action_to_event_handler(action):
    """ Converts a dialog_manager_msgs/ActionMsg to ocular/EventHandler.msg."""
    d = {arg.name: arg.value for arg in action.args}
    return EventHandler(hand=d['hand'], event=d['event'],
                        last_event=d['last_event'])
