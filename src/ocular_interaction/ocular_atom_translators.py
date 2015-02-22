#!/usr/bin/env python

"""
    :author: Victor Gonzalez-Pacheco (VGonPa) vgonzale@ing.uc3m.es

    This module provides Atom Translators for OCULAR messages
"""

import roslib
roslib.load_manifest('ocular_interaction')

from monarch_multimodal_fusion import translators
from dialog_manager_msgs.msg import VarSlot


def generate_event_handler_slots(event_msg):
    """ Generates the slots for the OCULAR's event_handler_atom """
    yield VarSlot(name="hand", val=event_msg.hand, type="string")
    yield VarSlot(name="event", val=event_msg.event, type="string")
    yield VarSlot(name="last_event", val=event_msg.last_event, type="string")


def event_handler_to_atom(event_handler_msg):
    """ Generates an OCULAR's event_handler atom """
    return (translators.to_atom_msg(event_handler_msg,
                                    generate_event_handler_slots,
                                    'event_handler'))
