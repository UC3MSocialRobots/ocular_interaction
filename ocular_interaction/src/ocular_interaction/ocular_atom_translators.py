#!/usr/bin/env python

"""
:author: Victor Gonzalez-Pacheco (VGonPa) vgonzale@ing.uc3m.es

This module provides Atom Translators for OCULAR messages
"""

import roslib
roslib.load_manifest('ocular_interaction')
from itertools import chain
from dialog_manager_msgs.msg import (VarSlot, AtomMsg)


def generate_default_slots(atom_type='_NO_VALUE_', atom_subtype='user'):
    """
    Generate the slots that are common to every Atom.

    :param str atom_type: Type of the atom
    :param str atom_subtype: Subtype of te atom. Defaults to 'user'
    :return: Yields the default VarSlots
    :rtype: VarSlot
    """
    yield VarSlot(name="type", val=atom_type)
    yield VarSlot(name="subtype", val=atom_subtype)
    yield VarSlot(name="timestamp", val="_NO_VALUE_", type="number")
    yield VarSlot(name="consumed", val="false", type="string")


def to_atom_msg(msg, generator_func, atom_name, atom_subtype='user'):
    """
    Return an AtomMsg from a msg instance and a generator function.

    :param msg: The message to translate
    :param function generator_func: The function to translate the message
    :param str atom_name: Name of the atom
    :param str atom_subtype: (default: 'user') Name of the atom subtype
    :return: The msg converted to an AtomMsg
    :rtype: AtomMsg
    """
    if not msg:
        return AtomMsg()
    def_slots = generate_default_slots(atom_name, atom_subtype)
    varslots = chain(def_slots, generator_func(msg))
    return AtomMsg(varslots=list(varslots))


def generate_event_handler_slots(event_msg):
    """ Generate the slots for the OCULAR's event_handler_atom. """
    yield VarSlot(name="hand", val=event_msg.hand, type="string")
    yield VarSlot(name="event", val=event_msg.event, type="string")
    yield VarSlot(name="last_event", val=event_msg.last_event, type="string")


def generate_user_command_slots(user_command):
    """ Generate the slots for the OCULAR's user_command atom. """
    yield VarSlot(name="command", val=user_command, type="string")


def event_handler_to_atom(event_handler_msg):
    """ Generate an OCULAR's event_handler atom. """
    return (to_atom_msg(event_handler_msg, generate_event_handler_slots,
            'event_handler'))


def user_command_to_atom(command):
    """ Generate an OCULAR's user_command atom. """
    return (to_atom_msg(command, generate_user_command_slots, 'user_command'))
