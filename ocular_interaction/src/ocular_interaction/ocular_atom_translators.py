#!/usr/bin/env python

"""
This module provides Atom Translators for OCULAR messages.

:author: Victor Gonzalez-Pacheco (VGonPa) vgonzale@ing.uc3m.es

Attributes:
-----------
varslotters: Dict that maps a generic slot generator function to a ROS msg type.
"""

import roslib
roslib.load_manifest('ocular_interaction')
from itertools import chain
from functools import partial

from dialog_manager_msgs.msg import (VarSlot, AtomMsg)


###############################################################################
# Slot translators
###############################################################################

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


def slot_from_string(string, slotname, slottype='string'):
    """
    Produce a VarSlot from a string.

    Example:

        >>> next(slot_from_string('the_slot_value', 'the_slot_name'))
        name: the_slot_name
        relation: ''
        val: the_slot_value
        type: string
        unique_mask: False
    """
    yield VarSlot(name=slotname, val=string, type=slottype)


def slot_from_num(num, slotname, slottype='number'):
    """
    Produce a VarSlot from an num.

    It accepts int and float.

    Example:

        >>> next(slot_from_num(123, 'an_int_slot'))
        name: an_int_slot
        relation: ''
        val: 123
        type: number
        unique_mask: False

        >>> next(slot_from_num(0.12345, 'a_float_slot'))
        name: a_float_slot
        relation: ''
        val: 0.12345
        type: number
        unique_mask: False
    """
    yield VarSlot(name=slotname, val=str(num), type=slottype)


def slot_from_bool(b, slotname, slottype='string'):
    """
    Produce a VarSlot from a boolean.

    Example:
        >>> next(slot_from_bool(False, 'a_boolean_slot'))
        name: a_boolean_slot
        relation: ''
        val: false
        type: string
        unique_mask: False
    """
    yield VarSlot(name=slotname, val=str(b).lower(), type=slottype)


basic_type_parsers = {'bool': slot_from_bool,
                      'string': slot_from_string,
                      'int16': slot_from_num,
                      'int32': slot_from_num,
                      'int64': slot_from_num,
                      'float16': slot_from_num,
                      'float32': slot_from_num,
                      'float64': slot_from_num}


def get_slot_parser(slot_type):
    """Return a slot parser function associated to slot_type."""
    try:
        return basic_type_parsers[slot_type]
    except KeyError:
        return parse_array


def parse_array(arr, slotname, slottype='string'):
    """parse an array."""
    slot_value = '|'.join(str(item) for item in arr)
    yield VarSlot(name=slotname, val=slot_value, type=slottype)


def _get_msg_fields(msg):
    """Return all message field names and types except its header."""
    try:
        getattr(msg, 'header')
    except AttributeError:
        return zip(msg.__slots__, msg._slot_types)
    return zip(msg.__slots__[1:], msg._slot_types[1:])


def msg_to_slots(msg):
    """Convert msg fields to VarSlots.

    At this point only bool, string, int*, and float* types are supported.

    Examples:
    --------
    >>> from std_msgs.msg import Bool
    >>> list(msg_to_slots(Bool()))
    [name: data
    relation: ''
    val: false
    type: string
    unique_mask: False]

    >>> from std_msgs.msg import ColorRGBA
    >>> list(msg_to_slots(ColorRGBA()))
    [name: r
    relation: ''
    val: 0.0
    type: number
    unique_mask: False, name: g
    relation: ''
    val: 0.0
    type: number
    unique_mask: False, name: b
    relation: ''
    val: 0.0
    type: number
    unique_mask: False, name: a
    relation: ''
    val: 0.0
    type: number
    unique_mask: False]
    """
    for sname, stype in _get_msg_fields(msg):
        # parser = varslotters[stype]
        parser = get_slot_parser(stype)
        yield next(parser(msg.__getattribute__(sname), slotname=sname))


def generate_event_handler_slots(event_msg):
    """Generate the slots for the OCULAR's event_handler_atom."""
    yield VarSlot(name="hand", val=event_msg.hand, type="string")
    yield VarSlot(name="event", val=event_msg.event, type="string")
    yield VarSlot(name="last_event", val=event_msg.last_event, type="string")


def generate_user_command_slots(user_command):
    """Generate the slots for the OCULAR's user_command atom."""
    yield VarSlot(name="command", val=user_command, type="string")


def generate_google_asr_slots(asr_msg):
    """Generate the slots for the Atoms corresponding Goolge ASR messages."""
    yield VarSlot(name="content", val=asr_msg.content, type="string")
    yield VarSlot(name="confidence", val=str(asr_msg.confidence), type="number")
    yield VarSlot(name="languageID", val=str(asr_msg.languageID), type="number")


def generate_object_name_slots(string_msg):
    """Generate the slots for the object_name Atom."""
    yield VarSlot(name="object_name", val=string_msg.data, type="string")


def generate_learned_object_slots(learned_msg):
    """Generate the slots for the ObjectDescriptor Messages."""
    yield VarSlot(name="object", val=learned_msg.name, type="string")
    yield VarSlot(name="id2D", val=str(learned_msg.id2D), type="number")
    yield VarSlot(name="id3D", val=str(learned_msg.id3D), type="number")


###############################################################################
# Atom translators
###############################################################################

def to_atom_msg(msg, generator_func, atom_name,
                atom_subtype='user', *args, **kwargs):
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
    varslots = chain(def_slots, generator_func(msg, *args, **kwargs))
    return AtomMsg(varslots=list(varslots))


def event_handler_to_atom(event_handler_msg):
    """Generate an OCULAR's event_handler atom."""
    return to_atom_msg(event_handler_msg,
                       generate_event_handler_slots,
                       'event_handler')


def user_command_to_atom(command):
    """Generate an OCULAR's user_command atom."""
    return to_atom_msg(command, generate_user_command_slots, 'user_command')


def asr_msg_to_atom(asr_msg):
    """Generate an OCULAR's asr atom."""
    return to_atom_msg(asr_msg, generate_google_asr_slots, 'asr_googleOK')


def asr_first_noun_to_atom(string):
    """Generate an OCULAR's first_noun Atom."""
    return to_atom_msg(string, slot_from_string, 'first_noun',
                       slotname='first_noun')


def object_name_to_atom(object_name):
    """Generate an OCULAR's object_name Atom."""
    return to_atom_msg(object_name, generate_object_name_slots, 'object_name')


def learned_object_to_atom(obj_msg):
    """Generate an OCULAR's learned_object atom."""
    return to_atom_msg(obj_msg, generate_learned_object_slots, 'object_learned')


def prediction_to_atom(prediction_msg):
    """Generate an OCULAR's prediction atom."""
    return to_atom_msg(prediction_msg, msg_to_slots,
                       atom_name='prediction', atom_subtype='user')

