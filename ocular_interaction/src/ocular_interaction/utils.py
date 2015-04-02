#!/usr/bin/env python

import roslib
roslib.load_manifest('monarch_multimodal_fusion')
# import rospy
from rospy import loginfo


# LOGGING UTILS ###############################################################
def_slots = {'timestamp', 'consumed', 'subtype', 'type'}


def log_atom(msg, logger=loginfo):
    """
    Log an AtomMsg.


    :param msg AtomMsg: The message to send to logger
    :param logger: The logger function. Default :py:func:rospy.loginfo
    """
    dmsg = {varslot.name: varslot for varslot in msg.varslots}
    atom_name = dmsg['type'].val
    logger("----------")
    logger("Translated Atom with Type {}".format(atom_name))
    logger("Slots of {} Atom are:".format(atom_name))
    for k in dmsg.keys():
        if k not in def_slots:
            log_varslot(dmsg[k], logger=logger)
    logger("----------")


def log_varslot(varslot, logger=loginfo):
    """
    Log a VarSlot msg.

    :param varslot VarSlot: The message to send to logger
    :param logger: The logger function. Default :py:func:rospy.loginfo
    """
    logger("Name: {} -- Val: {} -- Type: {}"
           .format(varslot.name, varslot.val, varslot.type))
