#!/usr/bin/env python

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
Several utilities for the OCULAR Interaction packages.

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco
"""

import roslib
roslib.load_manifest('monarch_multimodal_fusion')
# import rospy
from rospy import loginfo
from enum import Enum

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

# TTS UTILS ###################################################################
TTS_ENGINES = ('novoice', 'festival', 'google', 'loquendo', 'microsoft',
               'nonverbal', 'music_score', 'pico', 'espeak')
TTSEngine = Enum('TTSEngine', ' '.join(TTS_ENGINES))
