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
from pattern.es import parse
from enum import Enum
from rospy import loginfo
from functools import partial
import colorama


# LOGGING UTILS ###############################################################
def_slots = {'timestamp', 'consumed', 'subtype', 'type'}


def colorize(logmsg, color=colorama.Fore.MAGENTA):
    """Colorize a message with green colors."""
    return ''.join([color, logmsg, colorama.Fore.RESET])

blue = partial(colorize, color=colorama.Fore.BLUE)
gree = partial(colorize, color=colorama.Fore.GREEN)


def log_atom(msg, logger=loginfo, color=blue):
    """
    Log an AtomMsg.

    :param msg AtomMsg: The message to send to logger
    :param logger: The logger function. Default :py:func:rospy.loginfo
    :param color: function to colororize a string
    :type color: function
    """
    dmsg = {varslot.name: varslot for varslot in msg.varslots}
    atom_name = dmsg['type'].val
    logger(color("----------"))
    logger(color("Translated Atom with Type: ") + "{}".format(atom_name))
    logger(color("The Slots of the Atom are:"))
    for k in dmsg.keys():
        if k not in def_slots:
            log_varslot(dmsg[k], logger=logger)
    logger(color("----------"))


def log_varslot(varslot, logger=loginfo, color=blue):
    """
    Log a VarSlot msg.

    :param varslot VarSlot: The message to send to logger
    :param logger: The logger function. Default :py:func:rospy.loginfo
    :param color: function to colororize a string
    :type color: function
    """
    logger("{} {} -- {} {} -- {} {}"
           .format(color("Name:"), varslot.name,
                   color("Val:"), varslot.val,
                   color("Type:"), varslot.type))

# TTS UTILS ###################################################################
TTS_ENGINES = ('novoice', 'festival', 'google', 'loquendo', 'microsoft',
               'nonverbal', 'music_score', 'pico', 'espeak')
TTSEngine = Enum('TTSEngine', ' '.join(TTS_ENGINES))


# TEXT PARSING UTILS ##########################################################
def tokenize(pos_sentence):
    """
    Tokenize a POS sentence.

    Note: POS is Part-of-Speech Tagging.

    :return: list of [word, *tags]
    :rtype: list of lists

    Example:

        >>>s = parse("esto es una botella.")
        >>> s
        u'esto/DT/O/O es/VB/B-VP/O una/DT/B-NP/O botella/NN/I-NP/O ././O/O'
        >>> list(tokenize(s.split(' ')))
        [[u'esto', u'DT', u'O', u'O'],
         [u'es', u'VB', u'B-VP', u'O'],
         [u'una', u'DT', u'B-NP', u'O'],
         [u'botella', u'NN', u'I-NP', u'O'],
         [u'.', u'.', u'O', u'O']]
    """
    return (word.split('/') for word in pos_sentence)


def get_tagged(tokens, tag):
    """
    Return tokens that have been tagged with tag.

    Example:

        >>>s = parse("esto es una botella.")
        >>> s
        u'esto/DT/O/O es/VB/B-VP/O una/DT/B-NP/O botella/NN/I-NP/O ././O/O'
        >>> list(get_tags(tokenize(s.split(' ')), 'NN'))
        [[u'botella', u'NN', u'I-NP', u'O']]
    """
    return (t for t in tokens if t[1] == tag)

get_nouns = partial(get_tagged, tag='NN')
get_verbs = partial(get_tagged, tag='VB')


def get_first_noun(sentence):
    """Return the first noun that is found in a sentence."""
    s = parse(sentence)
    tokens = tokenize(s.split(' '))
    nouns = zip(*get_nouns(tokens))[0]
    return nouns[0]


# ASR PARSING UTILS ###########################################################

def get_nouns_from_asr(asr_msg):
    """Get all nouns from an ASR message."""
    sentence = parse(asr_msg.content)
    nouns = zip(*get_nouns(tokenize(sentence.split(' '))))[0]
    return nouns
