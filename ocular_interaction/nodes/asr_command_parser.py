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
:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco

Node that parses the asr output and converts it to certain commands.
"""
import roslib
roslib.load_manifest('ocular_interaction')
from functools import partial
from itertools import ifilter
from nltk.stem import SnowballStemmer as Stemmer
import rospy
from rospy_utils import coroutines as co
from rospy_utils import param_utils as pu
from asr_msgs.msg import open_grammar_recog_results as ASRmsg
from std_msgs.msg import String
from dialog_manager_msgs.msg import AtomMsg
from ocular_interaction import ocular_atom_translators as tr
from ocular_interaction import utils
atom_logger = partial(utils.log_atom, logger=rospy.logdebug)

translator_pipe = co.pipe([co.transformer(tr.user_command_to_atom),
                           co.do(atom_logger),
                           co.publisher('im_atom', AtomMsg)])

splitter = co.splitter(co.publisher('asr_command', String),
                       translator_pipe)


def get_command(word, commands, stemmer):
    """Check wether word is a command from the commands dictionary."""
    try:
        return commands.get(stemmer.stem(word), None)
    except UnicodeDecodeError:
        rospy.logwarn("UnicodeDecodeError while stemming word: {}".format(word))


def parse_sentence(sentence, commands, stemmer):
    """Parse a sentence and return the first command it founds in it."""
    cmds = (get_command(word, commands, stemmer) for word in sentence.split())
    return ifilter(bool, cmds)


def parse_asr_msg(msg, commands, stemmer):
    """Parse an ASR msg to found a command in it."""
    try:
        return next(parse_sentence(msg.content, commands, stemmer))
    except StopIteration:
        pass


def log_msg(msg):
    """Log a msg to a rospy logger."""
    rospy.logdebug("ASR message parsed to a command as: {}".format(msg))


if __name__ == '__main__':
    stemmer = Stemmer('spanish')
    try:
        rospy.init_node('ocular_event_handler_translator')
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))
        commands = next(pu.load_params('asr_commands'))
        commands_stemmed = {stemmer.stem(k): v for k, v in commands.items()}
        rospy.logdebug("Available commands are: {}".format(commands))
        parse_msg = \
            partial(parse_asr_msg, commands=commands_stemmed, stemmer=stemmer)
        pipe = co.pipe([co.transformer(parse_msg),
                        co.do(log_msg),
                        co.filterer(bool),
                        splitter])
        co.PipedSubscriber('open_grammar_results', ASRmsg, pipe)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
