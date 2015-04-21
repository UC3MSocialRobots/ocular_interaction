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
Node that parses the asr output and converts it to certain commands.

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco
"""
import roslib; roslib.load_manifest('ocular_interaction')
from functools import partial
from itertools import ifilter
import rospy
from rospy_utils import coroutines as co
from rospy_utils import param_utils as pu
from asr_msgs.msg import open_grammar_recog_results as ASRmsg
from std_msgs.msg import String
from dialog_manager_msgs.msg import AtomMsg
from ocular_interaction import ocular_atom_translators as tr
from ocular_interaction import utils

atom_logger = partial(utils.log_atom, logger=rospy.loginfo)

translator_pipe = co.pipe([co.transformer(tr.user_command_to_atom),
                           co.do(atom_logger),
                           co.publisher('im_atom', AtomMsg)])

splitter = co.splitter(co.publisher('asr_command', String),
                       translator_pipe)


def get_command(word, commands):
    """Check wether word is a command from the commands dictionary."""
    try:
        return commands.get(utils.to_infinitive(word), None)
    except UnicodeDecodeError:
        rospy.logwarn("UnicodeDecodeError while stemming word: {}".format(word))


def parse_sentence(sentence, commands):
    """Parse a sentence and return the first command it founds in it."""
    rospy.loginfo("Going to parse sentence: {}".format(utils.blue(sentence)))
    rospy.logdebug("With type: {}".format(utils.blue(str(type(sentence)))))
    cmds = (get_command(word, commands) for word in sentence.split())
    return ifilter(bool, cmds)


def parse_asr_msg(msg, commands):
    """Parse an ASR msg to found a command in it."""
    try:
        return next(parse_sentence(msg.content, commands))
    except StopIteration:
        pass


def _log_msg(msg):
    """Log a msg to a rospy logger."""
    rospy.loginfo(utils.colorize("ASR msg parsed to command: {}".format(msg)))


if __name__ == '__main__':
    try:
        rospy.init_node('asr_command_parser')
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))
        commands = next(pu.load_params('asr_commands'))
        rospy.logdebug("Available commands are: {}".format(commands))
        parsed_cmds = {utils.to_infinitive(k): v for k, v in commands.items()}
        parse_msg = partial(parse_asr_msg, commands=parsed_cmds)
        pipe = co.pipe([co.transformer(parse_msg),
                        co.do(_log_msg),
                        co.filterer(bool),
                        splitter])
        co.PipedSubscriber('open_grammar_results', ASRmsg, pipe)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
