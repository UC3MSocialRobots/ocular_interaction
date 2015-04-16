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

Node that performs POS tagging to the ASR Output.
"""
import roslib
roslib.load_manifest('ocular_interaction')
from functools import partial
from nltk.stem import SnowballStemmer as Stemmer
from pattern.es import parse
import rospy
from rospy_utils import coroutines as co
from rospy_utils import param_utils as pu
from asr_msgs.msg import open_grammar_recog_results as ASRmsg
from std_msgs.msg import String
from dialog_manager_msgs.msg import AtomMsg
from ocular_interaction import ocular_atom_translators as tr
from ocular_interaction import utils

atom_logger = partial(utils.log_atom, logger=rospy.loginfo)

translator_pipe = co.pipe([co.transformer(tr.asr_first_noun_to_atom),
                           co.do(atom_logger),
                           co.publisher('im_atom', AtomMsg)])

splitter = co.splitter(co.publisher('nouns_from_asr', String),
                       translator_pipe)

get_asr_content = partial(getattr, name='content')


def _log_msg(msg):
    """Log a msg to a rospy logger."""
    rospy.loginfo(utils.colorize("First noun of ASR msg: {}".format(msg)))


if __name__ == '__main__':
    stemmer = Stemmer('spanish')
    try:
        rospy.init_node('asr_noun_parser')
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))

        pipe = co.pipe([co.transformer(get_asr_content),
                        co.transformer(utils.get_first_noun),
                        co.do(_log_msg),
                        co.filterer(lambda s: s != '_UNKNOWN_'),
                        co.splitter(co.publisher('nouns_from_asr', String),
                                    translator_pipe)])
        co.PipedSubscriber('open_grammar_results', ASRmsg, pipe)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
