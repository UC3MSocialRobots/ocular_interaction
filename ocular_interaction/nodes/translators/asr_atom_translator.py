#!/usr/bin/env python

# :copyright:    Copyright (C) 2015 Universidad Carlos III de Madrid.
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
Node that translates ASR Messages to AtomMsgs.

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco
"""

import roslib
roslib.load_manifest('ocular_interaction')
import rospy
from rospy_utils import coroutines as co

from ocular_interaction import ocular_atom_translators as tr
from ocular_interaction import utils

from dialog_manager_msgs.msg import AtomMsg
from asr_msgs.msg import open_grammar_recog_results as ASRMsg


if __name__ == '__main__':
    try:
        rospy.init_node('ocular_asr_translator')
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))
        pipe = co.pipe([co.transformer(tr.asr_msg_to_atom),
                        co.do(utils.log_atom),
                        co.publisher('im_atom', AtomMsg)])
        co.PipedSubscriber('open_grammar_results', ASRMsg, pipe)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
