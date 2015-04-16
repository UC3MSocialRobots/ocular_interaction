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
Node that translates ActionMsg messages to Utterance Messages.

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco
"""
import roslib
roslib.load_manifest('ocular_interaction')

from functools import partial

import rospy
from rospy_utils import coroutines as co

from ocular_interaction import ocular_action_translators as tr
from dialog_manager_msgs.msg import ActionMsg
from etts_msgs.msg import Utterance


is_action_valid = partial(tr.should_process_action,
                          actor_name="etts",
                          action_name="say_text")

if __name__ == '__main__':
    try:
        rospy.init_node('ocular_etts_translator')
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))
        pipe = co.pipe([co.filterer(is_action_valid),
                        co.transformer(tr.action_to_etts),
                        co.publisher('etts', Utterance)])
        co.PipedSubscriber('im_action', ActionMsg, pipe)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
