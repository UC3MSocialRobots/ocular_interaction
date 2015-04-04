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
:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco

Node that detects when a user
"""

import roslib
roslib.load_manifest('ocular_interaction')
import rospy
from rospy_utils import coroutines as co
from ocular_interaction import utils

from ocular.msg import LearningFinished as LFMsg
from ocular_interaction.msg import ObjectDescriptor
from asr_msgs.msg import open_grammar_recog_results as ASRMsg

_DEFAULT_NAME = 'object_id_notifier'


class ObjectIDAccumulator(object):

    """docstring for ObjectIDAccumulator."""

    def __init__(self, arg):
        """Init."""
        super(ObjectIDAccumulator, self).__init__()
        self.learners = ('2D', '3D')
        self.ids = dict.fromkeys(self.learners)
        self.object_name = None
        self.pub = rospy.publisher('named_object', ObjectDescriptor)
        rospy.Subscriber('learning_finished', LFMsg, self.callback)
        rospy.Subscriber('open_grammar_results', ASRMsg, self.asr_callback)

    def callback(self, msg):
        """Check if action_msg is valid and send it to the loggers."""
        try:
            self.ids[msg.learner] = msg.object_id
        except KeyError:
            rospy.logwarn("Discarding unknown learner %s.", msg.learner)
            return

        for learner, obj_id in self.ids.iteritems():
            if obj_id is None:
                return

        self.pub.publish(ObjectDescriptor(name=self.object_name,
                                          id2D=self.ids['2D'],
                                          id3D=self.ids['3D']))
        self.ids = dict.fromkeys(self.learners)  # Clear dict values.

    def asr_callback(self, asrmsg):
        """Store object name from an asr sentence."""
        self.object_name = utils.get_nouns_from_asr(asrmsg)[0]

    def run(self):
        """Run the node."""
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node(_DEFAULT_NAME)
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))
        accum = ObjectIDAccumulator()
        accum.run()
    except rospy.ROSInterruptException:
        pass
