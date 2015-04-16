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
This node fuses the 2D and 3D object id with an object name.

It gets the object's id for its 2D and 3D templates
and matches them with an object name given by the user.

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco
"""

import roslib
roslib.load_manifest('ocular_interaction')
import rospy

from std_msgs.msg import String
from ocular.msg import LearningFinished as LFMsg
from ocular_interaction.msg import ObjectDescriptor

_DEFAULT_NAME = 'object_id_notifier'


class ObjectNameIDAggregator(object):

    """
    This node fuses the 2D and 3D object id with an object name.

    It gets the object's id for its 2D and 3D templates
    and matches them with an object name given by the user.
    """

    def __init__(self):
        """Init."""
        super(ObjectNameIDAggregator, self).__init__()
        self.learners = ('2D', '3D')
        self.ids = dict.fromkeys(self.learners)
        self.object_name = None
        self.pub = rospy.Publisher('learned_object', ObjectDescriptor)
        rospy.Subscriber('learning_finished', LFMsg, self.callback)
        rospy.Subscriber('object_name', String, self.object_name_cb)

    def callback(self, msg):
        """Aggregate the object name to the received ids."""
        try:
            self.ids[msg.learner] = msg.object_id
        except KeyError:
            rospy.logwarn("Discarding unknown learner %s.", msg.learner)
            return

        for obj_id in self.ids.values():
            if obj_id is None:
                return

        self.pub.publish(ObjectDescriptor(name=self.object_name,
                                          id2D=self.ids['2D'],
                                          id3D=self.ids['3D']))
        self.ids = dict.fromkeys(self.learners)  # Clear dict values.

    def object_name_cb(self, name):
        """Store object name."""
        self.object_name = name.data

    def run(self):
        """Run the node."""
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node(_DEFAULT_NAME)
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))
        accum = ObjectNameIDAggregator()
        accum.run()
    except rospy.ROSInterruptException:
        pass
