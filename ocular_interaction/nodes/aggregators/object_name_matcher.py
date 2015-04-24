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
Matches the predicted object ids to its corresponding names.

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco
"""

import roslib
roslib.load_manifest('ocular_interaction')

import sys
import rospy

from ocular_interaction import utils
from ocular_interaction import object_database_manager as odbm

from std_msgs.msg import String
from ocular.msg import SystemOutput


class ObjectNameMatcher(object):

    """
    Node that bulds an Object Database from ObjectDescriptor Messages.

    The database can be stored to a given filename or it is automatically
    saved to `db_filename` when the node is shutdown (via rospy.shutdown).
    """

    def __init__(self, db_filename):
        """
        Constructor.

        Args:
            db_filename (str): Filename of the Object DB to load.
        """
        super(ObjectNameMatcher, self).__init__()
        self.db_filename = db_filename
        self.db = odbm.ObjectDBHelper(self.db_filename)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('ocular/final_object_id', SystemOutput, self.callback)
        self.pub = rospy.Publisher('recognized_object_name', String)

    def callback(self, msg):
        """Publish the name of the received object_id."""
        matched_name = self.seek_object_name(msg.id_2d_plus_3d)
        rospy.loginfo("Predicted Object (ID: {}, Name: {})"
                      .format(utils.green(str(msg.id_2d_plus_3d)),
                              utils.green(matched_name)))
        self.pub.publish(matched_name)

    def seek_object_name(self, object_id):
        """Seek object_id in object DB and returns its name."""
        for name, ids_rgb, ids_pcloud in self.db.items():
            # rospy.logwarn("ObjectID: {}".format(object_id))
            # rospy.logwarn("DB Entry:")
            # rospy.logwarn("Name {} -- IDs RGB {} -- IDs Pcloud {}"
            #               .format(name, ids_rgb, ids_rgb))
            if any([object_id in ids_rgb, object_id in ids_pcloud]):
                return name
        return 'NOT_FOUND'

    def shutdown(self):
        """Hook to be executed when rospy.shutdown is called."""
        pass

    def run(self):
        """Run the node."""
        rospy.spin()


_DEFAULT_NAME = 'object_database_builder'

if __name__ == '__main__':
    db_filename = rospy.myargv(argv=sys.argv)[1]
    rospy.loginfo("Loaded Object Database file from: {}"
                  .format(utils.blue(db_filename)))
    try:
        rospy.init_node(_DEFAULT_NAME)
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))
        matcher = ObjectNameMatcher(db_filename)
        matcher.run()
    except rospy.ROSInterruptException:
        pass
