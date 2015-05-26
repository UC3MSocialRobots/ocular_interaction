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
Creates and maintains a database of object names and IDs.

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco

The database automatically saved to `db_filename` when
the node is shutdown via rospy.shutdown.
"""

import roslib
roslib.load_manifest('ocular_interaction')

import sys
# import yaml
import rospy
# from collections import defaultdict
from ocular_interaction import utils
from ocular_interaction import object_database_manager as odbm

from std_msgs.msg import Bool
from ocular_msgs.msg import ObjectDescriptor


class ObjectDBNode(object):

    """
    Node that bulds an Object Database from ObjectDescriptor Messages.

    The database can be stored to a given filename or it is automatically
    saved to `db_filename` when the node is shutdown (via rospy.shutdown).

    Subscribes: 'learned_object' (ocular_msgs/ObjectDescriptor)
    Publishes 'object_database_updated' (std_msgs/Bool)

    """

    def __init__(self, db_filename):
        """Init."""
        super(ObjectDBNode, self).__init__()
        self.db_filename = db_filename
        self.db = odbm.ObjectDBHelper(self.db_filename)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('learned_object', ObjectDescriptor, self.callback)
        self.pub = rospy.Publisher('object_database_updated', Bool)

    def callback(self, msg):
        """
        Get an object and store it in the DB.

        The updated version of the DB is kept both in memory and in file called
        `<db_filename>.autosave`.
        """
        self.db.add(msg.name, msg.id2D, msg.id3D)
        self.db.save(self.db_filename + ".autosave")

    def save_db(self, filename):
        """Save the database to the given filename."""
        rospy.loginfo("Saving Object DB to {}".format(utils.blue(filename)))
        self.db.save(filename)
        self.pub.publish(True)
        return self

    def shutdown(self):
        """Hook to be executed when rospy.shutdown is called."""
        self.save_db(self.db_filename)
        return self

    def run(self):
        """Run the node."""
        rospy.spin()

_DEFAULT_NAME = 'object_database_builder'

if __name__ == '__main__':
    db_filename = rospy.myargv(argv=sys.argv)[1]
    rospy.loginfo("Object Database file is: {}"
                  .format(utils.blue(db_filename)))
    try:
        rospy.init_node(_DEFAULT_NAME)
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))
        db_node = ObjectDBNode(db_filename)
        db_node.run()
    except rospy.ROSInterruptException:
        pass
