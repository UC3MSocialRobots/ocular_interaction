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

import roslib; roslib.load_manifest('ocular_interaction')

import sys
# import yaml
import rospy
# from collections import defaultdict
from ocular_interaction import utils
from ocular_interaction import object_database_manager as odbm

from ocular_interaction.msg import ObjectDescriptor


# def _db_to_defaultdict(db):
#     """Transform the type of the dabase subdicts to defaultdict."""
#     return {'2D': defaultdict(list, db['2D']),
#             '3D': defaultdict(list, db['3D'])}


# def _db_to_dict(db):
#     """Convert the database to dict."""
#     return {'2D': dict(db['2D']),
#             '3D': dict(db['3D'])}


# class ObjectDBHelper(object):

#     """Loads and stores the ObjectDatabase to YAML file."""

#     def __init__(self, db_filename):
#         """Init."""
#         super(ObjectDBHelper, self).__init__()
#         self.db_filename = db_filename
#         self.load(self.db_filename)

#     def add(self, name, id2D, id3D):
#         """Add an entry to the database."""
#         self.db['2D'][name].append(id2D)
#         self.db['3D'][name].append(id3D)
#         return self

#     def get(self, key, *args, **kwargs):
#         """
#         Return an entry from the DB with similar syntax to 'dict.get'.

#         :param key: the key of the values to retrieve.
#         :type key: str
#         :return: tuple: (key, value2D, value3D)

#         Example:

#             >>> db = ObjectDBHelper('/tmp/example_db')
#             >>> db.add('A', 1, 1)
#             >>> db.add('B', 2, 2)
#             >>> db.get('A')
#             ('A', [1], [1])
#             >>> db.get('B')
#             ('B', [2], [2])
#             >>> db.get('C', 'ITEM NOT FOUND')
#             ('C', 'ITEM NOT FOUND', 'ITEM NOT FOUND')
#         """
#         return (key,
#                 self.db['2D'].get(key, *args, **kwargs),
#                 self.db['3D'].get(key, *args, **kwargs))

#     def __getitem__(self, key):
#         """
#         Retrieve an item in a dict-like syntax.

#         :param key: the key of the values to retrieve.
#         :type key: str
#         :return: tuple: (key, value2D, value3D)

#         Example:
#             >>> db = ObjectDBHelper('/tmp/example_db')
#             >>> db.add('A', 1, 1)
#             >>> db.add('B', 2, 2)
#             >>> db['A']
#             ('A', [1], [1])
#             >>> db['B']
#             ('B', [2], [2])
#             >>> db['C']
#             ('C', [], [])
#         """
#         return (key, self.db['2D'][key], self.db['3D'][key])

#     def load(self, filename):
#         """
#         Load the object database from a filename.

#         If filename doesn't exist, it creates an empty database.
#         """
#         try:
#             with open(filename, 'r') as f:
#                 db = yaml.load(f)
#                 self.db = _db_to_defaultdict(db)
#         except IOError:
#             rospy.logwarn("ObjectDB %s not found. Creating empty DB", filename)
#             self.db = _db_to_defaultdict({'2D': {}, '3D': {}})
#         return self

#     def save(self, filename):
#         """Save the object database to the given filename."""
#         with open(filename, 'w') as f:
#             yaml.dump(_db_to_dict(self.db), f)
#         return self


class ObjectDBNode(object):

    """
    Node that bulds an Object Database from ObjectDescriptor Messages.

    The database can be stored to a given filename or it is automatically
    saved to `db_filename` when the node is shutdown (via rospy.shutdown).
    """

    def __init__(self, db_filename):
        """Init."""
        super(ObjectDBNode, self).__init__()
        self.db_filename = db_filename
        self.db = odbm.ObjectDBHelper(self.db_filename)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('learned_object', ObjectDescriptor, self.callback)

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
    try:
        rospy.init_node(_DEFAULT_NAME)
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))
        db_filename = rospy.myargv(argv=sys.argv)[1]
        rospy.loginfo("Object Database file is: {}"
                      .format(utils.blue(db_filename)))
        db_node = ObjectDBNode(db_filename)
        db_node.run()
    except rospy.ROSInterruptException:
        pass
