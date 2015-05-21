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
This module provides the utilities to load an save an object DB.

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco
"""

import roslib
roslib.load_manifest('ocular_interaction')
import yaml
import rospy
from collections import defaultdict
from itertools import chain
from toolz import memoize


def _db_to_defaultdict(db):
    """Transform the type of the dabase subdicts to defaultdict."""
    return {'2D': defaultdict(list, db['2D']),
            '3D': defaultdict(list, db['3D'])}


def _db_to_dict(db):
    """Convert the database to dict."""
    return {'2D': dict(db['2D']),
            '3D': dict(db['3D'])}


def _get_object_name_from_id(object_id, db):
    """
    Seek object_id in object db and returns its name.

    Example:

        >>> db = ObjectDBHelper('/tmp/example_db')
        >>> db.add('A', 1, 1)
        >>> db.add('B', 2, 2)
        >>> get_object_name_from_id(1, db)
        'A'
        >>> get_object_name_from_id(2, db)
        'B'
        >>> get_object_name_from_id(3, db)
        'NOT_FOUND'
    """
    found_names = [name
                   for name, ids_rgb, ids_pcloud in db.items()
                   if any([object_id in ids_rgb,
                           object_id in ids_pcloud])]
    return found_names[0] if found_names else 'NOT_FOUND'

# Note:
#   This is the "memoized" [1] version of a private function with the same name
#   and is the one that should be used.

#   [1] http://toolz.readthedocs.org/en/latest/api.html#toolz.functoolz.memoize
get_object_name_from_id = memoize(_get_object_name_from_id)


class ObjectDBHelper(object):

    """Loads and stores the ObjectDatabase to YAML file."""

    def __init__(self, db_filename):
        """
        Constructor.

        Args:
            db_filename (str): Filename (and path) to the DB YAML file.
        """
        super(ObjectDBHelper, self).__init__()
        self.db_filename = db_filename
        self.load(self.db_filename)

    def add(self, name, id2D, id3D):
        """Add an entry to the database."""
        self.db['2D'][name].append(id2D)
        self.db['3D'][name].append(id3D)

    def get(self, key, *args, **kwargs):
        """
        Return an entry from the DB with similar syntax to 'dict.get'.

        Args
            key (str): the key of the values to retrieve.

        Returns:
            tuple of (key, value2D, value3D)

        Example:

            >>> db = ObjectDBHelper('/tmp/example_db')
            >>> db.add('A', 1, 1)
            >>> db.add('B', 2, 2)
            >>> db.get('A')
            ('A', [1], [1])
            >>> db.get('B')
            ('B', [2], [2])
            >>> db.get('C', 'ITEM NOT FOUND')
            ('C', 'ITEM NOT FOUND', 'ITEM NOT FOUND')
        """
        return (key,
                self.db['2D'].get(key, *args, **kwargs),
                self.db['3D'].get(key, *args, **kwargs))

    def __getitem__(self, key):
        """
        Retrieve an item in a dict-like syntax.

        :param key: the key of the values to retrieve.
        :type key: str
        :return: tuple: (key, value2D, value3D)

        Example:
            >>> db = ObjectDBHelper('/tmp/example_db')
            >>> db.add('A', 1, 1)
            >>> db.add('B', 2, 3)
            >>> db['A']
            ('A', [1], [1])
            >>> db['B']
            ('B', [2], [3])
            >>> db['C']
            ('C', [], [])
        """
        return (key, self.db['2D'][key], self.db['3D'][key])

    def items(self):
        """Return all DB elements."""
        keys = set(chain(self.db['2D'].keys(), self.db['3D'].keys()))
        return tuple(self.get(key) for key in keys)

    def keys(self):
        """
        Return all keys of the Database.

        Example:
            >>> db = ObjectDBHelper('/tmp/example_db')
            >>> db.add('A', 1, 1)
            >>> db.add('B', 2, 3)
            >>> db.add('C', 3, 4)
            >>> db.keys()
            ('A', 'B', 'C')
        """
        return tuple(sorted(set(chain(self.db['2D'].keys(),
                                      self.db['3D'].keys()))))

    def load(self, filename):
        """
        Load the object database from a filename.

        If filename doesn't exist, it creates an empty database.
        """
        try:
            with open(filename, 'r') as f:
                db = yaml.load(f)
                self.db = _db_to_defaultdict(db)
        except IOError:
            rospy.logwarn("ObjectDB %s not found. Creating empty DB", filename)
            self.db = _db_to_defaultdict({'2D': {}, '3D': {}})
        return self

    def save(self, filename):
        """Save the object database to the given filename."""
        with open(filename, 'w') as f:
            yaml.dump(_db_to_dict(self.db), f)
        return self
