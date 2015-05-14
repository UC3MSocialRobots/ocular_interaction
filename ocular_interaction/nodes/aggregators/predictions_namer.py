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
from functools import partial
import rospy_utils as rpyu
from rospy_utils import coroutines as co

from ocular_interaction import utils
from ocular_interaction import object_database_manager as odbm

from ocular_msgs.msg import AccumulatedPredictions as Predictions
from ocular_msgs.msg import NamedPredictions


@rpyu.coroutine
def match_names(db_file, target=None):
    """Match names in a DB."""
    db = odbm.ObjectDBHelper(db_filename)
    find_in_db = partial(odbm.get_object_name_from_id, db=db)
    if not target:
        target = (yield)
    while True:
        preds = (yield)
        named_preds = NamedPredictions(rgb=map(find_in_db, preds.rgb),
                                       pcloud=map(find_in_db, preds.pcloud))
        target.send(named_preds)


def _init_node(node_name):
    """Common routines for start a node."""
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} Node".format(rospy.get_name()))


def _log_predictions(predictions, logger=rospy.loginfo):
    """Log predictions msg to logger."""
    logger("Predictions RGB: {}".format(utils.blue(str(predictions.rgb))))
    logger("Predictions PCloud: {}".format(utils.blue(str(predictions.pcloud))))


_DEFAULT_NAME = 'predictions_namer'

if __name__ == '__main__':
    db_filename = rospy.myargv(argv=sys.argv)[1]
    rospy.loginfo("Object Database file: {}".format(utils.blue(db_filename)))
    try:
        _init_node(_DEFAULT_NAME)
        pipe = co.pipe([match_names(db_filename),
                        co.do(_log_predictions),
                        co.publisher('named_predictions', NamedPredictions)])
        co.PipedSubscriber('predictions', Predictions, pipe)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
