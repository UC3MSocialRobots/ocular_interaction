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
Node that performs data processing on named_predictions.

It publishes the estimated value of the received predictions,
and the entropy and margin of these predictions.

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco
"""

import roslib
roslib.load_manifest('ocular_active_learning')

import rospy
from rospy_utils import coroutines as co

from ocular_interaction import utils
from ocular_active_learning import al_utils as alu

# from std_msgs.msg import Float32
from ocular_msgs.msg import (NamedPredictions, Prediction)
from ocular_msgs.msg import UncertaintyMetric as Uncertainty


def _make_entropy_msg(predictions):
    """Make an Uncertainty msg with entropy values of the input predictions."""
    return Uncertainty(name='entropy',
                       rgb=alu.entropy(predictions.rgb),
                       pcloud=alu.entropy(predictions.pcloud))


def _make_margin_msg(predictions):
    """Make an Uncertainty msg with margin values of the input predictions."""
    return Uncertainty(name='margin',
                       rgb=alu.numerize(alu.margin(predictions.rgb)),
                       pcloud=alu.numerize(alu.margin(predictions.pcloud)))


def _make_prediction_msg(prediction_fields):
    """Make a prediction message from a tuple containing the msg fields."""
    combined, rgb, pcloud = prediction_fields
    return Prediction(combined=combined, rgb=rgb, pcloud=pcloud)


def _init_node(node_name):
    """Common routines for start a node."""
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} Node".format(rospy.get_name()))


def _log_predictions(predictions, logger=rospy.loginfo):
    """Log predictions msg to logger."""
    logger("Predictions RGB: {}".format(utils.blue(str(predictions.rgb))))
    logger("Predictions PCloud: {}".format(utils.blue(str(predictions.pcloud))))

entropy_pipe = co.pipe([co.mapper(_make_entropy_msg),
                        co.publisher('predictions_entropy', Uncertainty)])

margin_pipe = co.pipe([co.mapper(_make_margin_msg),
                       co.publisher('predictions_margin', Uncertainty)])

estimator_pipe = co.pipe([co.mapper(alu.estimate),
                          co.mapper(_make_prediction_msg),
                          co.publisher('predicted_object', Prediction)])

metrics_pipes = co.splitter(entropy_pipe, margin_pipe, estimator_pipe)

_DEFAULT_NAME = 'uncertainty_puglisher'

if __name__ == '__main__':
    try:
        _init_node(_DEFAULT_NAME)
        # pipe = co.pipe([co.mapper(alu.numerize), metrics_pipes])
        co.PipedSubscriber('named_predictions', NamedPredictions, metrics_pipes)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
