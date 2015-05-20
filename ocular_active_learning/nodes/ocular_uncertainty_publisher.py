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
Node that performs data processing on NamedPredictions.

It publishes the estimated value of the received predictions,
and the entropy and margin of these predictions.

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco
"""

import roslib
roslib.load_manifest('ocular_active_learning')

import rospy
from rospy_utils import coroutines as co

from ocular_active_learning import al_utils as alu

# from std_msgs.msg import Float32
from ocular_msgs.msg import (NamedPredictions, Prediction)
from ocular_msgs.msg import UncertaintyMetric as Uncertainty


def calc_entropy(predictions):
    """Make an Uncertainty msg with entropy values of the input predictions."""
    return Uncertainty(name='entropy',
                       rgb=alu.entropy(predictions.rgb),
                       pcloud=alu.entropy(predictions.pcloud))


def calc_margin(predictions):
    """Make an Uncertainty msg with margin values of the input predictions."""
    return Uncertainty(name='margin',
                       rgb=alu.margin(alu.numerize(predictions.rgb)),
                       pcloud=alu.margin(alu.numerize(predictions.pcloud)))


def estimate(predictions):
    """
    Estimate of the matched object from the last named predictions.

    Args:
      predictions (ocular_msgs/NamedPredictions):
        The predictions wrapped in a message

    Returns:
        ocular_msgs/Prediction: The predicted object wrapped in a message.
    """
    combined, rgb, pcloud = alu.estimate(predictions.rgb, predictions.pcloud)
    rospy.logwarn("Fields: {} {} {}".format(combined, rgb, pcloud))
    return Prediction(combined=combined, rgb=rgb, pcloud=pcloud)


def _init_node(node_name):
    """Common routines for start a node."""
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} Node".format(rospy.get_name()))

entropy_pipe = co.pipe([co.mapper(calc_entropy),
                        co.publisher('predictions_entropy', Uncertainty)])
"""Pipe that gets NamedPredictions and publishes their entropy."""

margin_pipe = co.pipe([co.mapper(calc_margin),
                       co.publisher('predictions_margin', Uncertainty)])
"""Pipe that gets NamedPredictions and publishes their margin."""

estimator_pipe = co.pipe([co.mapper(estimate),
                          co.publisher('predicted_object', Prediction)])
"""Pipe that gets NamedPredictions and publishes the estimated object."""

pipes = co.splitter(entropy_pipe, margin_pipe, estimator_pipe)
"""Single entry point to entropy_pipe, margin_pipe, and estimator_pipe."""

_DEFAULT_NAME = 'uncertainty_publisher'
"""Default name of the node."""

if __name__ == '__main__':
    try:
        _init_node(_DEFAULT_NAME)
        co.PipedSubscriber('named_predictions', NamedPredictions, pipes)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
