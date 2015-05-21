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
Publishes uncertainty metrics and estimates object from recv. predictions.

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco


Publishes the estimated value of the received predictions,
and the entropy and margin of these predictions.


Attributes:
    entropy_pipe: Gets NamedPredictions and publishes their entropy.
    margin_pipe: Gets NamedPredictions and publishes their margin.
    estimator_pipe: Gets NamedPredictions and publishes the estimated object.
    pipes: Single entry point to entropy_pipe, margin_pipe, and estimator_pipe.
    _DEFAULT_NAME: Default name of the node.
"""

import roslib
roslib.load_manifest('ocular_active_learning')

import toolz as tz
import rospy
from rospy_utils import coroutines as co

from ocular_active_learning import al_utils as alu

from ocular_msgs.msg import (NamedPredictions, Prediction)
from ocular_msgs.msg import UncertaintyMetric as Uncertainty


def calc_uncertainty(metric, predictions_msg, name=None):
    """
    Get uncertainty from a predictions_msg.

    Args:
        metric (callable): The function that calculates the uncertainty
            of the predictions_msg. Typicaly al_utils.entropy or margin
        predictions_msg (ocular_msgs.msg.NamedPredictions): the predictions
            for which the uncertainty is going to be calculated.
        name (string): Name of the metric to put in the message. Default: None
            If name is not set, the func gets the name from the metric function.

    """
    return Uncertainty(name=name or metric.__name__,
                       rgb=metric(predictions_msg.rgb),
                       pcloud=metric(predictions_msg.pcloud))


def calc_entropy(predictions):
    """Make an Uncertainty msg with entropy values of the input predictions."""
    return calc_uncertainty(alu.entropy, predictions)


def calc_margin(predictions):
    """Make an Uncertainty msg with margin values of the input predictions."""
    margin = tz.compose(alu.margin, alu.numerize)
    return calc_uncertainty(margin, predictions, name=alu.margin.__name__)


def estimate(predictions):
    """
    Estimate of the matched object from the last named predictions.

    Args:
        predictions (ocular_msgs/NamedPredictions):
            The predictions wrapped in a message

    Returns:
        ocular_msgs/Prediction: The predicted object wrapped in a message.
    """
    combined, rgb, pcloud = alu.estimate(alu.numerize(predictions.rgb),
                                         alu.numerize(predictions.pcloud))
    rospy.logwarn("Fields: {} {} {}".format(combined, rgb, pcloud))
    return Prediction(combined=combined, rgb=rgb, pcloud=pcloud)


def _init_node(node_name):
    """Common routines for start a node."""
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} Node".format(rospy.get_name()))

entropy_pipe = co.pipe([co.mapper(calc_entropy),
                        co.publisher('predictions_entropy', Uncertainty)])

margin_pipe = co.pipe([co.mapper(calc_margin),
                       co.publisher('predictions_margin', Uncertainty)])

estimator_pipe = co.pipe([co.mapper(estimate),
                          co.publisher('predicted_object', Prediction)])

pipes = co.splitter(entropy_pipe, margin_pipe, estimator_pipe)

_DEFAULT_NAME = 'uncertainty_publisher'

if __name__ == '__main__':
    try:
        _init_node(_DEFAULT_NAME)
        co.PipedSubscriber('named_predictions', NamedPredictions, pipes)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
