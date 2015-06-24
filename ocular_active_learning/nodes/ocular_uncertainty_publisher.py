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

import sys
import toolz as tz
import rospy
from rospy_utils import coroutines as co

from ocular_active_learning import al_utils as alu
from ocular_interaction import object_database_manager as odbm

from std_msgs.msg import Bool
from ocular_msgs.msg import (NamedPredictions, Prediction, PredictionSummary)
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


def calc_margin(predictions, numerizer=alu.numerize):
    """Make an Uncertainty msg with margin values of the input predictions.

    Args:
        predictions (ocular_msgs/NamedPredictions):
            The predictions wrapped in a message
        numerizer: func with type [a] -> [(int, a)] that numerizes predictions.
            (default: ocular_active_learning.al_utils.numerize)
    """
    margin = tz.compose(alu.margin, numerizer)
    return calc_uncertainty(margin, predictions, name=alu.margin.__name__)


def estimate(predictions, numerizer=alu.numerize):
    """
    Estimate of the matched object from the last named predictions.

    Args:
        predictions (ocular_msgs/NamedPredictions):
            The predictions wrapped in a message
        numerizer: func with type [a] -> [(int, a)] that numerizes predictions.
            (default: ocular_active_learning.al_utils.numerize)

    Returns:
        ocular_msgs/Prediction: The predicted object wrapped in a message.
    """
    combined, rgb, pcloud = alu.estimate(numerizer(predictions.rgb),
                                         numerizer(predictions.pcloud))
    rospy.logdebug("Estimation Fields: {} {} {}".format(combined, rgb, pcloud))
    return Prediction(combined=combined[0], rgb=rgb[0], pcloud=pcloud[0])


def _init_node(node_name):
    """Common routines for start a node."""
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} Node".format(rospy.get_name()))


class UncertaintyPublisher(object):

    """
    Node that publishes estimation, entropy and margin of this estimation.

    **Subscribes**: 'object_database_updated' (std_msgs/Bool)
                    'named_predictions' (ocular_msgs/NamedPredictions)
    **Publishes**: 'predictions_entropy' (ocular_msgs/Uncertainty)
                   'predictions_margin' (ocular_msgs/Uncertainty)
                   'predicted_object' (ocular_msgs/Prediction)
                   'predicted_summary' (ocular_msgs/PredictionSummary)
    """

    def __init__(self, db_filename):
        """
        Constructor.

        Args:
            db_filename (str): Filename of the Object DB to load.
        """
        super(UncertaintyPublisher, self).__init__()
        self.db_filename = db_filename
        self.db = odbm.ObjectDBHelper(self.db_filename)
        self.numeric_keys = self.numerize_db_keys()
        rospy.on_shutdown(self.shutdown)

        self.summary_pipe = \
            co.pipe([co.mapper(self.summarize_results),
                     co.publisher('prediction_summary', PredictionSummary)])

        # self.entropy_pipe = \
        #     co.pipe([co.mapper(calc_entropy),
        #              co.publisher('predictions_entropy', Uncertainty)])
        # self.margin_pipe = \
        #     co.pipe([co.starmapper(calc_margin, None, self.numerize_array),
        #              co.publisher('predictions_margin', Uncertainty)])
        # self.estimator_pipe = \
        #     co.pipe([co.starmapper(estimate, None, self.numerize_array),
        #              co.publisher('predicted_object', Prediction)])

        # self.pipes = co.splitter(self.entropy_pipe,
        #                          self.margin_pipe,
        #                          self.estimator_pipe,
        #                          self.summary_pipe)

        # co.PipedSubscriber('named_predictions', NamedPredictions, self.pipes)
        co.PipedSubscriber(
            'named_predictions', NamedPredictions, self.summary_pipe)
        rospy.Subscriber('object_database_updated', Bool, self.callback)

    def summarize_results(self, predictions):
        """Return an ocular_msgs/PredictionSummary from received predictions."""
        summary = PredictionSummary()
        summary.categories = self.db.keys()
        summary.predicted = estimate(predictions, numerizer=self.numerize_array)
        summary.entropy = calc_entropy(predictions)
        summary.margin = calc_margin(predictions, numerizer=self.numerize_array)
        summary.named_predictions = predictions
        summary.all_predictions.pcloud = \
            zip(*self.numerize_array(predictions.pcloud))[0]
        summary.all_predictions.rgb = \
            zip(*self.numerize_array(predictions.rgb))[0]
        return summary

    def numerize_db_keys(self):
        """Convert DB keys to numbers.

        Return a dict whose keys are the DB labels and its values the numeric id
        corresponding to these labels.
        """
        self.keys = self.db.keys()
        numeric_keys = {v: k for k, v in alu.numerize(self.keys)}
        numeric_keys['NOT_FOUND'] = -1
        return numeric_keys

    def numerize_array(self, array):
        """Convert an array of labels to an array of their corresponding ids."""
        return [(self.numeric_keys[elem], elem) for elem in array]

    def callback(self, msg):
        """Callback that fetches last changes from the object database."""
        self.db.load(self.db_filename)
        self.numeric_keys = self.numerize_db_keys()

    def shutdown(self):
        """Hook to be executed when rospy.shutdown is called."""
        pass

    def run(self):
        """Run the node."""
        rospy.spin()


_DEFAULT_NAME = 'uncertainty_publisher'

if __name__ == '__main__':
    db_filename = rospy.myargv(argv=sys.argv)[1]
    rospy.loginfo("Loaded Object Database file from: {}".format(db_filename))
    try:
        _init_node(_DEFAULT_NAME)
        matcher = UncertaintyPublisher(db_filename)
        matcher.run()
    except rospy.ROSInterruptException:
        pass
