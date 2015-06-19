#!/usr/bin/env python

# :copyright:    Copyright (C) 2015 Universidad Carlos III de Madrid.
#                Todos los derechos reservados.
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
Fuses all information related to a prediction and summarizes in a single topic.

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco

"""
import roslib; roslib.load_manifest('ocular_active_learning')
import rospy

from ocular_msgs.msg import (AccumulatedPredictions, NamedPredictions,
                             Prediction, PredictionSummary, UncertaintyMetric)

from message_filters import TimeSynchronizer, Subscriber
# from ocular_active_learning.message_filters import (ApproximateTimeSynchronizer,
#                                                     Subscriber)

subscribers = [Subscriber('predicted_object', Prediction),
               Subscriber('predictions_entropy', UncertaintyMetric),
               Subscriber('predictions_margin', UncertaintyMetric),
               Subscriber('accumulated_predictions', AccumulatedPredictions),
               Subscriber('named_predictions', NamedPredictions)]


class PredictionSummarizer(object):

    """Node that decides whether to ask a question from uncertainty metrics."""

    def __init__(self):
        """Init."""
        super(PredictionSummarizer, self).__init__()
        rospy.on_shutdown(self.shutdown)

        self.pub = rospy.Publisher('prediction_summary', PredictionSummary)
        self.tss = TimeSynchronizer(subscribers, queue_size=3)
        # self.tss = \
        #     ApproximateTimeSynchronizer(subscribers, queue_size=3, slop=5)

        self.tss.registerCallback(self.callback)

    def callback(self, prediction, entropy, margin,
                 all_predictions, named_predictions):
        """Produce a summary msg and publish it."""
        summary = PredictionSummary(predicted=prediction,
                                    entropy=entropy,
                                    margin=margin,
                                    all_predictions=all_predictions,
                                    named_predictions=named_predictions)
        self.pub.publish(summary)

    def shutdown(self):
        """Hook to be executed when rospy.shutdown is called."""
        pass

    def run(self):
        """Run the node."""
        rospy.spin()


_DEFAULT_NAME = 'object_database_builder'

if __name__ == '__main__':
    try:
        rospy.init_node(_DEFAULT_NAME)
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))
        node = PredictionSummarizer()
        node.run()
    except rospy.ROSInterruptException:
        pass
