#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
Averages ouput from ocular's learner_recognizer_node.

:author: Victor Gonzalez-Pacheco
"""
# from itertools import chain, islice
# from toolz import frequencies
# import pandas as pd

import roslib
roslib.load_manifest('ocular_active_learning')
import rospy
from rospy import (Publisher, Subscriber)
from rospy import loginfo

from ocular_active_learning import al_utils as alu

from ocular.msg import SystemOutput
from ocular_msgs.msg import AccumulatedPredictions as Predictions


class Estimator(object):

    """
    Estimator Node: publishes object_id estimations once a second.

    It subscribes to the accumulated predictions topic and produces estimations
    of the object_id each second from these accumulated predictions.
    """

    def __init__(self):
        """Constructor."""
        rospy.init_node('ocular_object_id_averager', anonymous=True)
        self.node_name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)
        loginfo("Initializing " + self.node_name + " node...")

        # Publishers and Subscribers
        Subscriber("predictions", Predictions, self.callback)
        self.pub = Publisher('final_object_id', SystemOutput)

    def callback(self, predictions):
        """Callback that publishes updated predictions when new msg is recv."""
        y, y_rgb, y_pcloud = alu.estimate(predictions.rgb, predictions.pcloud)
        output_msg = SystemOutput(id_2d_plus_3d=y, id_2d=y_rgb, id_3d=y_pcloud)
        try:
            self.pub.publish(output_msg)
        except ValueError as ve:
            rospy.logwarn(str(ve))

    def run(self):
        """Run (wrapper of ``rospy.spin()``."""
        rospy.spin()

    def shutdown(self):
        """Shudown hook to close the node."""
        loginfo('Shutting down ' + rospy.get_name() + ' node.')


if __name__ == '__main__':
    try:
        node = Estimator()
        node.run()
    except rospy.ROSInterruptException:
        pass
