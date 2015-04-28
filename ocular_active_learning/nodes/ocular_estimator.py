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
from rospy import loginfo, logfatal

from ocular_active_learning import al_utils as alu

from ocular.msg import (RecognizedObject, SystemOutput)
from rospy_utils.param_utils import load_params
from rospy_utils.func_utils import error_handler as eh

__RATE = 30


class Estimator():

    """
    Estimator Node: publishes final result once a second.

    Accumulate predictions and publishes a final result every second
    """

    def __init__(self, **kwargs):
        """Constructor."""
        rospy.init_node('ocular_object_id_averager', anonymous=True)
        self.node_name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)
        loginfo("Initializing " + self.node_name + " node...")

        with eh(logger=logfatal, log_msg="Couldn't load parameters",
                reraise=True):
            self.hz = load_params(['rate']).next()

        # Publishers and Subscribers
        self.pub = Publisher('final_object_id', SystemOutput)
        Subscriber("object_id", RecognizedObject, self.callback)

        # self.r = rospy.Rate(self.hz)
        # Accumulates Hz items in per second. Ex: 30Hz -> ~30items/sec
        self.accumulator = alu.Accumulator(self.hz)

    def callback(self, data):
        """Callback that publishes updated predictions when new msg is recv."""
        self.accumulator.append(data.object_id)
        if self.accumulator.isfull():
            rospy.logdebug("Accumulator full. Printing all predictions")
            rospy.logdebug("{}".format(self.accumulator))
            predictions_rgb, predictions_pcloud = zip(*self.accumulator)
            y, y_rgb, y_pcloud = alu.estimate(predictions_rgb,
                                              predictions_pcloud)
            output_msg = SystemOutput(id_2d_plus_3d=y,
                                      id_2d=y_rgb,
                                      id_3d=y_pcloud)
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
