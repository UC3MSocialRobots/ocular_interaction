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
Node that triggers a question to the user if a prediction has high uncertainty.

Attributes
----------
pipe : rospy_utils.coroutines.pipe
    Pipe that gets as input an ocular_msgs/PredictionSummary msg,
    calculates the Jensen-Shannon Divergence of that prediction and
    decides whether is worth or not to send an ask a question bool depending on
    how big is that divergence.

**Subscribes**: 'prediction_summary' (ocular_msgs/PredictionSummary)
**Publishes**: 'should_ask_question', (std_msgs/Bool)

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco

"""
import roslib; roslib.load_manifest('ocular_active_learning')
import rospy
import random

from rospy_utils import coroutines as co

from ocular_active_learning import al_utils as alu

from std_msgs.msg import Bool
from ocular_msgs.msg import PredictionSummary


def jensen_shannon_divergence(msg):
    """Return the JS Divergence of an ocular_msgs/PredictionSummary msg.

    See Also
    --------
    ocular_active_learning.al_utils.jsd : The implementation of the JSD function
    """
    rgbpmf = alu.pmf(msg.named_predictions.rgb, categories=msg.categories)
    pcloudpmf = alu.pmf(msg.named_predictions.pcloud, categories=msg.categories)
    jsd = alu.jsd(rgbpmf, pcloudpmf)
    # rospy.logdebug("JSD: {}".format(jsd))
    return jsd


def should_ask_question(jsd_value):
    """Function that decides whether to ask a question or not to the user.

    Parameters
    ----------
    jsd_value : float
        Value indicating the JSD (Jensen-Shannon Divergence) of a prediction.
        This value should be between 0 and 1 (both included).

    Returns
    -------
    bool
        A bool indicating whether to ask a question or not.
    """
    randvalue = random.random()
    should_ask = randvalue < jsd_value
    rospy.logdebug("JSD: {}, Randval: {}, Going to Ask?: {}"
                   .format(jsd_value, randvalue, should_ask))
    return should_ask


pipe = co.pipe([co.mapper(jensen_shannon_divergence),
                co.filterer(should_ask_question),
                co.do(lambda _: rospy.loginfo("Going to ask")),
                co.publisher("should_ask_question", Bool)])


_DEFAULT_NAME = 'al_asker_node'

if __name__ == '__main__':
    try:
        rospy.init_node(_DEFAULT_NAME)
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))
        co.PipedSubscriber('prediction_summary', PredictionSummary, pipe)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
