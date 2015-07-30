#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('ocular_data')

import pandas as pd
from ocular_active_learning import al_utils as alu
from std_msgs.msg import (Bool, String)
from ocular_msgs.msg import PredictionSummary as SummaryMsg


SUMMARY_TOPIC = '/ocular/predictions/prediction_summary'
ASK_TOPIC = '/ocular/predictions/should_ask_question'
Y_TOPIC = '/real_label'

SUMMARY_FIELDS = ('y', 'pred_combined', 'pred_rgb', 'pred_pcloud',
                  'entropy_rgb', 'entropy_pcloud',
                  'margin_rgb', 'margin_pcloud',
                  'jsd')


def jensen_shannon_divergence(msg):
    """Return the JS Divergence of an ocular_msgs/PredictionSummary msg.

    See Also
    --------
    ocular_active_learning.al_utils.jsd : The implementation of the JSD function
    """
    rgbpmf = alu.pmf(msg.named_predictions.rgb, categories=msg.categories)
    pcloudpmf = alu.pmf(
        msg.named_predictions.pcloud, categories=msg.categories)
    jsd = alu.jsd(rgbpmf, pcloudpmf)
    return jsd


def unpack_prediction_summary(summary, y):
    """Return a tuple with prediciton summary and real label."""
    jsd = jensen_shannon_divergence(summary)
    return (y, summary.predicted.combined,
            summary.predicted.rgb, summary.predicted.pcloud,
            summary.entropy.rgb, summary.entropy.pcloud,
            summary.margin.rgb, summary.margin.pcloud,
            jsd)


class DataAnalizer(object):

    """Main data analizer class."""

    def __init__(self):
        """Constructor."""
        rospy.Subscriber(SUMMARY_TOPIC, SummaryMsg, self.summary_cb)
        rospy.Subscriber(Y_TOPIC, String, self.y_cb)
        self.summary = pd.DataFrame(columns=SUMMARY_FIELDS)
        self.should_ask = []
        self.y = None
        rospy.on_shutdown(self.to_csv)

    def y_cb(self, msg):
        """Store real label."""
        self.y = msg.data

    def summary_cb(self, msg):
        """Append a new summary message to the dataframe."""
        _summary = pd.Series(unpack_prediction_summary(msg, self.y),
                             index=SUMMARY_FIELDS)
        self.summary = self.summary.append(_summary, ignore_index=True)

    def to_csv(self):
        """Write results into a csv file."""
        self.summary.to_csv('/tmp/summary_data.csv')

    def run(self):
        """Spin."""
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('data_analizer_node')
        rospy.loginfo("Initializing {} Node".format(rospy.get_name()))
        node = DataAnalizer()
        node.run()
    except rospy.ROSInterruptException:
        pass
