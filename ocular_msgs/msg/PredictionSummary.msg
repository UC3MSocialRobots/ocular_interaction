# Message that summarizes all the data from a predicted object

Header header
ocular_msgs/Prediction predicted
ocular_msgs/UncertaintyMetric entropy
ocular_msgs/UncertaintyMetric margin
ocular_msgs/AccumulatedPredictions all_predictions
ocular_msgs/NamedPredictions named_predictions
string[] categories         # All object categories