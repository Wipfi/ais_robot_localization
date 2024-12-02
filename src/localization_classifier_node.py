#!/usr/bin/env python3

#mercator verioson of scikit-learn befor update : 1.0.2

import rospy
import dill
from std_msgs.msg import Float32MultiArray, Int32MultiArray

# TODO: Add parameter and launch files.

class LocalizationClassifierNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('localization_classifier_node')

        # Load both classifiers from dill files
        self.classifier_1 = self.load_classifier('../models/trained_classifier_dill_1.pkl')
        self.classifier_2 = self.load_classifier('../models/trained_classifier_dill_1.pkl')

        rospy.loginfo(f"classifier 1 {self.classifier_1}, classifier 2 {self.classifier_2}")

        # Subscribe to the "RPE_values" topic
        rospy.Subscriber('/RPE_values', Float32MultiArray, self.rpe_callback)

        # Publisher to publish the predictions
        self.prediction_pub = rospy.Publisher('/localization_class_prediction', Int32MultiArray, queue_size=10)

    def load_classifier(self, filename):
        # Load the classifier using dill
        with open(filename, 'rb') as file:
            classifier = dill.load(file)
        rospy.loginfo(f'Loaded classifier from {filename}')
        return classifier

    def rpe_callback(self, msg):
        # Extract RPE_max_trans and RPE_max_rot from the incoming message
        rpe_avg_trans = msg.data[0]
        rpe_avg_rot = msg.data[1]

        # Prepare the feature for prediction (as an array)
        features = [[rpe_avg_trans, rpe_avg_rot]]

        # Predict the class using both classifiers
        prediction_1 = self.classifier_1.predict(features)[0]
        prediction_2 = self.classifier_2.predict(features)[0]

        # Log the predictions for debugging purposes
        rospy.loginfo(f'Predictions: Classifier 1: {prediction_1}, Classifier 2: {prediction_2}')

        # Prepare the output message
        prediction_msg = Int32MultiArray()
        prediction_msg.data = [int(prediction_1), int(prediction_2)]

        # Publish the prediction
        self.prediction_pub.publish(prediction_msg)

    def run(self):
        # Spin to keep the node running and process callbacks
        rospy.spin()


if __name__ == '__main__':
    try:
        # Create an instance of the node and run it
        node = LocalizationClassifierNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
