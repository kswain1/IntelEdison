#!/usr/bin/python2
import numpy as np
import pandas as pd
from sklearn import metrics
from sklearn.neighbors import KNeighborsClassifier

class IsSwinging(object):
    def __init__(self,data_file=None, accel_vector=None):
        self._current_index = 0
        if not data_file:
            #data_file = "./data.csv"
            data_file = "guzman_logs/accel_sfswing.csv"
        if not accel_vector:
            return "please send accel vector ax, ay, az"
        self._accel_data = pd.read_csv(data_file, index_col=False)
        self._feature_cols = ['Ax', 'Ay', 'Az']
        self._accel_vector = accel_vector

    def is_swinging(self):
        #sets up the training data for classification(swing or not swinging) using nearest neighbor algorithm
        input = self._accel_data[self._feature_cols]
        output = self._accel_data['Swing']
        knn = KNeighborsClassifier(n_neighbors=1)
        knn.fit(input,output)
        accel_point = self._accel_vector
        return knn.predict(self._accel_vector)


# # Training data
# data = pd.read_csv('guzman_logs/accel_sfswing.csv', index_col=False)
# feature_cols = ['Ax', 'Ay', 'Az']
# X = data[feature_cols]
# X.values[0]
# Y = data['Swing']
# knn = KNeighborsClassifier(n_neighbors=1)
# knn.fit(X, Y)
#
# # Predict whether we are swinging or not
# y_pred = knn.predict(X)
# print knn.predict([1,1,1])
# for swing in y_pred:
#     if swing: print("Swinging")
#     else: print("Not Swinging")
# print(metrics.accuracy_score(Y, y_pred))

#Example on how to use the class
#ami = IsSwinging(accel_vector = [1,1,1])
#print ami.is_swinging()



