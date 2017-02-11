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
        data_file = open("guzman_logs/accel_sfswing.csv","r")
        read_lines = data_file.readline()
        read_data = str(read_lines).split()
        self._feature_cols = []
        self._output_data = []
        self._accel_vector = accel_vector
        for row in read_data[1:]:
            input = row.split(',')
            row = [float(i) for i in input[:3]]
            self._feature_cols.append(row[:3])
            self._output_data.append(int(input[3]))

    def is_swinging(self):
        #sets up the training data for classification(swing or not swinging) using nearest neighbor algorithm
        input = self._feature_cols
        output = self._output_data
        knn = KNeighborsClassifier(n_neighbors=1)
        knn.fit(input,output)
        accel_point = self._accel_vector
        return knn.predict(self._accel_vector)


#Example on how to use the class
# ami = IsSwinging(accel_vector = [1,1,1])
# print ami.is_swinging()



