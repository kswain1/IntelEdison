#!/usr/bin/python2
import numpy as np
import pandas as pd
from sklearn import metrics
from sklearn.neighbors import KNeighborsClassifier

# Training data
data = pd.read_csv('logs/accel_sfswing.csv', index_col=False)
feature_cols = ['Ax', 'Ay', 'Az']
X = data[feature_cols]
Y = data['Swing']
knn = KNeighborsClassifier(n_neighbors=1)
knn.fit(X, Y)

# Predict whether we are swinging or not
y_pred = knn.predict(X)
for swing in y_pred:
    if swing: print("Swinging")
    else: print("Not Swinging")
print(metrics.accuracy_score(Y, y_pred))

