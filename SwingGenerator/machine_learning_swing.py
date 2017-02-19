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


# Training data
data = pd.read_csv('guzman_logs/accel_sfswing.csv', index_col=False)
feature_cols = ['Ax', 'Ay', 'Az']
X = data[feature_cols]
print X.values[0]
Y = data['Swing']
knn = KNeighborsClassifier(n_neighbors=1)
knn.fit(X, Y)

#There are three cases
# whensomeone is swinging 
#    continue to save the recorded data into an array
# when the swing is just completed
#    send the recorded data of swinging 
# when the bat is not swinging 
#    wait until someone is swinging 
# Predict whether we are swinging or not
y_pred = knn.predict(X)
# print knn.predict([1,1,1])
record_data = []
counter = 0
x = 0 
i = 0 
while i+3 < len(X):
    zero_counter = 0
    swing = list(X.values[i])
    print swing
    #import pdb; pdb.set_trace()
    #import pdb; pdb.set_trace()\
    if (knn.predict(X.values[i]) or knn.predict(X.values[i+1]) or knn.predict(X.values[i+3])):
        x = i 
         #case for when someone is swinging 
	    #print "saving swing"
        while (knn.predict(X.values[x]) or knn.predict(X.values[x+1]) or knn.predict(X.values[x+3])):      
             record_data.append(X.values[x])
             x += 1
             zero_counter += 1


    	with open('swing.txt', 'a') as outfile:
            counter += 1
    	    outfile.write('recorded swing, {:d}, {:d}, {} \n'.format(counter,  zero_counter, record_data))
            record_data = []
            i = x
            zero_counter = 0 
    	#import pdb; pdb.set_trace()
    # case three for when someone is not swinging the baseball bat currently not working because this value 
    else:
        i += 1
    # if knn.predict(X.values[i]) == 0: 
    #     zero_counter += 1
        # check to see if the next value is 1 to send the next value
    # 2 case for right after the swing zero counter should be greater 1 this condition doesn't work after the knn.predict has ran multiple times 
   # if zero_counter == 1:
    #   print("sending recorded information to the database")
     #  zero_counter = 0 
    #   with open('swing_log.txt', 'a') as outfile:
     #      outfile.write("sending data to database\n")

print(metrics.accuracy_score(Y, y_pred))

#Example on how to use the class
#ami = IsSwinging(accel_vector = [1,1,1])
#print ami.is_swinging()



