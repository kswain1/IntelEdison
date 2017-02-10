Developing the Swing prediction algorithm was used with SK learning kit 
## Step 1 Create Machine learning data 


- Mark the Prediction area as True against the listed data
Ax, Ay, Az, Swinging 
0.03, 0.2, 0.5, 0
0.03, 0.2, 0.5, 0
00.03, 0.2, 0.5, 0
00.03, 0.2, 0.5, 0
1.5, 2.7, 3.5, 1
1.5, 2.9, 3.5, 1
1.5, 2.9, 4.0, 1

- above documents how the data is manually marked when someone is swinging the baseball bat, swinging=1, and when some is not,
swingin=0,


## Step 2 Read in Data
``` python
data = pd.read_csv('logs/accel_sfswing.csv', index_col=0)
``` 

##Step 3 Establish feature columns and output columns

``` python
feature_cols = ['Ax', 'Ay', 'Az']
X = data[feature_cols]
Y = data['Swing']
``` 

## Step 4 Establish KNneighbor KN nearest neighbor algorithm 

We have to establish the nearest neighbor for our data which is one. Have yet to figure out what this does exactly something with the order
but you are welcome to find more information about how it works here. https://ashokharnal.wordpress.com/2015/01/20/a-working-example-of-k-d-tree-formation-and-k-nearest-neighbor-algorithms/


```python 
knn = KNeighborsClassifier(n_neighbors=1)
knn.fit(X, Y)
``` 

## Step 5 Prediction Time 
- To predit whether the data is either swinging or not swinging we use the predict function from our knn class.
- The function knn.predict() will return 0 for false or 1 for true 
To Predict each set of data in a list below

```python
y_predict = knn.predict(X)
```

- To Predict on only one set of data, one has to figure out the position that they want to determine true or false

```python 
y_predict = knn.predict(X.values[0])
```

- To predict the data on each particular value: iteration technique

```python 
for i in range(0,len(X.values)):
    y_predict == knn.predict(X.values[i])
```  

#Summary Closing Remarks
- Create training data, create a controlled experiment where you will know the training data
- Toughest part was figuring out how to do a prediction time at each step of the integration. Turns out it was very simple, 
just had to read up on to read data from a panda list. Prediction data also made it to 99% accuracy! 
- Next step is to implement this when the programming is running for the Sensor to automonously send data for the baseball bat.

