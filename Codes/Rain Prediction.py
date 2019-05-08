import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestRegressor

def RF(xtrain, ytrain, xtest, ytest):
    clf = RandomForestRegressor()
    clf = clf.fit(xtrain, ytrain)
    y = clf.predict(xtest)
    #print(y)
    error = 0
    for  i in range(len(xtest)):
        if(y[i]!=ytest[i]):
            error = error + 1;
    print(error/len(y)) 
    


def NN(xtrain, ytrain, xtest, ytest):
    clf = KNeighborsClassifier()
    clf = clf.fit(xtrain, ytrain)
    y = clf.predict(xtest)
    #print(y)
    error = 0
    for  i in range(len(xtest)):
        if(y[i]!=ytest[i]):
            error = error + 1;
    print(error/len(y))

def DT(xtrain, ytrain, xtest, ytest):
    clf = DecisionTreeClassifier()
    clf = clf.fit(xtrain, ytrain)
    y = clf.predict(xtest)
    #print(y)
    error = 0
    for  i in range(len(xtest)):
        if(y[i]!=ytest[i]):
            error = error + 1;
    print(error/len(y))
        
   

dataset = pd.read_csv('weatherAUS.csv') 

YesTo1 = {'Yes': 1, 'No':0}


rain_pred = dataset[['MaxTemp','Humidity3pm','RainToday']].copy()
#rain_pred['RainToday']= rain_pred.RainToday.map(YesTo1)

rain_pred = rain_pred.dropna(subset=['Humidity3pm'])
rain_pred = rain_pred.dropna(subset=['MaxTemp'])
rain_pred = rain_pred.dropna(subset=['RainToday'])

train, test = train_test_split(rain_pred, test_size=0.3)

xtrain = train[['MaxTemp','Humidity3pm']].copy()
ytrain = train[['RainToday']].copy()

xtest = test[['MaxTemp','Humidity3pm']].copy()
ytest = test[['RainToday']].copy()

xtrain = np.array(xtrain)
ytrain = np.array(ytrain)
xtest = np.array(xtest)
ytest = np.array(ytest)

print("Error % with NN:\n")
NN(xtrain, ytrain, xtest, ytest)

print("\n Error % with RF:\n")
RF(xtrain, ytrain, xtest, ytest)
print("\n Error % with DT:\n")
DT(xtrain, ytrain, xtest, ytest)
