### My Report

##### Step 1: Sensor Noise
```
import numpy as np
paths = ["/home/chu-chu/flying_car_nanodegree/FCND-Estimation-CPP/config/log/Graph1.txt", "/home/chu-chu/flying_car_nanodegree/FCND-Estimation-CPP/config/log/Graph2.txt"]
for path in paths:
    data = np.loadtxt(path, skiprows=1, dtype=np.float64, delimiter=',')
    std = np.std(data[:,1])
    print(std)
```
which give `0.7063681747857126 and 
0.4936211816328591`