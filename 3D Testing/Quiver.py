# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 17:04:50 2022

@author: Husayn
"""

import numpy as np
import pandas
import matplotlib.pyplot as plt

Data = pandas.read_excel("RichmondBrighouseToLangara.xlsx")

DataGPS = Data.loc[ Data["GPSLock"] == True] 
DataNoGPS = Data.loc[ Data["GPSLock"] == False] 

# plt.quiver(Data['posE'],Data['posN'],Data['velE'],Data['velN'])
fig = plt.figure(figsize=(12, 12), dpi=200)
plt.scatter(DataGPS['posE'],DataGPS['posN'],s=0.1,color='green')
plt.scatter(DataNoGPS['posE'],DataNoGPS['posN'],s=0.1,color='red')

plt.axis('equal')

plt.show()


fig.savefig('RichmondBrighouseToLangara.png', transparent=True)

