# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 17:04:50 2022

@author: Husayn
"""

import numpy as np
import pandas
import matplotlib.pyplot as plt

Data = pandas.read_excel("YVRtoMDS.xlsx")

plt.quiver(Data['posE'],Data['posN'],Data['velE'],Data['velN'])
plt.plot(Data['posE'],Data['posN'])

plt.show()


