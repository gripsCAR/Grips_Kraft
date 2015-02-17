#! /usr/bin/env python
import rospy, math
import numpy as np


# First-Order Adaptive Windowing (FOAW)
def best_fit_foaw(y, fs, m, d):  
  T = 1.0/fs
  result = np.zeros(len(y))
  for k in range(len(y)):
    slope = 0
    for n in range(1, min(m,k)):
      # Calculate slope over interval (best-fit-FOAW)
      b = ( ( n*sum([y[k-i]     for i in range(n+1)])
            - 2*sum([y[k-i]*i   for i in range(n+1)]) )
          / (T*n*(n+1)*(n+2)/6) )
      # Check the linear estimate of each middle point
      outside = False
      for j in range(1,n):
        ykj = y[k]-(b*j*T)
        # Compare to the measured value within the noise margin
        # If it's outside noise margin, return last estimate
        if abs(y[k-j] - ykj) > 2*d:
          outside = True
          break
      if outside: break
      slope = b
    result[k] = slope
  return result
