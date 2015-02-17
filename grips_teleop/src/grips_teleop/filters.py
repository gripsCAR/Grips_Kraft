#! /usr/bin/env python
import rospy, math
import numpy as np
from scipy.signal import medfilt

def smooth_diff(n):
  """A smoothed differentiation filter (digital differentiator). 

  Such a filter has the following advantages:

  First, the filter involves both the smoothing operation and differentation operation. 
  It can be regarded as a low-pass differention filter (digital differentiator). 
  It is well known that the common differentiation operation amplifies the high-frequency noises.
  Therefore, the smoothded differentiation filter would be valuable in experimental (noisy) data processing. 

  Secondly, the filter coefficients are all convenient integers (simple units) except for an integer scaling factor,
  as may be especially significant in some applications such as those in some single-chip microcomputers
  or digital signal processors. 

  Usage:
  h=smooth_diff(n)
  n: filter length (positive integer larger no less than 2)
  h: filter coefficients (anti-symmetry)

  Author:
  Jianwen Luo <luojw@bme.tsinghua.edu.cn, luojw@ieee.org> 2004-11-02
  Department of Biomedical Engineering, Department of Electrical Engineering
  Tsinghua University, Beijing 100084, P. R. China  

  References:
  Usui, S.; Amidror, I., 
  Digital Low-Pass Differentiation for Biological Signal-Processing. 
  IEEE Transactions on Biomedical Engineering 1982, 29, (10), 686-693.
  Luo, J. W.; Bai, J.; He, P.; Ying, K., 
  Axial strain calculation using a low-pass digital differentiator in ultrasound elastography. 
  IEEE Transactions on Ultrasonics Ferroelectrics and Frequency Control 2004, 51, (9), 1119-1127.
  """
  if n >= 2 and math.floor(n) == math.ceil(n):
    if n % 2 == 1:                    # is odd
        m = math.trunc((n-1) / 2.0);
        h = np.concatenate( (-np.ones(m), [0], np.ones(m)) ) / (m * (m+1))
    else:                             # is even
        m = math.trunc(n / 2.0);
        h = np.concatenate( (-np.ones(m), np.ones(m)) ) /m**2
  else:
    raise TypeError("The input parameter (n) should be a positive integer larger no less than 2.")
  return h



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
        #~ if abs(y[k-j] - ykj) > 2*d:
        if ykj < (y[k-j]-d) or ykj > (y[k-j]+d):
          outside = True
          break
      if outside: break
      slope = b
    result[k] = slope
  return result
  
  
def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    r"""Smooth (and optionally differentiate) data with a Savitzky-Golay filter.
    The Savitzky-Golay filter removes high frequency noise from data.
    It has the advantage of preserving the original shape and
    features of the signal better than other types of filtering
    approaches, such as moving averages techniques.
    Parameters
    ----------
    y : array_like, shape (N,)
        the values of the time history of the signal.
    window_size : int
        the length of the window. Must be an odd integer number.
    order : int
        the order of the polynomial used in the filtering.
        Must be less then `window_size` - 1.
    deriv: int
        the order of the derivative to compute (default = 0 means only smoothing)
    Returns
    -------
    ys : ndarray, shape (N)
        the smoothed signal (or it's n-th derivative).
    Notes
    -----
    The Savitzky-Golay is a type of low-pass filter, particularly
    suited for smoothing noisy data. The main idea behind this
    approach is to make for each point a least-square fit with a
    polynomial of high order over a odd-sized window centered at
    the point.
    Examples
    --------
    t = np.linspace(-4, 4, 500)
    y = np.exp( -t**2 ) + np.random.normal(0, 0.05, t.shape)
    ysg = savitzky_golay(y, window_size=31, order=4)
    import matplotlib.pyplot as plt
    plt.plot(t, y, label='Noisy signal')
    plt.plot(t, np.exp(-t**2), 'k', lw=1.5, label='Original signal')
    plt.plot(t, ysg, 'r', label='Filtered signal')
    plt.legend()
    plt.show()
    References
    ----------
    .. [1] A. Savitzky, M. J. E. Golay, Smoothing and Differentiation of
       Data by Simplified Least Squares Procedures. Analytical
       Chemistry, 1964, 36 (8), pp 1627-1639.
    .. [2] Numerical Recipes 3rd Edition: The Art of Scientific Computing
       W.H. Press, S.A. Teukolsky, W.T. Vetterling, B.P. Flannery
       Cambridge University Press ISBN-13: 9780521880688
    """
    import numpy as np
    from math import factorial

    try:
        window_size = np.abs(np.int(window_size))
        order = np.abs(np.int(order))
    except ValueError, msg:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order+1)
    half_window = (window_size -1) // 2
    # precompute coefficients
    b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
    m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs( y[1:half_window+1][::-1] - y[0] )
    lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    return np.convolve( m[::-1], y, mode='valid')
