import numpy as np
from ExtractLines import FitLine

LineExtractionParams = {'MIN_SEG_LENGTH': 0.1,             # minimum length of each line segment (m)
                        'LINE_POINT_DIST_THRESHOLD': 0.20, # max distance of pt from line to split
                        'MAX_P2P_DIST': 0.4,               # max distance between two adjent pts within a segment
                        'MIN_POINTS_PER_SEGMENT': 3}       # minimum number of points per line segment

NoiseParams = {'Sigma0': 0.01*np.eye(3),  # initial state covariance (x0 comes from ground truth; nonzero in case of timing mismatch)
               'R': 0.1*np.eye(2),    # control noise covariance (corresponding to dt = 1 second)
               'var_theta': 0.03,     # laser scan noise variance in theta measurement (per point)
               'var_rho': 0.05,       # laser scan noise variance in rho measurement (per point)
               'g': 3.,               # validation gate (essentially maximum z-score)
               'std_alpha': 0.1,      # noisy map stdev in alpha for EKF_SLAM (per line)
               'std_r': 0.2}          # noisy map stdev in r for EKF_SLAM (per line)

MAZE = [
  ((5, 5), (-5, 5)),
  ((-5, 5), (-5, -5)),
  ((-5,-5), (5, -5)),
  ((5, -5), (5, 5)),
  ((-3, -3), (-3, -1)),
  ((-3, -3), (-1, -3)),
  ((3, 3), (3, 1)),
  ((3, 3), (1, 3)),
  ((1, -1), (3, -1)),
  ((3, -1), (3, -3)),
  ((-1, 1), (-3, 1)),
  ((-3, 1), (-3, 3)),
  ((-1, -1), (1, -3)),
  ((-1, 5), (-1, 2)),
  ((0, 0), (1, 1))
]

ARENA = [
  ((-4,0), (-3,-6)),
  ((-3,-6), (4,-4)),
  ((4,-4), (5,2)),
  ((5,2), (1,4)),
  ((1,4), (-4,0)),
  ((-1,-1), (1,-3))
]

MapParams = np.array([FitLine(np.array([np.arctan2(p1[1], p1[0]), np.arctan2(p2[1], p2[0])]),
                              np.array([np.linalg.norm(p1), np.linalg.norm(p2)])) for p1, p2 in MAZE]).T

ArenaParams = np.array([FitLine(np.array([np.arctan2(p1[1], p1[0]), np.arctan2(p2[1], p2[0])]),
                                np.array([np.linalg.norm(p1), np.linalg.norm(p2)])) for p1, p2 in ARENA]).T
