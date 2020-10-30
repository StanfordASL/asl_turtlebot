############################################################
# ExtractLines.py
#
# This script reads in range data from a csv file, and
# implements a split-and-merge to extract meaningful lines
# in the environment.
############################################################

# Imports
import numpy as np

############################################################
# functions
############################################################

#-----------------------------------------------------------
# ExtractLines
#
# This function implements a split-and-merge line
# extraction algorithm
#
# INPUT:     theta - (1D) np array of angle 'theta' from data (rads)
#              rho - (1D) np array of distance 'rho' from data (m)
#           params - dictionary of parameters for line extraction
#        var_theta - variance in theta measurement (pointwise)
#          var_rho - variance in rho measurement (pointwise)
#
# OUTPUT: (alpha, r, segend, pointIdx)
#         alpha - (1D) np array of 'alpha' for each fitted line (rads)
#             r - (1D) np array of 'r' for each fitted line (m)
#          C_AR - list of covariance matrices for (alpha, r)
#        segend - np array (N_lines, 4) of line segment endpoints.
#                 each row represents [x1, y1, x2, y2]
#      pointIdx - (N_lines,2) segment's first and last point index

def ExtractLines(theta, rho, params, var_theta = None, var_rho = None):

    nan_idxs = np.isnan(rho)
    if all(nan_idxs):
        if var_theta is not None and var_rho is not None:
            return np.array([]), np.array([]), [], np.zeros((0,4)), np.array([])
        else:
            return np.array([]), np.array([]), np.zeros((0,4)), np.array([])
    theta = theta[~nan_idxs]
    rho = rho[~nan_idxs]

    ### Split Lines ###
    r = np.array([])
    alpha = np.array([])
    pointIdx = np.empty((0,2), dtype=int)

    # This implementation pre-prepartitions the data  according to "MAX_P2P_DIST"
    #  parameter. It forces line segmentation at sufficiently large range jumps
    N_pts = len(rho)
    rho_diff = np.abs(rho[1:] - rho[:(len(rho)-1)])
    LineBreak = np.hstack((np.where(rho_diff > params['MAX_P2P_DIST'])[0]+1, N_pts))
    startIdx = 0
    for endIdx in LineBreak:
        alpha_seg, r_seg, pointIdx_seg = SplitLinesRecursive(theta, rho, startIdx, endIdx, params)
        N_lines = r_seg.size

        ### Merge Lines ###
        if (N_lines > 1):
            alpha_seg, r_seg, pointIdx_seg = MergeColinearNeigbors(theta, rho, alpha_seg, r_seg, pointIdx_seg, params)
        alpha = np.append(alpha,alpha_seg)
        r = np.append(r, r_seg)
        pointIdx = np.vstack((pointIdx, pointIdx_seg))
        startIdx = endIdx

    pointIdx = pointIdx.astype(int)
    N_lines = alpha.size

    ### Compute endpoints/lengths of the segments ###
    segend = np.zeros((N_lines, 4))
    seglen = np.zeros(N_lines)
    for i in range(N_lines):
        rho1 = r[i]/np.cos(theta[pointIdx[i,0]]-alpha[i])
        rho2 = r[i]/np.cos(theta[pointIdx[i,1]-1]-alpha[i])
        x1 = rho1*np.cos(theta[pointIdx[i,0]])
        y1 = rho1*np.sin(theta[pointIdx[i,0]])
        x2 = rho2*np.cos(theta[pointIdx[i,1]-1])
        y2 = rho2*np.sin(theta[pointIdx[i,1]-1])
        segend[i,:] = np.hstack((x1, y1, x2, y2))
        seglen[i] = np.linalg.norm(segend[i,0:2] - segend[i,2:4])

    ### Filter Lines ###
    #Find and remove line segments that are too short
    goodSegIdx = np.where((seglen >= params['MIN_SEG_LENGTH']) &
    (pointIdx[:,1] - pointIdx[:,0] >= params['MIN_POINTS_PER_SEGMENT']))[0]
    pointIdx = pointIdx[goodSegIdx, :]
    alpha = alpha[goodSegIdx]
    r = r[goodSegIdx]
    segend = segend[goodSegIdx, :]

    ### Compute Covariances ###
    if var_theta is not None and var_rho is not None:
        C_AR = [FitLine(theta[startIdx:endIdx], rho[startIdx:endIdx], var_theta, var_rho)[2] for startIdx, endIdx in pointIdx]
        return alpha, r, C_AR, segend, pointIdx

    return alpha, r, segend, pointIdx


#-----------------------------------------------------------
# SplitLineRecursive
#
# This function executes a recursive line-slitting algorithm,
# which recursively sub-divides line segments until no further
# splitting is required.
#
# INPUT:  theta - (1D) np array of angle 'theta' from data (rads)
#           rho - (1D) np array of distance 'rho' from data (m)
#      startIdx - starting index of segment to be split
#        endIdx - ending index of segment to be split
#        params - dictionary of parameters
#
# OUTPUT: alpha - (1D) np array of 'alpha' for each fitted line (rads)
#             r - (1D) np array of 'r' for each fitted line (m)
#           idx - (N_lines,2) segment's first and last point index

def SplitLinesRecursive(theta, rho, startIdx, endIdx, params):

    N_pts = endIdx - startIdx

    # Fit a line using the N_pts points
    alpha, r = FitLine(theta[startIdx:endIdx], rho[startIdx:endIdx])

    if (N_pts <= params['MIN_POINTS_PER_SEGMENT']):
        idx = np.array([[startIdx, endIdx]]);
        return alpha, r, idx


    # Find the splitting position (if there is one)
    splitIdx = FindSplit(theta[startIdx:endIdx], rho[startIdx:endIdx], alpha, r, params)

    if (splitIdx != -1): # found a splitting point
        # if line is splitable, make recursive splitting call on each half
        alpha1, r1, idx1 = SplitLinesRecursive(theta, rho, startIdx, startIdx+splitIdx, params);
        alpha2, r2, idx2 = SplitLinesRecursive(theta, rho, startIdx+splitIdx, endIdx, params);
        alpha = np.hstack((alpha1, alpha2))
        r = np.hstack((r1, r2))
        idx = np.concatenate((idx1, idx2),axis=0)
    else: # no need to split
        idx = np.array([[startIdx, endIdx]])

    return alpha, r, idx



#-----------------------------------------------------------
# FindSplit
#
# This function takes in a line segment and outputs the best
# index at which to split the segment
#
# INPUT:  theta - (1D) np array of angle 'theta' from data (rads)
#           rho - (1D) np array of distance 'rho' from data (m)
#         alpha - 'alpha' of input line segment (1 number)
#             r - 'r' of input line segment (1 number)
#        params - dictionary of parameters
#
# OUTPUT: SplitIdx - idx at which to split line (return -1 if
#                    it cannot be split)

def FindSplit(theta, rho, alpha, r, params):

    N_pts = len(theta)

    # compute absolute distance of each point to the fitted line
    d = np.abs(rho*np.cos(theta-alpha) - r)

    # Don't want to split too close to the ends of the segments. Setting the
    # distance of endpoints to zero ensures they will not be selected for splitting.
    d[:params['MIN_POINTS_PER_SEGMENT']] = 0
    d[(N_pts-params['MIN_POINTS_PER_SEGMENT']+1):] = 0

    if (max(d) > params['LINE_POINT_DIST_THRESHOLD']):
        splitIdx = np.argmax(d)
    else:
        splitIdx = -1

    return splitIdx


#-----------------------------------------------------------
# FitLine
#
# This function outputs a best fit line to a segment of range
# data, expressed in polar form (alpha, r)
#
# INPUT:  theta - (1D) np array of angle 'theta' from data (rads)
#           rho - (1D) np array of distance 'rho' from data (m)
#     var_theta - variance in theta measurement
#       var_rho - variance in rho measurement
#
# OUTPUT: alpha - 'alpha' of best fit for range data (1 number) (rads)
#             r - 'r' of best fit for range data (1 number) (m)
#          C_AR - covariance of (alpha, r) if var_theta and var_rho are provided

def FitLine(theta, rho, var_theta = None, var_rho = None):

    N_pts = len(theta)

    rhoSquare = rho * rho
    cs  = np.cos(theta)
    cs2 = np.cos(2*theta)
    sn  = np.sin(theta)
    sn2 = np.sin(2*theta)
    thetaTemp = np.outer(theta, np.ones(N_pts))
    thetaDyadSum = thetaTemp + thetaTemp.T
    cosThetaDyadSum = np.cos(thetaDyadSum)
    rhoDyad = np.outer(rho, rho)
    csIJ = np.sum(rhoDyad * cosThetaDyadSum)

    if var_theta is not None and var_rho is not None:
        sinThetaDyadSum = np.sin(thetaDyadSum)
        grad_thetaCsIJ = -np.sum(rhoDyad * sinThetaDyadSum, axis=0) - np.sum(rhoDyad * sinThetaDyadSum, axis=1)
        grad_rhoCsIJ = 2 * rho.dot(np.cos(thetaDyadSum))

    num = rhoSquare.dot(sn2) - 2.0*rho.dot(cs)*rho.dot(sn) / N_pts
    den = rhoSquare.dot(cs2) - csIJ / N_pts
    alpha = 0.5*(np.arctan2(num, den) + np.pi)
    r = rho.dot(np.cos(theta - alpha)) / N_pts

    alphaOrg = alpha
    flipped = False
    # Let's keep r positive for consistency (though, negative r-values are fine)
    if (r < 0):
        alpha = alpha + np.pi;
        r = -r
        flipped = True
    # and let's keep alpha between -pi to pi
    if (alpha > np.pi):
        alpha = alpha - 2*np.pi
    elif (alpha < -np.pi):
        alpha = alpha + 2*np.pi

    if var_theta is not None and var_rho is not None:
        grad_rhoY = 2*sn2*rho - (2.0/N_pts)*(rho.dot(sn)*cs + rho.dot(cs)*sn)
        grad_rhoX = 2*cs2*rho - (1.0/N_pts)*(grad_rhoCsIJ)
        grad_thetaY = 2*rhoSquare*cs2 - (2.0/N_pts)*(-rho.dot(sn)*rho*sn + rho.dot(cs)*rho*cs)
        grad_thetaX = -2*rhoSquare*sn2 - (1.0/N_pts)*grad_thetaCsIJ

        if abs(den) > 1e-3:
            gradAlpha = 0.5/((num/den)**2 + 1) * (np.concatenate((grad_thetaY, grad_rhoY))/den - num/(den**2) * np.concatenate((grad_thetaX, grad_rhoX)))
        else:
            gradAlpha = -0.5/num * np.concatenate((grad_thetaX, grad_rhoX))

        grad_rhoR = (np.cos(theta - alphaOrg) + rho.dot(np.sin(theta - alphaOrg))*gradAlpha[N_pts:])/N_pts
        temp = -rho*np.sin(theta - alphaOrg)
        grad_thetaR = (temp - sum(temp)*gradAlpha[:N_pts]) / N_pts
        gradR = np.concatenate((grad_thetaR, grad_rhoR))
        if flipped:
            gradR = -gradR

        F_TR = np.vstack((gradAlpha, gradR))
        C_TR = np.diag(np.concatenate((var_theta*np.ones(N_pts), var_rho*np.ones(N_pts))))
        C_AR = F_TR.dot(C_TR).dot(F_TR.T)
        return alpha, r, C_AR

    return alpha, r

#---------------------------------------------------------------------
# MergeColinearNeigbors
#
# This function merges neighboring segments that are colinear and outputs
# a new set of line segments
#
# INPUT:  theta - (1D) np array of angle 'theta' from data (rads)
#           rho - (1D) np array of distance 'rho' from data (m)
#         alpha - (1D) np array of 'alpha' for each fitted line (rads)
#             r - (1D) np array of 'r' for each fitted line (m)
#      pointIdx - (N_lines,2) segment's first and last point indices
#        params - dictionary of parameters
#
# OUTPUT: alphaOut - output 'alpha' of merged lines (rads)
#             rOut - output 'r' of merged lines (m)
#      pointIdxOut - output start and end indices of merged line segments

def MergeColinearNeigbors(theta, rho, alpha, r, pointIdx, params):

    z = np.array([alpha[0], r[0]])
    z_test = np.zeros(2)

    startIdx = pointIdx[0, 0]
    lastEndIdx = pointIdx[0, 1]

    rOut = np.array([])
    alphaOut = np.array([])
    pointIdxOut = np.empty((0,2))

    N_segs = len(r)
    # Loop through segment enpoints
    for i in range(N_segs)[1:]:

        endIdx = pointIdx[i, 1]

        # Try fitting a line between two neighboring segments
        z_test[0], z_test[1] = FitLine(theta[startIdx:endIdx], rho[startIdx:endIdx])
        # test if this line is splitable
        splitIdx = FindSplit(theta[startIdx:endIdx], rho[startIdx:endIdx], z_test[0], z_test[1], params)

        if (splitIdx == -1): # if it cannot be split, we finally merge
            z = z_test
        else: # if it is splittable, no need to merge
            alphaOut = np.append(alphaOut, z[0])
            rOut = np.append(rOut, z[1])
            pointIdxOut = np.vstack((pointIdxOut, [startIdx, lastEndIdx]))
            z = np.array([alpha[i], r[i]])
            startIdx = pointIdx[i, 0]

        lastEndIdx = endIdx

    # add last segment
    alphaOut = np.append(alphaOut, z[0])
    rOut = np.append(rOut, z[1])
    pointIdxOut = np.vstack((pointIdxOut, [startIdx, lastEndIdx]))

    return alphaOut, rOut, pointIdxOut

def normalize_line_parameters(alpha_r):
    alpha, r = alpha_r
    r_flipped = False
    if r < 0:
        alpha = alpha + np.pi
        r = -r
        r_flipped = True
    alpha = (alpha + np.pi) % (2*np.pi) - np.pi
    return r_flipped, np.array([alpha, r])

def angle_difference(a, b):
    a = a % (2*np.pi)
    b = b % (2*np.pi)
    if abs(a - b) <= np.pi:
        return a - b
    if a > b:
        return a - b - 2*np.pi
    return a - b + 2*np.pi
