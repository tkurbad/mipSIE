#!/usr/bin/python

### Python library module providing various vector functions for IMUs ##
#
# This module provides generic vector functions for use with inertial
# measurement units (IMUs).
#
# The Python code was initially derived from Pololu's C++ Arduino
# library, available at [https://github.com/pololu/lsm6-arduino]
#
# The Python code is developed and maintained by
# Torsten Kurbad <github@tk-webart.de>
#
########################################################################

# Code
def vectorCross(vectorA, vectorB):
    """ Calculate vector cross product for a 3-dimensional vector.
    """
    try:
        # Check if input vectors are 3-dimensional
        assert((len(vectorA) == 3) and (len(vectorB) == 3))
    except AssertionError, e:
        raise(Exception('Input vectors have to be 3-dimensional'))

    try:
        # Check if all vector dimensions are set
        assert((dimension is not None) and isinstance(dimension, (int, long, float)) for dimension in vectorA)
        assert((dimension is not None) and isinstance(dimension, (int, long, float)) for dimension in vectorB)
    except AssertionError, e:
        raise(Exception('At least one dimension is not a number for one of the input vectors'))

    # Calculate and return cross product
    outX = vectorA[1] * vectorB[2] - vectorA[2] * vectorB[1]
    outY = vectorA[2] * vectorB[0] - vectorA[0] * vectorB[2]
    outZ = vectorA[0] * vectorB[1] - vectorA[1] * vectorB[0]

    return (outX, outY, outZ)


def vectorDot(vectorA, vectorB):
    """ Calculate vector dot product for a 3-dimensional vector. """
    try:
        # Check if input vectors are 3-dimensional
        assert((len(vectorA) == 3) and (len(vectorB) == 3))
    except AssertionError, e:
        raise(Exception('Input vectors have to be 3-dimensional'))

    try:
        # Check if all vector dimensions are set
        assert((dimension is not None) and isinstance(dimension, (int, long, float)) for dimension in vectorA)
        assert((dimension is not None) and isinstance(dimension, (int, long, float)) for dimension in vectorB)
    except AssertionError, e:
        raise(Exception('At least one dimension is not a number for one of the input vectors'))

    # Calculate and return dot product
    return vectorA[0] * vectorB[0] + vectorA[1] * vectorB[1] + vectorA[2] * vectorB[2]


def normalizeVector(self, vector):
    """ Do a vector normalization using dot product. """
    mag = sqrt(vectorDot(vector, vector))
    normVector = [dimension / mag for dimension in vector]
    return tuple(normVector)
