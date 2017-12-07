#pragma once

// Some utility functions

#include "chai3d.h"


using namespace chai3d;


/**
 * Calculates rotation matrix to euler angles in order 'XYZ'
 * Implementation from https://www.learnopencv.com/rotation-matrix-to-euler-angles/
 */
cVector3d rotationMatrixToEulerAngles(cMatrix3d R, float singularThreshold = 1e-6)
{
	// XXX ajs 7/Dec/2017 indices into R might need to be swapped below
	
	double sy = sqrt(
		R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)
	);

	double x, y, z;
	if (sy < singularThreshold)
	{
		// Matrix is singular
		x = atan2(-R(1, 2), R(1, 1));
		y = atan2(-R(2, 0), sy);
		z = 0;
	}
	else
	{
		// Matrix is not singular
		x = atan2(R(2, 1), R(2, 2));
		y = atan2(-R(2, 0), sy);
		z = atan2(R(1, 0), R(0, 0));
	}

	return cVector3d(x, y, z);
}
