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
	/* XXX ajs 7/Dec/2017 indices into values array might need
	 * to be swapped below
	 */
	double values[3][3];
	R.get((double **) values);

	double sy = sqrt(
		values[0][0] * values[0][0] + values[1][0] * values[1][0]
	);

	double x, y, z;
	if (sy < singularThreshold)
	{
		// Matrix is singular
		x = atan2(-values[1][2], values[1][1]);
		y = atan2(-values[2][0], sy);
		z = 0;
	}
	else
	{
		// Matrix is not singular
		x = atan2(values[2][1], values[2][2]);
		y = atan2(-values[2][0], sy);
		z = atan2(values[1][0], values[0][0]);
	}

	return cVector3d(x, y, z);
}
