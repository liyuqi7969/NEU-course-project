#include "util.h"

#define __BSD_VISIBLE
#include "math.h"
/**
 * Make sure that -pi <= angle < pi,
 *
 * TODO: Maybe use fmodf instead?
 *
 * @param angle
 * The angle to normalize in radians.
 * WARNING: Don't use too large angles.
 */
void utils_norm_angle_rad_Q22(_iq22 *angle) {
	while (*angle < _IQ22(-M_PI)) {
		*angle += _IQ22(2.0 * M_PI);
	}

	while (*angle >  _IQ22(M_PI)) {
		*angle -= _IQ22(2.0 * M_PI);
	}
}
