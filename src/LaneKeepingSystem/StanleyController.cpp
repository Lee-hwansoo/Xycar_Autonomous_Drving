// Copyright (C) 2023 Grepp CO.
// All rights reserved.

#include "LaneKeepingSystem/StanleyController.hpp"
#include <cmath>

namespace Xycar {

template <typename PREC>
void StanleyController<PREC>::calculateSteeringAngle(PREC crossTrackError, PREC headingError, PREC velocity)
{
    // Calculate the cross-track error (cte) compensation
    PREC alpha = std::atan2(mGain * crossTrackError, velocity);

    // Calculate the desired heading angle
    PREC desiredHeading = this->normalizeAngle(headingError) + alpha;

    // Calculate the steering angle using the desired heading and look-ahead distance
    this->mResult = std::atan2(2 * mLookAheadDistance * std::sin(desiredHeading), velocity) * (180.0 / M_PI);
    // this->mResult = desiredHeading * (180.0 / M_PI);
}

template class StanleyController<float>;
template class StanleyController<double>;
} // namespace Xycar
