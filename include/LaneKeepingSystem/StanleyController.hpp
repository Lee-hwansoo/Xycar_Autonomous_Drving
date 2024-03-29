// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file StanleyController.hpp
 * @brief StanleyController Class header file
 * @version 1.0
 * @date 2023-05-23
 */
#ifndef STANLEY_CONTROLLER_HPP_
#define STANLEY_CONTROLLER_HPP_

#include <cstdint>
#include <memory>

#include "LaneKeepingSystem/Controller.hpp"

namespace Xycar {
/**
 * @brief Stanley Controller Class
 * @tparam PREC Precision of data
 */
template <typename PREC>
class StanleyController : public Controller<PREC>
{
public:
    /**
     *
     * @brief Construct a new Stanley Controller object
     *
     * @param[in] gain Stanley control gain
     * @param[in] lookAheadDistance Look-ahead distance
     */
    StanleyController(PREC gain, PREC lookAheadDistance) : mGain(gain), mLookAheadDistance(lookAheadDistance) {}

    /**
     * @brief
     *
     * @param[in] crossTrackError
     * @param[in] headingError
     * @param[in] velocity
     * @return
     */
    void calculateSteeringAngle(PREC crossTrackError, PREC headingError, PREC velocity);
    // double getResult(){return mResult;}
private:
    PREC mGain;              ///< Stanley control gain
    PREC mLookAheadDistance;
    // PREC mResult; ///< Look-ahead distance
};
} // namespace Xycar
#endif // STANLEY_CONTROLLER_HPP_
