#ifndef DYNAMIC_CAMERA_H
#define DYNAMIC_CAMERA_H

#include <iostream>
#include <limits>

#include "camera.h"

constexpr float INF = std::numeric_limits<float>::infinity();

// AABB in 1D
struct Interval {
    float min;
    float max;

    constexpr bool contains(const Interval& other) const {
        return (min <= other.min) && (max >= other.max);
    }

    constexpr bool contains(float x) const { return (min <= x) && (x <= max); }

    // signedDistance returns a positive smallest distance of x from the interval boundary
    // if it is not contained, else it returns a smallest (in magnitude) negative distance
    // from the interval boundary.
    constexpr float signedDistance(float x) const {
        if (contains(x)) {
            // 0 or negative
            return std::max(x - max, min - x);
        }
        // positive
        return std::max(x - max, min - x);
    }
};

struct DynamicCamera {
    void update(Camera& camera,                           // the camera to update
                const sf::Vector2f& trackedPosWorld,      // world coords of player to track
                const sf::Vector2f& trackedVelocityWorld  // world velocity of player to track
    ) {
        // Set the camera center to the unadjusted center from last fram with no lookahead
        camera.setCameraCenter(cameraUnadjustedCenter);

        // lookDir checks the direction of the tracked velocity, and lookAheadWorld is the world
        // distance to look ahead in that direction
        // Then we update the lookaheadState using an EMA with alpha
        float lookDir = 0.f;
        if (trackedVelocityWorld.x > 0.f) {
            lookDir = 1.f;
        } else if (trackedVelocityWorld.x < 0.f) {
            lookDir = -1.f;
        }
        float lookAheadWorld = camera.windowPercentToWorldDist({lookAhead * lookDir, 0.f}).x;
        lookaheadState = lookaheadState + alpha * (lookAheadWorld - lookaheadState);

        // normalized player position in window percent coordinates
        const float playerPercentX = camera.worldToWindowPercent(trackedPosWorld).x;
        const float leftHard = softZoneX.min;   // red
        const float leftSoft = deadZoneX.min;   // green
        const float rightSoft = deadZoneX.max;  // green
        const float rightHard = softZoneX.max;  // red

        float deltaPercentX = 0.f;
        if (playerPercentX < leftHard) {
            // past the hard left limit, put the player on the red line
            deltaPercentX = playerPercentX - leftHard;
        } else if (playerPercentX > rightHard) {
            deltaPercentX = playerPercentX - rightHard;
        } else if (playerPercentX < leftSoft) {
            // past the soft left limit, ease toward the green line
            deltaPercentX = alpha * (playerPercentX - leftSoft);
        } else if (playerPercentX > rightSoft) {
            deltaPercentX = alpha * (playerPercentX - rightSoft);
        }
        // get the world distance to move the camera center by, and update the unadjusted camera
        // center
        float deltaWorldX = camera.windowPercentToWorldDist({deltaPercentX, 0.f}).x;
        cameraUnadjustedCenter.x += deltaWorldX;
        camera.setCameraCenter(
            {cameraUnadjustedCenter.x + lookaheadState, cameraUnadjustedCenter.y});
    }

    float alpha;  // alpha to use for damping of camera position, *as well as* for smoothing of
                  // lookahead offset.
    Interval deadZoneX;  // percentage interval along x-axis to use for dead zone in [0, 1]
    // percentage interval along x-axis to use for soft rejection zone in [0, 1]
    Interval softZoneX = {-INF, INF};
    // (everything beyond softZone is considered hard zone).
    float lookAhead;  // screen-percentage lookahead
    mutable sf::Vector2f
        cameraUnadjustedCenter;      // the camera world center from last frame, ignoring lookahead
    mutable float lookaheadState{};  // the EMA of the accumulated lookahead offset in world coords
};

#endif  // DYNAMIC_CAMERA_H
