#ifndef ASYNCDC_GEOFENCE_H
#define ASYNCDC_GEOFENCE_H

namespace config {
    // Shared geofence values for drone application and gcs
    // NB: Changing these values currently may result in distorted visualization. This is not recommended.
    constexpr float MIN_X = -100.0f;
    constexpr float MAX_X = 100.0f;
    constexpr float MIN_Y = -100.0f;
    constexpr float MAX_Y = 100.0f;
}

#endif //ASYNCDC_GEOFENCE_H
