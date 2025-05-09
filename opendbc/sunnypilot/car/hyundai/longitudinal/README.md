# **How custom longitudinal tuning works for Hyundai/Kia/Genesis vehicles in sunnypilot.**

This document explains the custom longitudinal control tuning implemented in sunnypilot for Hyundai, Kia, and Genesis vehicles.

**Safety First: ISO 15622:2018**

Our primary safety guideline is [ISO 15622:2018](https://www.iso.org/obp/ui/en/#iso:std:iso:15622:ed-3:v1:en).
This standard provides the foundation for the safety limits enforced by this tuning, particularly concerning maximum allowable jerk based on speed.

**Core Concepts**

1.  **Jerk-Limited Acceleration:** Instead of directly applying the planner's target acceleration (`CC.actuators.accel`), we use a jerk-limited integrator. This ensures smoother transitions by limiting the rate of change of acceleration (jerk).
    *   `desired_accel`: The target acceleration from the planner, clipped within safe limits (`CarControllerParams.ACCEL_MIN`, `CarControllerParams.ACCEL_MAX`). During stopping, this is forced to 0.
    *   `actual_accel`: The acceleration command sent to the car. It's calculated by integrating `desired_accel` but limiting the change from the previous step (`state.accel_last`) based on calculated `jerk_upper` and `jerk_lower` limits.
    *   `jerk_limited_integrator(desired_accel, last_accel, jerk_upper, jerk_lower)`: This function calculates the maximum allowable acceleration change per control step (`DT_CTRL * 2`) based on the upper and lower jerk limits and applies it using `rate_limit`.

2.  **Predictive Jerk Limits:** The upper and lower jerk limits (`jerk_upper`, `jerk_lower`) are not fixed but calculated dynamically based on several factors:
    *   **Actual Acceleration (`aego`):** The car's current acceleration (`CS.out.aEgo`, potentially blended with `CS.aBasis` at low speeds) is smoothed using a `FirstOrderFilter` (`self.aego`).
    *   **Acceleration Error:** The difference between the *commanded* acceleration (`self.accel_cmd`) and the *actual* smoothed acceleration (`self.aego.x`).
    *   **Lookahead Jerk:** We estimate the jerk required to reach the target acceleration (`accel_cmd`) within a future time window. This window (`future_t_upper`, `future_t_lower`) varies with speed, defined in `config.py` (`lookahead_jerk_bp`, `lookahead_jerk_upper_v`, `lookahead_jerk_lower_v`).
        *   `j_ego_upper = accel_error / future_t_upper`
        *   `j_ego_lower = accel_error / future_t_lower`
    *   **ISO 15622 Speed Factors:** Maximum allowed jerk decreases with speed according to ISO 15622. We interpolate values based on current velocity (`CS.out.vEgo`) to get `upper_speed_factor` and `lower_speed_factor`.
    *   **Minimum Jerk (`MIN_JERK`):** A baseline minimum jerk (currently 0.5 m/s³) is enforced to ensure responsiveness.
    *   **Combining Factors:**
        *   `desired_jerk_upper = min(max(j_ego_upper, MIN_JERK), upper_speed_factor)`
        *   `desired_jerk_lower = min(max(-j_ego_lower, MIN_JERK), lower_speed_factor)` (Note: `j_ego_lower` is negated for the lower limit calculation).

3.  **Dynamic Braking Tune:** For use of the tune without the `LONG_TUNING_PREDICTIVE` flag, a specific negative jerk profile is applied during braking (`gen1_accel_error < 0`).
    *   `gen1_accel_error = a_ego_blended - self.state.accel_last`
    *   A dynamic lower jerk limit (`gen1_lower_jerk`) is interpolated based on this error using `DYNAMIC_LOWER_JERK_BP` and a dynamically scaled `DYNAMIC_LOWER_JERK_V` (scaled by `car_config.jerk_limits`).
    *   `gen1_desired_jerk_lower = min(gen1_lower_jerk, lower_speed_factor)`

4.  **Applying Jerk Limits:**
    *   The `jerk_upper` limit is always smoothed using `ramp_update` to prevent abrupt changes.
    *   The `jerk_lower` limit is applied differently based on the `LONG_TUNING_PREDICTIVE` flag:
        *   **If Flag Set (predictive tune):** `jerk_lower` is set directly to `desired_jerk_lower`.
        *   **If Flag Not Set (dynamic tune):** `jerk_lower` is smoothed using `ramp_update` towards `gen1_desired_jerk_lower`.

**Configuration (`config.py`)**

*   The `CarTuningConfig` dataclass holds tuning parameters like stopping/starting speeds, lookahead jerk profiles, actuator delay, and base jerk limits.
*   `TUNING_CONFIGS` provides default configurations based on car type (CANFD, EV, HYBRID, DEFAULT).
*   `CAR_SPECIFIC_CONFIGS` allows overriding defaults for specific car models (e.g., `KIA_NIRO_EV`, `HYUNDAI_IONIQ`).
*   The `get_car_config` helper function selects the appropriate configuration based on the car fingerprint and flags.
*   `get_longitudinal_tune` applies relevant config values (like stopping/starting speeds, delay) to the main `CarParams` (CP).

**Summary**

This tuning aims for smoother and more responsive longitudinal control by:
*   Using a jerk-limited integrator for acceleration commands.
*   Dynamically calculating jerk limits based on lookahead predictions, actual acceleration, and ISO 15622 speed constraints.
*   Providing specific tuning profiles via a configuration system based on car type and model.
*   Applying special braking logic for use of predictive or dynamic tune.
