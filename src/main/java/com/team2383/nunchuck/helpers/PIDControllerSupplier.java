/*
 * Use is subject to Henry.
 *
 *
 */

package com.team2383.nunchuck.helpers;

import edu.wpi.first.math.controller.PIDController;

public interface PIDControllerSupplier {
    PIDController getAsPIDController();
}
