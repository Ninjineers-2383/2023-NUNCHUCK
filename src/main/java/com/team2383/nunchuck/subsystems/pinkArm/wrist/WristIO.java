package com.team2383.nunchuck.subsystems.pinkArm.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double angle = 0.0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
        public double current = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(WristIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }
}
