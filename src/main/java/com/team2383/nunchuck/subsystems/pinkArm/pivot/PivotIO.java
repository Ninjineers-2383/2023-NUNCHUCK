package com.team2383.nunchuck.subsystems.pinkArm.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double angle = 0.0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
        public double currentLeft = 0.0;
        public double currentRight = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(PivotIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }
}
