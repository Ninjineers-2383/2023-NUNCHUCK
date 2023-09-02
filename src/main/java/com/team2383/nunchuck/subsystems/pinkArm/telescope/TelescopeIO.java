package com.team2383.nunchuck.subsystems.pinkArm.telescope;

import org.littletonrobotics.junction.AutoLog;

public interface TelescopeIO {
    @AutoLog
    public static class TelescopeIOInputs {
        public double extension = 0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
        public double current = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(TelescopeIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setEncoderPosition(double position) {
    }
}
