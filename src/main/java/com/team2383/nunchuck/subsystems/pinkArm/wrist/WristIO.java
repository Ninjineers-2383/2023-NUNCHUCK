package com.team2383.nunchuck.subsystems.pinkArm.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public Rotation2d angle = new Rotation2d();
        public Rotation2d velocity = new Rotation2d();
        public double appliedVolts = 0.0;
        public double current = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(WristIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }
}
