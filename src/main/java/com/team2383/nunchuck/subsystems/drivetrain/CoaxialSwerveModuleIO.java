package com.team2383.nunchuck.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface CoaxialSwerveModuleIO {
    @AutoLog
    public static class CoaxialSwerveIOInputs {
        public double moduleAngle = 0.0;
        public double absoluteAngle = 0.0;
        public double wheelVelocity = 0.0;
        public double wheelPosition = 0.0;
        public double appliedVoltsDrive = 0.0;
        public double appliedVoltsAzimuth = 0.0;
        public double driveCurrent = 0.0;
        public double azimuthCurrent = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(CoaxialSwerveIOInputs inputs) {
    }

    public default void setWheelVelocity(double velocity) {
    }

    public default void setModuleAngle(Rotation2d position) {
    }
}
