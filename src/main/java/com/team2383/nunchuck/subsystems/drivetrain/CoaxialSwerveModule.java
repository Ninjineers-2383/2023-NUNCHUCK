package com.team2383.nunchuck.subsystems.drivetrain;

import com.team2383.lib.math.Conversions;
import com.team2383.lib.util.OnboardModuleState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CoaxialSwerveModule {
    private final CoaxialSwerveModuleIO m_io;
    private final CoaxialSwerveIOInputsAutoLogged m_inputs = new CoaxialSwerveIOInputsAutoLogged();

    private Rotation2d m_lastAngle;

    private SwerveModuleState m_desiredState;

    private double m_desiredVelocity;

    public CoaxialSwerveModule(CoaxialSwerveModuleIO io) {
        this.m_io = io;

        m_lastAngle = getState().angle;
    }

    public void simulate() {

    }

    public void setDesiredState(SwerveModuleState desiredState) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        m_desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        setAngle(m_desiredState);
        setSpeed(m_desiredState);
    }

    private void setSpeed(SwerveModuleState desiredState) {
        m_desiredVelocity = Conversions.MPSToFalconRPS(desiredState.speedMetersPerSecond,
                DriveConstants.kDriveWheelCircumferenceMeters, DriveConstants.kDriveGearRatio);
        m_io.setWheelVelocity(m_desiredVelocity);

    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d desiredAngle = (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.kMaxSpeed * 0.01))
                ? m_lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        m_io.setModuleAngle(desiredAngle);

        m_lastAngle = desiredAngle;

    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_inputs.moduleAngle);
    }

    public Rotation2d getAbsolute() {
        return Rotation2d.fromRadians(m_inputs.absoluteAngle);
    }

    public double getWheelVelocity() {
        return m_inputs.wheelVelocity;
    }

    public double getWheelPosition() {
        return m_inputs.wheelPosition;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getWheelVelocity(),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getWheelPosition(),
                getAngle());
    }
}
