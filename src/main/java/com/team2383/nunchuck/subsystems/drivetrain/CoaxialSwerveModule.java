package com.team2383.nunchuck.subsystems.drivetrain;

import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.team2383.nunchuck.subsystems.drivetrain.DriveConstants.ModuleConstants;
import com.team2383.lib.math.Conversions;
import com.team2383.lib.swerve.IAbsoluteEncoder;
import com.team2383.lib.util.OnboardModuleState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CoaxialSwerveModule implements Sendable {
    private final TalonFX m_angleMotor;
    private final TalonFX m_driveMotor;

    private final IAbsoluteEncoder m_angleEncoder;

    private final VelocityVoltage m_velocityOut = new VelocityVoltage(0, true, 0, 0, false);
    private final PositionVoltage m_positionOut = new PositionVoltage(0, true, 0, 0, false);

    private final ModuleConstants m_moduleConstants;

    private Rotation2d m_lastAngle;

    private final Rotation2d m_angleOffset;

    private SwerveModuleState m_desiredState;

    private double m_desiredVelocity;

    public CoaxialSwerveModule(ModuleConstants moduleConstants, IAbsoluteEncoder angleEncoder, String CANbus) {
        this.m_angleMotor = new TalonFX(moduleConstants.kAngleMotorID, CANbus);
        this.m_driveMotor = new TalonFX(moduleConstants.kDriveMotorID, CANbus);

        this.m_moduleConstants = moduleConstants;

        this.m_angleEncoder = angleEncoder;

        this.m_angleOffset = moduleConstants.kAngleOffset;

        /* Motor Config */
        configAngleMotor();
        configDriveMotor();

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
        m_driveMotor.setControl(m_velocityOut.withVelocity(m_desiredVelocity));

    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d desiredAngle = (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.kMaxSpeed * 0.01))
                ? m_lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        m_angleMotor.setControl(m_positionOut.withPosition(
                Conversions.degreesToRotations(desiredAngle.getDegrees(), DriveConstants.kAngleGearRatio)));
        m_lastAngle = desiredAngle;

    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.rotationsToDegrees(m_angleMotor.getRotorPosition().getValue(),
                        DriveConstants.kAngleGearRatio));
    }

    public Rotation2d getAbsolute() {
        return m_angleEncoder.getAbsoluteAngle();
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToRotations(
                getAbsolute().getDegrees() - m_angleOffset.getDegrees(),
                DriveConstants.kAngleGearRatio);
        m_angleMotor.setRotorPosition(absolutePosition);
    }

    private void configAngleMotor() {
        m_angleMotor.getConfigurator().apply(m_moduleConstants.kHardwareConfigs.kAngleMotorConfigs);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        m_driveMotor.getConfigurator().apply(m_moduleConstants.kHardwareConfigs.kDriveMotorConfigs);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPMToMPS(m_driveMotor.getVelocity().getValue(),
                        DriveConstants.kDriveWheelCircumferenceMeters,
                        DriveConstants.kDriveGearRatio),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(m_driveMotor.getPosition().getValue(),
                        DriveConstants.kDriveWheelCircumferenceMeters,
                        DriveConstants.kDriveGearRatio),
                getAngle());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle", () -> {
            return getAngle().getDegrees();
        }, null);
        builder.addDoubleProperty("Absolute Angle", () -> {
            return getAbsolute().getDegrees();
        }, null);
        builder.addDoubleProperty("Speed", () -> {
            return getState().speedMetersPerSecond;
        }, null);
        builder.addDoubleProperty("Control Speed", () -> {
            return m_desiredVelocity;
        }, null);

        builder.addDoubleProperty("Actual Velocity", () -> {
            return m_driveMotor.getVelocity().getValue();
        }, null);

    }
}
