package com.team2383.diffy.subsystems.drivetrain;

import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.sim.CANcoderSimState;
import com.ctre.phoenixpro.sim.TalonFXSimState;
import com.team2383.diffy.subsystems.drivetrain.DriveConstants.ModuleConstants;
import com.team2383.lib.math.Conversions;
import com.team2383.lib.util.OnboardModuleState;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CoaxialSwerveModule implements Sendable {
    private final TalonFX m_angleMotor;
    private final TalonFX m_driveMotor;
    private final CANcoder m_angleEncoder;

    private final TalonFXSimState m_angleMotorSim;
    private final TalonFXSimState m_driveMotorSim;
    private final CANcoderSimState m_anglerEncoderSim;

    private final VelocityVoltage m_velocityOut = new VelocityVoltage(0, true, 0, 0, false);
    private final PositionVoltage m_positionOut = new PositionVoltage(0, true, 0, 0, false);

    private final SimpleMotorFeedforward m_driveFeedForward;

    private final ModuleConstants m_moduleConstants;

    private final String m_name;
    private final DataLog m_log;

    private Rotation2d m_lastAngle;

    private final Rotation2d m_angleOffset;

    public CoaxialSwerveModule(ModuleConstants moduleConstants, String CANbus, DataLog log) {
        this.m_angleMotor = new TalonFX(moduleConstants.kAngleMotorID, CANbus);
        this.m_driveMotor = new TalonFX(moduleConstants.kDriveMotorID, CANbus);

        this.m_angleEncoder = new CANcoder(moduleConstants.kEncoderID, CANbus);

        this.m_angleMotorSim = m_angleMotor.getSimState();
        this.m_driveMotorSim = m_driveMotor.getSimState();

        this.m_anglerEncoderSim = m_angleEncoder.getSimState();

        this.m_driveFeedForward = new SimpleMotorFeedforward(moduleConstants.kS, moduleConstants.kV,
                moduleConstants.kA);

        this.m_moduleConstants = moduleConstants;

        this.m_name = moduleConstants.name;
        this.m_log = log;

        this.m_angleOffset = moduleConstants.kAngleOffset;

        configAngleEncoder();

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
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState);
    }

    private void setSpeed(SwerveModuleState desiredState) {
        double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                DriveConstants.kDriveWheelCircumferenceMeters, DriveConstants.kDriveGearRatio);
        m_driveMotor.setControl(m_velocityOut.withVelocity(velocity));
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.kMaxSpeed * 0.01))
                ? m_lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        m_angleMotor.setControl(m_positionOut.withPosition(
                Conversions.degreesToRotations(angle.getDegrees(), DriveConstants.kAngleGearRatio)));
        m_lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.rotationsToDegrees(m_angleMotor.getRotorPosition().getValue(),
                        DriveConstants.kAngleGearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToRotations(
                getCanCoder().getDegrees() - m_angleOffset.getDegrees(),
                DriveConstants.kAngleGearRatio);
        m_angleMotor.setRotorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        m_angleEncoder.getConfigurator().apply(m_moduleConstants.kHardwareConfigs.kAngleEncoderConfigs);
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
            return m_angleEncoder.getAbsolutePosition().getValue() * 360;
        }, null);
        builder.addDoubleProperty("Speed", () -> {
            return getState().speedMetersPerSecond;
        }, null);
    }
}
