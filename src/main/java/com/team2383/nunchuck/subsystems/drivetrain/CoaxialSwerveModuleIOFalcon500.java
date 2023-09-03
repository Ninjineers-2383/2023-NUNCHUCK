package com.team2383.nunchuck.subsystems.drivetrain;

import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.team2383.lib.math.Conversions;
import com.team2383.lib.swerve.IAbsoluteEncoder;
import com.team2383.nunchuck.Constants;
import com.team2383.nunchuck.subsystems.drivetrain.DriveConstants.ModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;

public class CoaxialSwerveModuleIOFalcon500 implements CoaxialSwerveModuleIO {
    private final TalonFX m_angleMotor;
    private final TalonFX m_driveMotor;

    private final IAbsoluteEncoder m_angleEncoder;

    private final VelocityVoltage m_velocityOut = new VelocityVoltage(0, true, 0, 0, false);
    private final PositionVoltage m_positionOut = new PositionVoltage(0, true, 0, 0, false);

    private final ModuleConstants m_moduleConstants;

    private final Rotation2d m_angleOffset;

    public CoaxialSwerveModuleIOFalcon500(int index) {
        switch (index) {
            case 0:
                m_moduleConstants = DriveConstants.frontLeftConstants;
                m_angleEncoder = DriveConstants.frontLeftEncoder;
              break;
            case 1:
                m_moduleConstants = DriveConstants.frontRightConstants;
                m_angleEncoder = DriveConstants.frontRightEncoder;
              break;
            case 2:
                m_moduleConstants = DriveConstants.rearConstants;
                m_angleEncoder = DriveConstants.rearEncoder;
              break;
            default:
              throw new RuntimeException("Invalid module index for CoaxialSwerveModuleIOFalcon500");
          }

        this.m_angleMotor = new TalonFX(m_moduleConstants.kAngleMotorID, Constants.kCANivoreBus);
        this.m_driveMotor = new TalonFX(m_moduleConstants.kDriveMotorID, Constants.kCANivoreBus);

        this.m_angleOffset = m_moduleConstants.kAngleOffset;

        /* Motor Config */
        configAngleMotor();
        configDriveMotor();
    }
    
    @Override
    public void updateInputs(CoaxialSwerveIOInputs inputs) {
        inputs.moduleAngle = Conversions.rotationsToRadians(m_angleMotor.getRotorPosition().getValue(),
            DriveConstants.kAngleGearRatio);
        inputs.absoluteAngle = m_angleEncoder.getAbsoluteAngle().getRadians();
        inputs.wheelVelocity = Conversions.RPSToMPS(m_driveMotor.getVelocity().getValue(),
            DriveConstants.kDriveWheelCircumferenceMeters,
            DriveConstants.kDriveGearRatio);
        inputs.wheelPosition = Conversions.rotationsToMeters(m_driveMotor.getPosition().getValue(),
            DriveConstants.kDriveWheelCircumferenceMeters,
            DriveConstants.kDriveGearRatio);
        inputs.appliedVoltsDrive = m_driveMotor.getSupplyVoltage().getValue();
        inputs.appliedVoltsAzimuth = m_angleMotor.getSupplyVoltage().getValue();
        inputs.driveCurrent = m_driveMotor.getSupplyCurrent().getValue();
        inputs.azimuthCurrent = m_angleMotor.getSupplyCurrent().getValue();
    }

    @Override
    public void setWheelVelocity(double velocity) {
        m_driveMotor.setControl(m_velocityOut.withVelocity(velocity));
    }

    @Override
    public void setModuleAngle(Rotation2d desiredAngle) {
        m_angleMotor.setControl(m_positionOut.withPosition(
            Conversions.degreesToRotations(desiredAngle.getDegrees(), DriveConstants.kAngleGearRatio)));
    }

    private void configAngleMotor() {
        m_angleMotor.getConfigurator().apply(m_moduleConstants.kHardwareConfigs.kAngleMotorConfigs);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        m_driveMotor.getConfigurator().apply(m_moduleConstants.kHardwareConfigs.kDriveMotorConfigs);
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToRotations(
            m_angleEncoder.getAbsoluteAngle().getDegrees() - m_angleOffset.getDegrees(),
            DriveConstants.kAngleGearRatio);
        m_angleMotor.setRotorPosition(absolutePosition);
    }
}
