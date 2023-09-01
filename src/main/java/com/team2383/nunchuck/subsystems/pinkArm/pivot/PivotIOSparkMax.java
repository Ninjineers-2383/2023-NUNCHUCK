package com.team2383.nunchuck.subsystems.pinkArm.pivot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.lib.simulation.SparkMaxSimWrapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOSparkMax implements PivotIO {
    private final SparkMaxSimWrapper m_rightMotor;
    private final SparkMaxSimWrapper m_leftMotor;

    private final DutyCycleEncoder m_absEncoder;

    public PivotIOSparkMax() {
        m_leftMotor = new SparkMaxSimWrapper(PivotConstants.BOTTOM_MOTOR_LEFT_ID, MotorType.kBrushless);
        m_rightMotor = new SparkMaxSimWrapper(PivotConstants.BOTTOM_MOTOR_RIGHT_ID, MotorType.kBrushless);
        m_rightMotor.restoreFactoryDefaults();
        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.setVelocityConversionFactor(PivotConstants.VELOCITY_CONVERSION_FACTOR.getRadians());
        m_leftMotor.setVelocityConversionFactor(PivotConstants.VELOCITY_CONVERSION_FACTOR.getRadians());
        m_leftMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);
        m_rightMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);
        m_leftMotor.setInverted(false);
        m_rightMotor.setInverted(true);

        m_absEncoder = new DutyCycleEncoder(PivotConstants.ABS_ENCODER_ID);

        m_absEncoder.setPositionOffset(PivotConstants.ENCODER_OFFSET);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.angle = Rotation2d.fromRotations(m_absEncoder.get());
        inputs.velocity = m_leftMotor.get();
        inputs.appliedVolts = m_leftMotor.getAppliedOutput();
        inputs.currentLeft = 0.0;
        inputs.currentRight = 0.0;
    }

    @Override
    public void setVoltage(double voltage) {
        m_leftMotor.setVoltage(voltage);
        m_rightMotor.setVoltage(voltage);
    }
}
