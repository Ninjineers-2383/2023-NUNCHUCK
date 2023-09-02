package com.team2383.nunchuck.subsystems.pinkArm.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.lib.math.AngularVelocityWrapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOSparkMax implements PivotIO {
    private final CANSparkMax m_rightMotor;
    private final CANSparkMax m_leftMotor;

    private final DutyCycleEncoder m_absEncoder;

    private AngularVelocityWrapper m_velocity;

    public PivotIOSparkMax() {
        m_leftMotor = new CANSparkMax(PivotConstants.BOTTOM_MOTOR_LEFT_ID, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(PivotConstants.BOTTOM_MOTOR_RIGHT_ID, MotorType.kBrushless);
        m_rightMotor.restoreFactoryDefaults();
        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.getEncoder().setVelocityConversionFactor(PivotConstants.VELOCITY_CONVERSION_FACTOR.getRadians());
        m_leftMotor.getEncoder().setVelocityConversionFactor(PivotConstants.VELOCITY_CONVERSION_FACTOR.getRadians());
        m_leftMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);
        m_rightMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);
        m_leftMotor.setInverted(false);
        m_rightMotor.setInverted(true);

        m_absEncoder = new DutyCycleEncoder(PivotConstants.ABS_ENCODER_ID);

        m_absEncoder.setPositionOffset(PivotConstants.ENCODER_OFFSET);

        m_velocity = new AngularVelocityWrapper(Rotation2d.fromRotations(m_absEncoder.get()));
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.angle = m_absEncoder.get();
        inputs.velocity = m_velocity.calculate(Rotation2d.fromRotations(inputs.angle)).getRotations();
        inputs.appliedVolts = m_leftMotor.getAppliedOutput();
        inputs.currentLeft = m_leftMotor.getOutputCurrent();
        inputs.currentRight = m_rightMotor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double voltage) {
        m_leftMotor.setVoltage(voltage);
        m_rightMotor.setVoltage(voltage);
    }
}
