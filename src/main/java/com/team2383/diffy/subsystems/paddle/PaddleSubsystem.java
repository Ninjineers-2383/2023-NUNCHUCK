package com.team2383.diffy.subsystems.paddle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PaddleSubsystem extends SubsystemBase {

    private final TalonSRX m_dick;

    private Rotation2d m_setAngle = new Rotation2d();

    private Rotation2d m_setVelocity = new Rotation2d();

    public PaddleSubsystem() {
        m_dick = new TalonSRX(PaddleConstants.DICK_ID);
        m_dick.configFactoryDefault();
        m_dick.setInverted(false);
        m_dick.getSensorCollection().setQuadraturePosition(0, 200);
        m_dick.enableCurrentLimit(true);
        m_dick.configPeakCurrentLimit(PaddleConstants.kMaxCurrent, 500);
    }

    @Override
    public void periodic() {
        m_setVelocity = Rotation2d.fromRadians(
                MathUtil.clamp(PaddleConstants.kP * (m_setAngle.getRadians() - getAngle().getRadians()),
                        -PaddleConstants.maxVelocity.getRadians(), PaddleConstants.maxVelocity.getRadians()));
        setVoltage(
                calculateVoltage(m_setVelocity));
    }

    public void setVoltage(double dutyCycle) {
        m_dick.set(ControlMode.PercentOutput, dutyCycle);
    }

    public void setPosition(Rotation2d angle) {
        m_setAngle = angle;
    }

    public double calculateVoltage(Rotation2d velocity) {
        double voltage = PaddleConstants.PID_CONTROLLER.calculate(getVelocity().getRadians(), velocity.getRadians());
        return voltage;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(
                (m_dick.getSensorCollection().getQuadraturePosition() - PaddleConstants.encoderOffset) / 4096.0);
    }

    public Rotation2d getVelocity() {
        return Rotation2d.fromRotations(m_dick.getSensorCollection().getQuadratureVelocity() / 4096.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Angle", () -> {
            return getAngle().getRadians();
        }, null);

        builder.addDoubleProperty("Set Angle", () -> {
            return m_setAngle.getRadians();
        }, null);

        builder.addDoubleProperty("Velocity", () -> {
            return getVelocity().getRadians();
        }, null);

        builder.addDoubleProperty("Set Velocity", () -> {
            return m_setVelocity.getRadians();
        }, null);

        builder.addDoubleProperty("ABS Raw", m_dick.getSensorCollection()::getPulseWidthPosition, null);

        builder.addDoubleProperty("Quad Raw", m_dick.getSensorCollection()::getQuadraturePosition, null);
    }
}
