package com.team2383.diffy.subsystems.paddle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PaddleSubsystem extends SubsystemBase {

    private final TalonSRX m_dick;

    private Rotation2d m_angle = new Rotation2d();

    public PaddleSubsystem() {
        m_dick = new TalonSRX(PaddleConstants.ID);
        m_dick.configFactoryDefault();
        m_dick.setInverted(false);
        m_dick.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 200);
        m_dick.enableCurrentLimit(true);
        m_dick.configPeakCurrentLimit(PaddleConstants.kMaxCurrent, 500);
        m_dick.setSelectedSensorPosition(0);

    }

    @Override
    public void periodic() {
        setVoltage(calculateVoltage(m_angle));
    }

    public void setVoltage(double dutyCycle) {
        m_dick.set(ControlMode.PercentOutput, dutyCycle);
    }

    public void setPosition(Rotation2d angle) {
        m_angle = angle;
    }

    public double calculateVoltage(Rotation2d angle) {
        double voltage = PaddleConstants.PID_CONTROLLER.calculate(getAngle().getRadians(), angle.getRadians());
        return voltage;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_dick.getSelectedSensorPosition() / 4096.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Angle", () -> {
            return getAngle().getRadians();
        }, null);
    }
}
