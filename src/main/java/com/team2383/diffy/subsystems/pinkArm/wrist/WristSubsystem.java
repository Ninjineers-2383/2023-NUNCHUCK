package com.team2383.diffy.subsystems.pinkArm.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team2383.diffy.Robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
    private final TalonSRX m_pivotMotor;

    private final LinearSystem<N1, N1, N1> m_motorSim = LinearSystemId
            .identifyVelocitySystem(WristConstants.kV, WristConstants.kA);

    private final PIDController m_fb = new PIDController(WristConstants.kP, 0, 0);

    private double m_voltage;

    // In radians
    private double m_desiredAngle;
    private double m_pivotAngle;
    private double m_angle;
    private double m_prevAngle = 0;

    // In radians per second
    private double m_velocity = 0;

    double m_simVelocity = 0;

    public WristSubsystem() {
        m_pivotMotor = new TalonSRX(WristConstants.kMotorID);
        m_pivotMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 200);
        m_pivotMotor.enableCurrentLimit(true);
        m_pivotMotor.configPeakCurrentLimit(20, 500);
    }

    public void periodic() {
        m_angle = getAngleRadians();

        m_velocity = (m_angle - m_prevAngle) / 0.02;
        m_prevAngle = m_angle;

        calculateVoltage();
    }

    @Override
    public void simulationPeriodic() {
        double newVel = m_motorSim.calculateX(VecBuilder.fill(m_velocity), VecBuilder.fill(m_voltage), 0.02).get(0,
                0);

        m_pivotMotor.set(TalonSRXControlMode.Velocity, newVel);

        // SmartDashboard.putNumber("Simulated Top Pivot Motor Output Velocity", null);

        SmartDashboard.putNumber("Simulated Encoder Radians", getAngleRadians());
    }

    public void setAngle(double desiredAngle) {
        if (m_desiredAngle > WristConstants.kUpperBound) {
            m_desiredAngle = WristConstants.kUpperBound;
        } else if (m_desiredAngle < WristConstants.kLowerBound) {
            m_desiredAngle = WristConstants.kLowerBound;
        } else {
            m_desiredAngle = desiredAngle;
        }
    }

    private void calculateVoltage() {
        m_voltage = m_fb.calculate(m_angle, m_desiredAngle);
        m_voltage += Math.signum(m_voltage) * WristConstants.kS;

        if (Robot.isReal()) {
            m_voltage += Math.sin(m_angle + m_pivotAngle) * WristConstants.kG;
        }
        setVoltage();
    }

    public void setVelocity(double desiredSpeed) {
        m_desiredAngle += desiredSpeed * 0.02;
        setAngle(m_desiredAngle);
    }

    public double getAngleRadians() {
        return (m_pivotMotor.getSelectedSensorPosition() + 2200) * Math.PI / 2000.0;
    }

    public double getAngleDegrees() {
        return Units.radiansToDegrees(getAngleRadians());
    }

    public void setVoltage() {
        m_pivotMotor.set(ControlMode.PercentOutput, m_voltage / 12);
    }

    public boolean isAtPosition() {
        return Math.abs(m_desiredAngle - m_angle) < 1;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pivot");
        
        builder.addDoubleProperty("Angle (Raw)", () -> {
            return m_pivotMotor.getSelectedSensorPosition() + 2200;
        }, null);

        builder.addDoubleProperty("Angle (Deg)", () -> {
            return getAngleDegrees();
        }, null);

        builder.addDoubleProperty("Desired Angle (Deg)", () -> {
            return m_desiredAngle;
        }, null);

        builder.addDoubleProperty("Velocity (Deg per sec)", () -> {
            return Units.radiansToDegrees(m_velocity);
        }, null);

        builder.addDoubleProperty("Voltage (Volts)", () -> {
            return m_voltage;
        }, null);
    }
}
