package com.team2383.diffy.subsystems.pinkArm.pivot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Robot;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PivotSubsystem extends SubsystemBase {
    private final Ninja_CANSparkMax m_rightMotor;
    private final Ninja_CANSparkMax m_leftMotor;

    private final LinearSystem<N1, N1, N1> m_motorSim = LinearSystemId.identifyVelocitySystem(
            PivotConstants.kV,
            PivotConstants.kA);

    private final DutyCycleEncoder m_bottomAngleEncoder;
    private final DutyCycleEncoderSim m_bottomAngleEncoderSim;

    private final PIDController m_PIDController;

    private double m_voltage;

    private double m_desiredAngle;

    private double m_angle;

    private double m_velocity = 0;
    private double m_prevAngle = 0;

    private double kG = PivotConstants.kG;

    public PivotSubsystem() {
        m_leftMotor = new Ninja_CANSparkMax(PivotConstants.kBottomMotorLeftId, MotorType.kBrushless);
        m_rightMotor = new Ninja_CANSparkMax(PivotConstants.kBottomMotorRightId, MotorType.kBrushless);

        m_rightMotor.setVelocityConversionFactor(2.0 * Math.PI * 60);
        m_leftMotor.setVelocityConversionFactor(2.0 * Math.PI * 60);

        m_bottomAngleEncoder = new DutyCycleEncoder(PivotConstants.kEncoderPortAbs);
        m_bottomAngleEncoderSim = new DutyCycleEncoderSim(m_bottomAngleEncoder);

        m_leftMotor.setSmartCurrentLimit(40);
        m_rightMotor.setSmartCurrentLimit(40);

        m_PIDController = new PIDController(PivotConstants.kP, 0, PivotConstants.kD);

        m_bottomAngleEncoder.reset();
        m_bottomAngleEncoder.setPositionOffset(PivotConstants.encoderOffset);

    }

    public void periodic() {
        m_angle = getAngleRadians();
        m_velocity = (m_angle - m_prevAngle) / 0.02;
        m_prevAngle = m_angle;
        
        calculateVoltage();
    }

    @Override
    public void simulationPeriodic() {
        var newX = m_motorSim.calculateX(VecBuilder.fill(m_velocity), VecBuilder.fill(m_voltage), 0.02);

        m_leftMotor.set(newX.get(0, 0));
        m_rightMotor.set(newX.get(0, 0));

        m_bottomAngleEncoderSim
                .setDistance(m_angle / (2 * Math.PI) + (newX.get(0, 0) / (2 * Math.PI) * 0.02));

        SmartDashboard.putNumber("Simulated Left Motor Output Velocity",
                m_leftMotor.get());

        SmartDashboard.putNumber("Simulated Right Motor Output Velocity",
                m_rightMotor.get());

        SmartDashboard.putNumber("Simulated Encoder Radians", getAngleRadians());
    }

    public void setAngle(double setAngle) {
        if (m_desiredAngle > PivotConstants.kUpperBound) {
            m_desiredAngle = PivotConstants.kUpperBound;
        } else if (m_desiredAngle < PivotConstants.kLowerBound) {
            m_desiredAngle = PivotConstants.kLowerBound;
        } else {
            m_desiredAngle = setAngle;
        }
    }
    
    private void calculateVoltage() {
        m_voltage = m_PIDController.calculate(getAngleDegrees(), m_desiredAngle);
        if (Robot.isReal()) { // Am I on a planet with gravity
            m_voltage += Math.sin(m_angle) * 1 * kG;
            m_voltage += Math.signum(m_voltage) * PivotConstants.kS;
        }

        setVoltage();
    }

    public void setVelocity(double angularVelocity) {
        m_desiredAngle += angularVelocity * 0.02;
        setAngle(m_desiredAngle);
    }

    public double getAngleRadians() {
        return m_bottomAngleEncoder.get() * 2 * Math.PI;
    }

    public double getAngleDegrees() {
        return m_bottomAngleEncoder.get() * 360;
    }

    public void setVoltage() {
        m_leftMotor.setVoltage(m_voltage);
        m_rightMotor.setVoltage(m_voltage);
    }

    public boolean isAtPosition() {
        return Math.abs(m_desiredAngle - getAngleDegrees()) < 1;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pivot");
        
        builder.addDoubleProperty("Angle (Deg)",  () -> {
            return Units.radiansToDegrees(m_angle);
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
