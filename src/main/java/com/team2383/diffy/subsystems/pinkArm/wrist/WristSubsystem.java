package com.team2383.diffy.subsystems.pinkArm.wrist;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Constants;
import com.team2383.diffy.Robot;
import com.team2383.diffy.Constants.WristConstants;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
    private final Ninja_CANSparkMax m_pivotMotor;

    private final LinearSystem<N1, N1, N1> m_motorSim = LinearSystemId
            .identifyVelocitySystem(Constants.WristConstants.kV, Constants.WristConstants.kA);

    private final DutyCycleEncoder m_topAngleEncoder;
    private final DutyCycleEncoderSim m_topAngleEncoderSim;

    private final SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(Constants.WristConstants.kS,
            Constants.WristConstants.kV, Constants.WristConstants.kA);
    private final PIDController m_fb = new PIDController(Constants.WristConstants.kP, 0, 0);

    private double m_voltage;

    // In radians
    private double m_desiredAngle;
    private double m_bottomAngle;
    private double m_angle;
    private double m_prevAngle = 0;

    // In radians per second
    private double m_currentVelocity = 0;

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1);

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    private TrapezoidProfile.State state = new TrapezoidProfile.State();

    double m_simVelocity = 0;

    public WristSubsystem() {
        m_pivotMotor = new Ninja_CANSparkMax(WristConstants.kMotorID, MotorType.kBrushless);

        m_pivotMotor.setVelocityConversionFactor(2.0 * Math.PI * 60);

        m_topAngleEncoder = new DutyCycleEncoder(Constants.WristConstants.kEncoderPortAbs);

        m_topAngleEncoderSim = new DutyCycleEncoderSim(m_topAngleEncoder);
    }

    public void periodic() {
        m_angle = getAngleRadians();

        m_currentVelocity = (m_angle - m_prevAngle) / 0.02;
        m_prevAngle = m_angle;
    }

    public void simulate() {
        var newVel = m_motorSim.calculateX(VecBuilder.fill(m_currentVelocity), VecBuilder.fill(m_voltage), 0.02).get(0,
                0);

        m_topAngleEncoderSim.setDistance(m_angle / (2 * Math.PI) + (newVel / (2 * Math.PI) * 0.02));

        SmartDashboard.putNumber("Simulated Top Pivot Motor Output Velocity",
                m_pivotMotor.get());

        SmartDashboard.putNumber("Simulated Encoder Radians", getAngleRadians());
    }

    public void setAngle(double desiredAngle, double bottomAngle) {
        if (m_desiredAngle > WristConstants.kUpperBound) {
            m_desiredAngle = WristConstants.kUpperBound;
        } else if (m_desiredAngle < WristConstants.kLowerBound) {
            m_desiredAngle = WristConstants.kLowerBound;
        } else {
            m_desiredAngle = desiredAngle;
        }

        goal = new TrapezoidProfile.State(
                m_desiredAngle,
                0);

        state = new TrapezoidProfile(constraints, goal, state).calculate(0.02);

        m_voltage = m_ff.calculate(state.velocity)
                + m_fb.calculate(m_angle, state.position);

        if (Robot.isReal()) {
            m_voltage += Math.sin(m_angle + m_bottomAngle) * Constants.WristConstants.kG;
        }

        setVoltage();

        
    }

    public void setVelocity(double desiredSpeed, double bottomAngle) {
        m_desiredAngle += desiredSpeed * 0.02;
        setAngle(m_desiredAngle, bottomAngle);
    }

    public double getAngleRadians() {
        return (-m_topAngleEncoder.get() + 0.53) * 2 * Math.PI;
    }

    public double getAngleDegrees() {
        return (-m_topAngleEncoder.get() + 0.53) * 360;
    }

    public void setVoltage() {
        m_pivotMotor.setVoltage(m_voltage);
    }
}
