package com.team2383.diffy.subsystems.pinkArm.pivot;


import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Robot;
import com.team2383.diffy.helpers.Clip;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;
import com.team2383.diffy.helpers.AngularVelocityWrapper;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;


public class PivotSubsystem extends TrapezoidProfileSubsystem {
    private final Ninja_CANSparkMax m_rightMotor;
    private final Ninja_CANSparkMax m_leftMotor;

    private final LinearSystem<N1, N1, N1> m_motorSim = LinearSystemId.identifyVelocitySystem(
            PivotConstants.FEEDFORWARD_CONTROLLER.kv,
            PivotConstants.FEEDFORWARD_CONTROLLER.ka);

    private final DutyCycleEncoder m_absEncoder;
    private final DutyCycleEncoderSim m_absEncoderSim;

    private double m_voltage;
    private Rotation2d m_desiredAngle;
    private final AngularVelocityWrapper m_velocity;

    private DoubleSupplier m_extensionSupplier;

    /**
     * Pivot Subsystem Constructor (The pivot is the big swinging joint on the pink arm)
     * @param extension for safety; checks extension to make sure it's not too long
     */
    public PivotSubsystem(DoubleSupplier extension) {
        super(PivotConstants.TRAPEZOIDAL_CONSTRAINTS, 0);
        m_leftMotor = new Ninja_CANSparkMax(PivotConstants.BOTTOM_MOTOR_LEFT_ID, MotorType.kBrushless);
        m_rightMotor = new Ninja_CANSparkMax(PivotConstants.BOTTOM_MOTOR_RIGHT_ID, MotorType.kBrushless);

        m_rightMotor.setVelocityConversionFactor(PivotConstants.VELOCITY_CONVERSION_FACTOR.getRadians());
        m_leftMotor.setVelocityConversionFactor(PivotConstants.VELOCITY_CONVERSION_FACTOR.getRadians());

        m_absEncoder = new DutyCycleEncoder(PivotConstants.ABS_ENCODER_PORT);
        m_absEncoderSim = new DutyCycleEncoderSim(m_absEncoder);

        m_leftMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);
        m_rightMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);

        m_absEncoder.setPositionOffset(PivotConstants.ENCODER_OFFSET);

        m_velocity = new AngularVelocityWrapper(getAngle());
    }

    public void periodic() {
        Rotation2d angle = getAngle();
        m_velocity.calculate(angle);
    }

    @Override
    public void simulationPeriodic() {
        var newX = m_motorSim.calculateX(VecBuilder.fill(m_velocity.get().getRadians()), VecBuilder.fill(m_voltage), 0.02);

        m_leftMotor.setSimVelocity(newX.get(0, 0));
        m_rightMotor.setSimVelocity(newX.get(0, 0));

        m_absEncoderSim
                .setDistance(getAngle().getRotations() + (newX.get(0, 0) / (2 * Math.PI) * 0.02));
    }

    /**
     * updates 
     * @param state desired state of the pivot
     */
    @Override
    public void useState(TrapezoidProfile.State state) {
        setVelocity(Rotation2d.fromRadians(state.velocity));
    }

    /**
     * Uses trapezoidal motion-profiling to implement pseudo-positional control
     * @param angle
     * @return boolean state to determine whether the input angle is safe
     */
    public boolean setGoal(Rotation2d angle) {
        // safety for upper bounds
        double adjustedAngle = Clip.clip(PivotConstants.LOWER_BOUND.getRadians(), angle.getRadians(), PivotConstants.UPPER_BOUND.getRadians());

        // safety for inside robot
        adjustedAngle = m_extensionSupplier.getAsDouble() < PivotConstants.EXTENSION_SAFETY ? adjustedAngle : 
            Clip.invClip(PivotConstants.LOWER_SAFETY.getRadians(), adjustedAngle, PivotConstants.UPPER_SAFETY.getRadians());

        m_desiredAngle = angle;
        super.setGoal(new TrapezoidProfile.State(adjustedAngle, 0));
        return adjustedAngle != angle.getRadians();
    }

    /**
     * Overrided goal method (bad practice to use because it uses double for angle)
     * @param angle in radians
     */
    @Override
    public void setGoal(double angle) {
        setGoal(Rotation2d.fromRadians(angle));
    }

    /**
     * Set velocity of the pivot using PID and feedforward control
     * If used externally, call disable() before using this method
     * Make sure to call enable() to resume positional control
     * @param desiredVelocity in radians per second
     */
    public void setVelocity(Rotation2d desiredVelocity) {
        m_voltage = PivotConstants.PID_CONTROLLER.calculate(m_velocity.get().getRadians(), desiredVelocity.getRadians());
        m_voltage += PivotConstants.FEEDFORWARD_CONTROLLER.calculate(m_velocity.get().getRadians(), desiredVelocity.getRadians());
        if (Robot.isReal()) { // Am I on a planet with gravity
            m_voltage += Math.sin(getAngle().getRadians()) * 1 * PivotConstants.kG;
        }

        m_leftMotor.setVoltage(m_voltage);
        m_rightMotor.setVoltage(m_voltage);
    }

    /**
     * Gets velocity of the abs encoder
     * @return
     */
    public Rotation2d getVelocity() {
        return m_velocity.get();
    }

    /** 
     * Returns current angle of offset abs encoder
     * @return angle in Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_absEncoder.get());
    }

    /** 
     * Returns target arm angle
     * @return angle in Rotation2d
     */
    public Rotation2d getDesiredAngle() {
        return m_desiredAngle != null ? m_desiredAngle : getAngle();
    }

    /**
     * Gets whether the arm has reached target point
     * @return boolean state
     */
    public boolean isAtPosition() {
        return m_desiredAngle == null || m_desiredAngle.getRadians() - getAngle().getRadians() < PivotConstants.POSITION_TOLERANCE.getRadians();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pivot");
        
        builder.addDoubleProperty("Angle (Deg)",  () -> {
            return getAngle().getDegrees();
        }, null);

        builder.addDoubleProperty("Desired Angle (Deg)", () -> {
            return getDesiredAngle().getDegrees();
        }, null);

        builder.addDoubleProperty("Velocity (Deg per sec)", () -> {
            return m_velocity.get().getDegrees();
        }, null);

        builder.addDoubleProperty("Voltage (Volts)", () -> {
            return m_voltage;
        }, null);
    }

}
