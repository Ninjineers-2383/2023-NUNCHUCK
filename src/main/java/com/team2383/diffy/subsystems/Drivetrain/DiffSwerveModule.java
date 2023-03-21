package com.team2383.diffy.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot2Configs;
import com.ctre.phoenixpro.controls.NeutralOut;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team2383.diffy.Robot;
import com.team2383.diffy.subsystems.drivetrain.DriveConstants.ModuleConstants;
import com.team2383.diffy.helpers.AngleWrapLinearSystemLoop;
import com.team2383.diffy.helpers.DoubleEncoder;
import com.team2383.diffy.helpers.SwerveModuleOptimizer;

public class DiffSwerveModule implements Sendable {
    private final TalonFX m_topMotor;
    private final TalonFX m_bottomMotor;

    // Simulation motor controllers
    private final TalonFXSimState m_topMotorSim;
    private final TalonFXSimState m_bottomMotorSim;

    private final VoltageOut m_voltageOut = new VoltageOut(0, true, false);

    // New fancy state space controller
    private final AngleWrapLinearSystemLoop<N3, N2, N3> m_systemLoop;

    // Encoder class that allows for the use of abs and quadrature encoders
    private final DoubleEncoder m_encoder;

    // Logging variables
    private final String m_name;
    private final DataLog m_log;

    // Logging variables for the module
    private final DoubleLogEntry m_topMotorCurrent;
    private final DoubleLogEntry m_bottomMotorCurrent;

    private final DoubleLogEntry m_topMotorVel;
    private final DoubleLogEntry m_bottomMotorVel;

    private final DoubleLogEntry m_wheelSpeed;
    private final DoubleLogEntry m_moduleAngleLog;

    private final DoubleLogEntry m_expectedSpeed;
    private final DoubleLogEntry m_expectedAngle;

    /**
     * Drive speed in meters per second
     */
    private double m_driveSpeed;
    /**
     * Module angle in degrees
     * <p>
     * Can be converted to radians with Rotation2d.fromDegrees()
     * <p>
     * CCW+
     */
    private double m_moduleAngle;

    // The static angle of the module
    private final Rotation2d m_staticAngle;

    // The angle angle of the module top plate (used to calculate offset)
    private final Rotation2d m_moduleMountAngle;

    // Temporary storage for the current desired voltage
    private double m_topVoltage;
    private double m_bottomVoltage;

    private final double m_kS;

    // The offset for the module encoder in degrees
    private double m_offset;

    /**
     * Desired speed in meters per second
     */
    private double m_desiredSpeed;

    /**
     * Desired angle in degrees
     * <p>
     * CCW+
     */
    private double m_desiredAngle;

    private SwerveModuleState m_desiredState;

    /**
     * Counter used for the timing of resetting the module angle
     */
    private int reset_counter = 0;

    /**
     * Creates a new DiffSwerveModule.
     * 
     * @param topMotorID     The CAN ID of the top motor
     * @param bottomMotorID  The CAN ID of the bottom motor
     * @param encoderPortA   The DIO channel of the quadrature encoder's A channel
     * @param encoderPortB   The DIO channel of the quadrature encoder's B channel
     * @param encoderPortAbs The DIO channel of the absolute encoder's PWM channel
     * @param name           The name of the module
     * @param CANbus         The CAN bus the module is on
     * @param log            The robot's DataLog
     */
    public DiffSwerveModule(
            ModuleConstants moduleConstants, String CANbus, DataLog log) {
        // Init all the fields
        m_topMotor = new TalonFX(moduleConstants.kTopMotorID, CANbus);
        m_topMotorSim = m_topMotor.getSimState();

        m_bottomMotor = new TalonFX(moduleConstants.kBottomMotorID, CANbus);
        m_bottomMotorSim = m_bottomMotor.getSimState();

        m_encoder = new DoubleEncoder(moduleConstants.kEncoderPortA, moduleConstants.kEncoderPortB,
                moduleConstants.kEncoderPortAbs);

        m_name = moduleConstants.name;
        m_log = log;

        m_staticAngle = moduleConstants.staticAngle;
        m_moduleMountAngle = moduleConstants.mountAngle;

        CurrentLimitsConfigs supply = new CurrentLimitsConfigs();
        supply.SupplyCurrentLimit = DriveConstants.kMaxCurrent;
        supply.SupplyCurrentLimitEnable = true;

        m_topMotor.getConfigurator().apply(supply);
        m_bottomMotor.getConfigurator().apply(supply);

        MotorOutputConfigs configs = new MotorOutputConfigs();
        configs.NeutralMode = NeutralModeValue.Coast;

        m_topMotor.getConfigurator().apply(configs);
        m_bottomMotor.getConfigurator().apply(configs);

        m_topMotorCurrent = new DoubleLogEntry(m_log, "/" + m_name + "/topMotorCurrent");
        m_bottomMotorCurrent = new DoubleLogEntry(m_log, "/" + m_name + "/bottomMotorCurrent");

        m_topMotorVel = new DoubleLogEntry(m_log, "/" + m_name + "/topMotorVel");
        m_bottomMotorVel = new DoubleLogEntry(m_log, "/" + m_name + "/bottomMotorVel");

        m_wheelSpeed = new DoubleLogEntry(m_log, "/" + m_name + "/wheelSpeed");
        m_moduleAngleLog = new DoubleLogEntry(m_log, "/" + m_name + "/moduleAngle");

        m_expectedSpeed = new DoubleLogEntry(m_log, "/" + m_name + "/expectedSpeed");
        m_expectedAngle = new DoubleLogEntry(m_log, "/" + m_name + "/expectedAngle");

        m_kS = moduleConstants.kS;

        // Initialize state space controller
        LinearSystem<N3, N2, N3> m_diffySwervePlant = new LinearSystem<>(
                Matrix.mat(Nat.N3(), Nat.N3()).fill(
                        // ---
                        -moduleConstants.kV / moduleConstants.kA, 0, 0,
                        // ---
                        0, -moduleConstants.kV / moduleConstants.kA, 0,
                        // ---
                        DriveConstants.kTurnGearRatio / 2.0,
                        DriveConstants.kTurnGearRatio / 2.0,
                        0),
                Matrix.mat(Nat.N3(), Nat.N2()).fill(
                        // ---
                        1 / moduleConstants.kA, 0,
                        // ---
                        0, 1 / moduleConstants.kA,
                        // ---
                        0, 0),
                Matrix.mat(Nat.N3(), Nat.N3()).fill(
                        1, 0, 0,
                        0, 1, 0,
                        0, 0, 1),
                Matrix.mat(Nat.N3(), Nat.N2()).fill(
                        0, 0,
                        0, 0,
                        0, 0));

        LinearQuadraticRegulator<N3, N2, N3> m_controller = new LinearQuadraticRegulator<>(m_diffySwervePlant,
                VecBuilder.fill(1, 1, 0.01),
                VecBuilder.fill(DriveConstants.kDriveMaxVoltage, DriveConstants.kDriveMaxVoltage), 0.02);

        KalmanFilter<N3, N2, N3> m_observer = new KalmanFilter<>(Nat.N3(), Nat.N3(), m_diffySwervePlant,
                VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.01, 0.01, 0.01), 0.02);

        m_systemLoop = new AngleWrapLinearSystemLoop<>(
                m_controller,
                new LinearPlantInversionFeedforward<>(m_diffySwervePlant, 0.02),
                m_observer,
                u -> StateSpaceUtil.desaturateInputVector(u, RobotController.getBatteryVoltage()),
                new ArrayList<Integer>(Arrays.asList(2)));
    }

    public void periodic() {
        double topMotorSpeed = m_topMotor.getRotorVelocity().getValue();
        double bottomMotorSpeed = m_bottomMotor.getRotorVelocity().getValue();

        m_driveSpeed = getDriveSpeed(topMotorSpeed, bottomMotorSpeed);
        m_moduleAngle = getModuleAngle();

        // data logging
        m_topMotorCurrent.append(m_topMotor.getStatorCurrent().getValue());
        m_bottomMotorCurrent.append(m_bottomMotor.getStatorCurrent().getValue());

        m_topMotorVel.append(topMotorSpeed);
        m_bottomMotorVel.append(bottomMotorSpeed);

        m_wheelSpeed.append(m_driveSpeed);
        m_moduleAngleLog.append(m_moduleAngle);

        if (reset_counter < 200) {
            if (reset_counter == 199) {
                m_encoder.reset();
                m_systemLoop.reset(VecBuilder.fill(0, 0, Math.toRadians(getModuleAngle())));
            }
            reset_counter++;
        }

        if (m_desiredState != null) {
            calculateVoltage();
        }
    }

    /**
     * Gets the current state of the module
     * Called from periodic
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveSpeed, Rotation2d.fromDegrees(m_moduleAngle));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDriveDistanceMeters(
                        m_topMotor.getRotorPosition().getValue(),
                        m_bottomMotor.getRotorPosition().getValue()),
                Rotation2d.fromDegrees(m_moduleAngle));
    }

    public void simulate() {

        double topMotorVel = m_systemLoop.getXHat(0) / (2 * Math.PI);
        m_topMotorSim.setRotorVelocity(topMotorVel);
        m_topMotorSim.addRotorPosition(topMotorVel * 0.02);

        double bottomMotorVel = m_systemLoop.getXHat(1) / (2 * Math.PI);
        m_bottomMotorSim.setRotorVelocity(bottomMotorVel);
        m_bottomMotorSim.addRotorPosition(bottomMotorVel * 0.02);

        SmartDashboard.putNumber("Simulated/" + m_name + "/Top Motor Simulator/Output Velocity",
                m_topMotor.getRotorVelocity().getValue());

        SmartDashboard.putNumber("Simulated/" + m_name + "/Bottom Motor Simulator/Output Velocity",
                m_bottomMotor.getRotorVelocity().getValue());

        m_encoder.simulate(new Rotation2d(m_systemLoop.getXHat(2)).getDegrees());

        SmartDashboard.putNumber("Simulated/" + m_name + "/Encoder/Rotation", getModuleAngle());
    }

    /**
     * Get drive speed of the module
     * 
     * @returns the speed of the module in m/s
     */
    public double getDriveSpeed(double topMotorSpeed, double bottomMotorSpeed) {
        double speed = (topMotorSpeed - bottomMotorSpeed) / 2 /* Average motor speed (rps) */ *
                DriveConstants.kDriveGearRatio /* Output revolutions per second */ *
                (DriveConstants.kDriveWheelDiameterMeters * Math.PI) /*
                                                                      * Circumference in meters
                                                                      * (meters/second)
                                                                      */;

        return speed;
    }

    public double getDriveDistanceMeters(double topMotorTicks, double bottomMotorTicks) {
        return ((topMotorTicks - bottomMotorTicks) / 2) *
                DriveConstants.kDriveGearRatio *
                (DriveConstants.kDriveWheelDiameterMeters * Math.PI);
    }

    /**
     * Gets the Motor velocity in radians per second from the drive speed in meters
     * per second
     */
    public double driveSpeedToMotorVelocity(double driveSpeed) {
        return driveSpeed *
                (1 / (DriveConstants.kDriveWheelDiameterMeters * Math.PI)) *
                (1 / DriveConstants.kDriveGearRatio) * /* Output revolutions per second */
                2 * Math.PI;
    }

    private double sensorVelocityToRadiansPerSecond(double sensorVelocity) {
        return sensorVelocity * 2 * Math.PI;
    }

    /**
     * Get the angle of the module
     * <p>
     * CCW+
     * 
     * @returns the angle of the module in degrees
     */
    public double getModuleAngle() {
        double angle = m_encoder.get() - m_offset;

        return angle;
    }

    /**
     * Sets the desired state of the module
     * 
     * @param desiredState the desired state of the module
     * @return the max voltage of the motors
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        m_desiredState = SwerveModuleOptimizer.customOptimize(desiredState,
                Rotation2d.fromDegrees(getModuleAngle()), m_staticAngle);

        // m_desiredState = desiredState;

        // SwerveModuleState state = desiredState;

    }

    private void calculateVoltage() {
        m_desiredSpeed = m_desiredState.speedMetersPerSecond;
        m_desiredAngle = m_desiredState.angle.getDegrees();

        m_expectedSpeed.append(m_desiredSpeed);
        m_expectedAngle.append(m_desiredAngle);

        double desiredMotorVel = driveSpeedToMotorVelocity(m_desiredSpeed);

        m_systemLoop.setNextR(VecBuilder.fill(desiredMotorVel, -desiredMotorVel, Math.toRadians(m_desiredAngle)));

        m_systemLoop.correct(
                VecBuilder.fill(
                        sensorVelocityToRadiansPerSecond(m_topMotor.getRotorVelocity().getValue()),
                        sensorVelocityToRadiansPerSecond(m_bottomMotor.getRotorVelocity().getValue()),
                        Math.toRadians(getModuleAngle())));

        m_systemLoop.predict(0.020);

        m_topVoltage = m_systemLoop.getU(0);
        if (Robot.isReal())
            m_topVoltage += Math.signum(m_topVoltage) * m_kS;

        m_bottomVoltage = m_systemLoop.getU(1);
        if (Robot.isReal())
            m_bottomVoltage += Math.signum(m_bottomVoltage) * m_kS;

        setVoltage();
    }

    /**
     * Set the voltage of the motors
     * 
     * @param driveMaxScale the max drive voltage divided by the maximum requested
     *                      voltage
     */
    private void setVoltage() {
        m_topMotor.setControl(m_voltageOut.withOutput(m_topVoltage));
        m_bottomMotor.setControl(m_voltageOut.withOutput(m_bottomVoltage));
    }

    public void resetEncoders() {
        m_topMotor.getConfigurator().setRotorPosition(0);
        m_bottomMotor.getConfigurator().setRotorPosition(0);
        m_encoder.reset();
    }

    public void motorsOff() {
        m_topMotor.setControl(new NeutralOut());
        m_bottomMotor.setControl(new NeutralOut());
    }

    public void setZeroOffset() {
        m_encoder.reset();
        double steerPosition = m_encoder.get() + m_moduleMountAngle.getDegrees();
        Slot2Configs configs = new Slot2Configs();
        configs.kP = steerPosition;
        m_topMotor.getConfigurator().apply(configs);
        loadZeroOffset();
    }

    public void loadZeroOffset() {
        Slot2Configs configs = new Slot2Configs();
        m_topMotor.getConfigurator().refresh(configs, 5000);
        m_offset = configs.kP - m_moduleMountAngle.getDegrees();
        DataLogManager.log(String.format("INFO: %s steerPosition %f\n", m_name, m_offset));

        if (Robot.isSimulation()) {
            m_offset = 0;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module");

        builder.addDoubleProperty("Desired Speed", () -> {
            return m_desiredSpeed;
        }, null);
        builder.addDoubleProperty("Drive Speed", () -> {
            return m_driveSpeed;
        }, null);

        builder.addDoubleProperty("Desired Angle (Degrees", () -> {
            return m_desiredAngle;
        }, null);

        builder.addDoubleProperty("Module Angle (Degrees)", () -> {
            return Math.toDegrees(MathUtil.angleModulus(Math.toRadians(m_moduleAngle)));
        }, null);

        builder.addDoubleProperty("Encoder Zero Offset", () -> {
            return m_encoder.getZeroOffset();
        }, null);

        builder.addDoubleProperty("Raw Quad", () -> {
            return m_encoder.getRawQuad();
        }, null);
        builder.addDoubleProperty("Raw Abs", () -> {
            return m_encoder.getRawAbs();
        }, null);

        builder.addDoubleProperty("Top Voltage", () -> {
            return m_topVoltage;
        }, null);
        builder.addDoubleProperty("Bottom Voltage", () -> {
            return m_bottomVoltage;
        }, null);

        builder.addDoubleProperty("Estimated Module Angle (x hat)", () -> {
            return Math.toDegrees(m_systemLoop.getXHat(2));
        }, null);
        builder.addDoubleProperty("Estimated Top Velocity (x hat)", () -> {
            return m_systemLoop.getXHat(0);
        }, null);
        builder.addDoubleProperty("Estimated Bottom Velocity (x hat)", () -> {
            return m_systemLoop.getXHat(1);
        }, null);

        builder.addDoubleProperty("Top Motor Velocity", () -> {
            return sensorVelocityToRadiansPerSecond(m_topMotor.getRotorVelocity().getValue());
        }, null);
        builder.addDoubleProperty("Bottom Motor Velocity", () -> {
            return sensorVelocityToRadiansPerSecond(m_bottomMotor.getRotorVelocity().getValue());
        }, null);

        builder.addDoubleProperty("Top Motor Velocity Error", () -> {
            return m_systemLoop.getError(0);
        }, null);
        builder.addDoubleProperty("Bottom Motor Velocity Error", () -> {
            return m_systemLoop.getError(1);
        }, null);
        builder.addDoubleProperty("Module Angle Error", () -> {
            return m_systemLoop.getError(2);
        }, null);
    }
}