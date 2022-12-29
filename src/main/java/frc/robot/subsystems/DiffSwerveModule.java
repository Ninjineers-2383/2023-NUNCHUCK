package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.helpers.DoubleEncoder;
import frc.robot.helpers.SwerveModuleOptimizer;

public class DiffSwerveModule implements Sendable {
    private final WPI_TalonFX m_topMotor;
    private final WPI_TalonFX m_bottomMotor;

    // Simulation motor controllers
    private final TalonFXSimCollection m_topMotorSim;
    private final TalonFXSimCollection m_bottomMotorSim;

    // New fancy state space controller
    private final LinearSystem<N3, N2, N3> m_diffySwervePlant = new LinearSystem<>(
            Matrix.mat(Nat.N3(), Nat.N3()).fill(
                    // ---
                    -Constants.ModuleConstants.kv / Constants.ModuleConstants.ka, 0, 0,
                    // ---
                    0, -Constants.ModuleConstants.kv / Constants.ModuleConstants.ka, 0,
                    // ---
                    Constants.ModuleConstants.kTurnGearRatio / 2.0, Constants.ModuleConstants.kTurnGearRatio / 2.0,
                    0),
            Matrix.mat(Nat.N3(), Nat.N2()).fill(
                    // ---
                    1 / Constants.ModuleConstants.ka, 0,
                    // ---
                    0, 1 / Constants.ModuleConstants.ka,
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

    private final LinearQuadraticRegulator<N3, N2, N3> m_controller = new LinearQuadraticRegulator<>(m_diffySwervePlant,
            VecBuilder.fill(1, 1, 0.001), VecBuilder.fill(12, 12), 0.02);

    private final KalmanFilter<N3, N2, N3> m_observer = new KalmanFilter<>(Nat.N3(), Nat.N3(), m_diffySwervePlant,
            VecBuilder.fill(0.01, 0.01, 0.01), VecBuilder.fill(0.01, 0.01, 0.01), 0.02);

    private final LinearSystemLoop<N3, N2, N3> m_systemLoop = new LinearSystemLoop<>(m_diffySwervePlant, m_controller,
            m_observer, 12.0, 0.02);

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
    private Rotation2d m_staticAngle;

    // The angle angle of the module top plate (used to calculate offset)
    private Rotation2d m_moduleMountAngle;

    // Temporary storage for the current desired voltage
    private double m_topVoltage;
    private double m_bottomVoltage;

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
            int topMotorID,
            int bottomMotorID,
            int encoderPortA,
            int encoderPortB,
            int encoderPortAbs,
            Rotation2d staticAngle,
            Rotation2d moduleAngle,
            String name,
            String CANbus, DataLog log) {
        // Init all the fields
        m_topMotor = new WPI_TalonFX(topMotorID, CANbus);
        m_topMotorSim = m_topMotor.getSimCollection();

        m_bottomMotor = new WPI_TalonFX(bottomMotorID, CANbus);
        m_bottomMotorSim = m_bottomMotor.getSimCollection();

        m_encoder = new DoubleEncoder(encoderPortA, encoderPortB, encoderPortAbs);

        m_name = name;
        m_log = log;

        m_staticAngle = staticAngle;
        m_moduleMountAngle = moduleAngle;

        m_topMotor.configVoltageCompSaturation(Constants.ModuleConstants.kDriveMaxVoltage);
        m_topMotor.enableVoltageCompensation(true);
        m_bottomMotor.configVoltageCompSaturation(Constants.ModuleConstants.kDriveMaxVoltage);
        m_bottomMotor.enableVoltageCompensation(true);

        SupplyCurrentLimitConfiguration supply = new SupplyCurrentLimitConfiguration(
                true,
                Constants.ModuleConstants.kMaxCurrent,
                Constants.ModuleConstants.kMaxCurrent, 10);

        m_topMotor.configSupplyCurrentLimit(supply);
        m_bottomMotor.configSupplyCurrentLimit(supply);

        m_topMotor.setNeutralMode(NeutralMode.Coast);
        m_bottomMotor.setNeutralMode(NeutralMode.Coast);

        m_topMotorCurrent = new DoubleLogEntry(m_log, "/" + m_name + "/topMotorCurrent");
        m_bottomMotorCurrent = new DoubleLogEntry(m_log, "/" + m_name + "/bottomMotorCurrent");

        m_topMotorVel = new DoubleLogEntry(m_log, "/" + m_name + "/topMotorVel");
        m_bottomMotorVel = new DoubleLogEntry(m_log, "/" + m_name + "/bottomMotorVel");

        m_wheelSpeed = new DoubleLogEntry(m_log, "/" + m_name + "/wheelSpeed");
        m_moduleAngleLog = new DoubleLogEntry(m_log, "/" + m_name + "/moduleAngle");

        m_expectedSpeed = new DoubleLogEntry(m_log, "/" + m_name + "/expectedSpeed");
        m_expectedAngle = new DoubleLogEntry(m_log, "/" + m_name + "/expectedAngle");
    }

    /**
     * Gets the current state of the module
     * Called from periodic
     */
    public SwerveModuleState getState() {
        double topMotorSpeed = m_topMotor.getSelectedSensorVelocity();
        double bottomMotorSpeed = m_bottomMotor.getSelectedSensorVelocity();

        m_driveSpeed = getDriveSpeed(topMotorSpeed, bottomMotorSpeed);
        m_moduleAngle = getModuleAngle();

        // data logging
        m_topMotorCurrent.append(m_topMotor.getStatorCurrent());
        m_bottomMotorCurrent.append(m_bottomMotor.getStatorCurrent());

        m_topMotorVel.append(topMotorSpeed);
        m_bottomMotorVel.append(bottomMotorSpeed);

        m_wheelSpeed.append(m_driveSpeed);
        m_moduleAngleLog.append(m_moduleAngle);

        if (reset_counter < 200) {
            if (reset_counter == 199) {
                m_encoder.reset();
            }
            reset_counter++;
        }

        return new SwerveModuleState(m_driveSpeed, Rotation2d.fromDegrees(m_moduleAngle));
    }

    public void simulate() {
        m_topMotorSim.setIntegratedSensorVelocity(
                (int) ((m_systemLoop.getXHat(0) / (2 * Math.PI)) * 2048 / 10.0));

        m_bottomMotorSim.setIntegratedSensorVelocity(
                (int) ((m_systemLoop.getXHat(1) / (2 * Math.PI)) * 2048 / 10.0));

        SmartDashboard.putNumber("Simulated/" + m_name + "/Top Motor Simulator/Output Velocity",
                m_topMotor.getSelectedSensorVelocity());

        SmartDashboard.putNumber("Simulated/" + m_name + "/Bottom Motor Simulator/Output Velocity",
                m_bottomMotor.getSelectedSensorVelocity());

        m_encoder.simulate(new Rotation2d(m_systemLoop.getXHat(2)).getDegrees());

        SmartDashboard.putNumber("Simulated/" + m_name + "/Encoder/Rotation", getModuleAngle());
    }

    /**
     * Get drive speed of the module
     * 
     * @returns the speed of the module in m/s
     */
    public double getDriveSpeed(double topMotorSpeed, double bottomMotorSpeed) {
        double speed = ((topMotorSpeed - bottomMotorSpeed) / 2) /* Average sensor Velocity (raw / 100ms) */ *
                (10.0 / 2048.0) /* Motor revolutions per second */ *
                Constants.ModuleConstants.kDriveGearRatio /* Output revolutions per second */ *
                (Constants.ModuleConstants.kDriveWheelDiameterMeters * Math.PI) /*
                                                                                 * Circumference in meters
                                                                                 * (meters/second)
                                                                                 */;

        return speed;
    }

    /**
     * Gets the Motor velocity in radians per second from the drive speed in meters
     * per second
     */
    public double driveSpeedToMotorVelocity(double driveSpeed) {
        return driveSpeed *
                (1 / (Constants.ModuleConstants.kDriveWheelDiameterMeters * Math.PI)) *
                (1 / Constants.ModuleConstants.kDriveGearRatio) * /* Output revolutions per second */
                2 * Math.PI;
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

        return Math.toDegrees(MathUtil.angleModulus(Math.toRadians(angle)));
    }

    /**
     * Sets the desired state of the module
     * 
     * @param desiredState the desired state of the module
     * @return the max voltage of the motors
     */
    public double setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleOptimizer.customOptimize(desiredState,
                Rotation2d.fromDegrees(getModuleAngle()), m_staticAngle);

        m_desiredSpeed = state.speedMetersPerSecond;
        m_desiredAngle = state.angle.getDegrees();

        m_expectedSpeed.append(m_desiredSpeed);
        m_expectedAngle.append(m_desiredAngle);

        double desiredMotorVel = driveSpeedToMotorVelocity(m_desiredSpeed);

        m_systemLoop.setNextR(VecBuilder.fill(desiredMotorVel, -desiredMotorVel, Math.toRadians(m_desiredAngle)));

        m_systemLoop.correct(
                VecBuilder.fill(
                        m_topMotor.getSelectedSensorVelocity() * (20 * Math.PI / 2048),
                        m_bottomMotor.getSelectedSensorVelocity() * (20 * Math.PI
                                / 2048),
                        Math.toRadians(getModuleAngle())));

        m_systemLoop.predict(0.020);

        m_topVoltage = m_systemLoop.getU(0);

        m_bottomVoltage = m_systemLoop.getU(1);

        return Math.max(Math.abs(m_topVoltage), Math.abs(m_bottomVoltage));
    }

    /**
     * Set the voltage of the motors
     * 
     * @param driveMaxScale the max drive voltage divided by the maximum requested
     *                      voltage
     */
    public void setVoltage(double driveMaxScale) {
        m_topMotor.setVoltage(m_topVoltage);
        m_bottomMotor.setVoltage(m_bottomVoltage);
    }

    public void resetEncoders() {
        m_topMotor.setSelectedSensorPosition(0);
        m_bottomMotor.setSelectedSensorPosition(0);
        m_encoder.reset();
    }

    public void motorsOff() {
        m_topMotor.set(ControlMode.PercentOutput, 0);
        m_bottomMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setZeroOffset() {
        m_encoder.reset();
        double steerPosition = m_encoder.get() + m_moduleMountAngle.getDegrees();
        m_topMotor.config_kP(2, steerPosition, 1000);
        loadZeroOffset();
    }

    public void loadZeroOffset() {
        SlotConfiguration slot = new SlotConfiguration();
        m_topMotor.getSlotConfigs(slot, 2, 5000);
        m_offset = slot.kP - m_moduleMountAngle.getDegrees();
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
            return m_moduleAngle;
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

        builder.addDoubleProperty("Top Temperature", () -> {
            return m_topMotor.getTemperature();
        }, null);
        builder.addDoubleProperty("Bottom Temperature", () -> {
            return m_bottomMotor.getTemperature();
        }, null);
    }
}
