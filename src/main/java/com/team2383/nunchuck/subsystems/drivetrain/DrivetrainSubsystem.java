package com.team2383.nunchuck.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team2383.nunchuck.subsystems.drivetrain.vision.PhotonCameraWrapper;

public class DrivetrainSubsystem extends SubsystemBase {
    private final CoaxialSwerveModule m_frontLeftModule;
    private final CoaxialSwerveModule m_frontRightModule;
    private final CoaxialSwerveModule m_rearModule;

    private final CoaxialSwerveModule[] m_modules;
    private final SwerveModuleState[] m_lastStates;

    private final PhotonCameraWrapper m_camera = new PhotonCameraWrapper();

    private final GyroIO m_gyro;
    private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();

    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            DriveConstants.frontLeftConstants.translation,
            DriveConstants.frontRightConstants.translation,
            DriveConstants.rearConstants.translation);

    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final Field2d m_field = new Field2d();
    private final FieldObject2d m_COR;

    public DrivetrainSubsystem(GyroIO gyro, SwerveModuleIO frontLeftIO, SwerveModuleIO frontRightIO,
            SwerveModuleIO rearIO) {
        m_gyro = gyro;

        m_frontLeftModule = new CoaxialSwerveModule(frontLeftIO, "FL");
        m_frontRightModule = new CoaxialSwerveModule(frontRightIO, "FR");
        m_rearModule = new CoaxialSwerveModule(rearIO, "R");

        m_modules = new CoaxialSwerveModule[] { m_frontLeftModule, m_frontRightModule, m_rearModule };

        m_poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                new Rotation2d(),
                getModulePositions(),
                new Pose2d());

        m_lastStates = new SwerveModuleState[m_modules.length];

        SmartDashboard.putData("Field", m_field);
        m_COR = m_field.getObject("COR");

        addChild(DriveConstants.frontLeftConstants.name, m_frontLeftModule);
        addChild(DriveConstants.frontRightConstants.name, m_frontRightModule);
        addChild(DriveConstants.rearConstants.name, m_rearModule);

        if (RobotBase.isSimulation()) {
            m_poseEstimator.resetPosition(new Rotation2d(), getModulePositions(),
                    new Pose2d(new Translation2d(0, 0), new Rotation2d()));
        }

        resetHeading();

    }

    @Override
    public void periodic() {
        m_gyro.updateInputs(m_gyroInputs);
        Logger.getInstance().processInputs("Gyro", m_gyroInputs);

        for (CoaxialSwerveModule module : m_modules) {
            module.periodic();
        }

        for (int i = 0; i < m_modules.length; i++) {
            m_lastStates[i] = m_modules[i].getState();
        }

        ChassisSpeeds chassis = m_kinematics.toChassisSpeeds(m_lastStates);

        if (m_gyroInputs.connected) {
            m_poseEstimator.update(Rotation2d.fromDegrees(m_gyroInputs.headingDeg), getModulePositions());
        } else {
            m_poseEstimator.update(
                    Rotation2d.fromRadians(getHeading().getRadians() + chassis.omegaRadiansPerSecond * 0.02),
                    getModulePositions());
        }

        EstimatedRobotPose cam_pose = m_camera.getEstimatedGlobalPose(getPose());

        if (cam_pose != null) {
            var estimate = cam_pose.estimatedPose.toPose2d();
            if (estimate.getX() > 0 && estimate.getY() > 0 && estimate.getX() < 17 &&
                    estimate.getY() < 9) {
                m_poseEstimator.addVisionMeasurement(cam_pose.estimatedPose.toPose2d(),
                        MathSharedStore.getTimestamp(),
                        VecBuilder.fill(1, 1, 1));
            }
        }

        Pose2d estimatedPose = m_poseEstimator.getEstimatedPosition();

        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            Translation2d transformedTranslation = new Translation2d(16.46 - estimatedPose.getX(),
                    8.02 - estimatedPose.getY());
            Rotation2d transformedHeading = estimatedPose.getRotation().plus(Rotation2d.fromDegrees(180));

            estimatedPose = new Pose2d(transformedTranslation, transformedHeading);
        }

        m_field.setRobotPose(estimatedPose);

        SmartDashboard.putNumber("Roll", getRoll());
    }

    @Override
    public void simulationPeriodic() {
        // m_gyroSim.setRawYaw(
        // m_gyro.getYaw().getValue() + (m_lastChassisSpeed.omegaRadiansPerSecond * 180
        // / Math.PI) * 0.02);
    }

    /**
     * Drive the robot using field or robot relative velocity
     * 
     * @param drive
     *            The speed for driving
     * @param angle
     *            The set angular speed
     * @param fieldRelative
     *            Whether the speeds are relative to the field or the
     *            robot
     * @param centerOfRotation
     *            Its the center of rotation duh
     */
    public void drive(Translation2d drive, Rotation2d angle, boolean fieldRelative,
            Translation2d centerOfRotation) {
        ChassisSpeeds speeds;

        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(drive.getX(), drive.getY(), angle.getRadians(),
                    getHeading());
        } else {
            speeds = new ChassisSpeeds(drive.getX(), drive.getY(), angle.getRadians());
        }

        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                speeds,
                centerOfRotation);

        setModuleStates(swerveModuleStates);

        m_COR.setPose(getPose().plus(new Transform2d(centerOfRotation, new Rotation2d())));
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeed);

        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setDesiredState(states[i]);
        }
    }

    public void resetEncoders() {
        // for (CoaxialSwerveModule module : m_modules) {
        // module.resetEncoders();
        // }
    }

    /**
     * Get the current robot heading
     * Note: This is normalized so that 0 is facing directly away from your alliance
     * wall
     * <p>
     * Note: CCW is positive
     * 
     * @return The heading of the robot in a Rotation2D
     */
    public Rotation2d getHeading() {
        return m_poseEstimator.getEstimatedPosition().getRotation();
    }

    /**
     * Reset the heading to the param
     * 
     * @param currentHeading
     *            the heading to reset the gyro to. This must be in field
     *            relative coordinates when CCW is position and 0 is
     *            facing directly towards the opposing alliance wall
     */
    public void forceHeading(Rotation2d currentHeading) {
        m_gyro.setHeading(currentHeading);
        m_poseEstimator.resetPosition(currentHeading, getModulePositions(), m_poseEstimator.getEstimatedPosition());
    }

    /**
     * Set the current heading to the calculated compass heading
     * <p>
     * Gyro offset needs to be saved to robot before this can be used.
     * If the compass heading is not stored this will set the forward direction to
     * north
     */
    public void resetHeading() {
        m_gyro.setHeading(Rotation2d.fromDegrees(getCompassHeading()));
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(),
                new Pose2d(m_poseEstimator.getEstimatedPosition().getTranslation(), getHeading()));
        resetEncoders();
    }

    /**
     * Get turn rate of robot
     * 
     * @return turn rate in degrees per second CCW positive
     */
    public double getTurnRate() {
        return m_gyroInputs.headingRateDPS;
    }

    public double getRoll() {
        return m_gyroInputs.rollDeg;
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                m_rearModule.getPosition() };
    }

    public void forceOdometry(Pose2d pose) {
        forceHeading(pose.getRotation());
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
        resetEncoders();
    }

    public void setPosition(Translation2d position) {
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(), new Pose2d(position, getHeading()));
        resetEncoders();
    }

    public double getCompassHeading() {
        // Compass is CW positive not CCW positive
        // double fieldCompassHeading = Preferences.getDouble("Compass", 0);
        // double currentCompassHeading = m_gyro.getComas().getValue();

        // return currentCompassHeading - fieldCompassHeading;
        return 0;
    }

    public void setCompassOffset() {
        // Compass is CW positive not CCW positive
        // double fieldCompassHeading = m_gyro.getCompassHeading();
        // Preferences.setDouble("Compass", fieldCompassHeading);

        // DataLogManager.log("INFO: Compass offset set to " + fieldCompassHeading +
        // "\n");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}