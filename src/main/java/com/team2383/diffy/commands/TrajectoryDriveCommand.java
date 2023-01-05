package com.team2383.diffy.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.team2383.diffy.Constants;
import com.team2383.diffy.subsystems.DrivetrainSubsystem;

public class TrajectoryDriveCommand extends CommandBase {
    private final HolonomicDriveController driveController = new HolonomicDriveController(
            new PIDController(5, 0, 0),
            new PIDController(5, 0, 0),
            new ProfiledPIDController(4, 0, 0,
                    new Constraints(Constants.DriveConstants.kMaxAngularVelocity, 2)));

    private final PathPlannerTrajectory trajectory;

    private final DrivetrainSubsystem drivetrain;

    private final List<EventMarker> markers;

    private final HashMap<String, Command> commandMap;

    private final Timer timer = new Timer();

    private int markerIndex = 0;
    private boolean atEnd = false;

    public TrajectoryDriveCommand(
            DrivetrainSubsystem drivetrain, PathPlannerTrajectory trajectory, HashMap<String, Command> commandMap) {
        this.trajectory = trajectory;

        this.driveController.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(0.2)));

        this.drivetrain = drivetrain;

        this.markers = trajectory.getMarkers();

        this.commandMap = commandMap;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        PathPlannerState state = (PathPlannerState) trajectory.sample(timer.get());
        ChassisSpeeds speeds = driveController.calculate(drivetrain.getPose(), state, state.holonomicRotation);

        drivetrain.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond,
                false,
                new Translation2d());

        if (markers.size() > markerIndex && markers.get(markerIndex).timeSeconds > timer.get()) {
            CommandScheduler.getInstance().schedule(commandMap.get(markers.get(markerIndex).name));
            markerIndex++;
        }

        atEnd = state == trajectory.getEndState();
    }

    @Override
    public boolean isFinished() {
        return atEnd && driveController.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.motorsOff();
    }

}
