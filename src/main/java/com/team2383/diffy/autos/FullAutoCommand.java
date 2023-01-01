package com.team2383.diffy.autos;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.team2383.diffy.Constants;
import com.team2383.diffy.commands.TrajectoryDriveCommand;
import com.team2383.diffy.subsystems.DrivetrainSubsystem;

public class FullAutoCommand extends SequentialCommandGroup {
    public FullAutoCommand(DrivetrainSubsystem drivetrain, String pathName, HashMap<String, Command> commandMap) {

        // This will load the file "FullAuto.path" and generate it with a max velocity
        // of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        ArrayList<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName,
                new PathConstraints(Constants.DriveConstants.kMaxVelocity, 3));
        Pose2d initialPos = pathGroup.get(0).getInitialHolonomicPose();
        drivetrain.resetOdometry(initialPos);

        for (PathPlannerTrajectory trajectory : pathGroup) {
            addCommands(new TrajectoryDriveCommand(drivetrain, trajectory, commandMap));
        }
    }
}
