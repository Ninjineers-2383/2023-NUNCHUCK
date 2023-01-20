package com.team2383.diffy.autos;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.team2383.diffy.Constants;
import com.team2383.diffy.subsystems.DrivetrainSubsystem;

public class TwoConeAuto extends SequentialCommandGroup {
    public TwoConeAuto(DrivetrainSubsystem drivetrain, String pathName, SwerveAutoBuilder autoBuilder) {

        // This will load the file "FullAuto.path" and generate it with a max velocity
        // of 3 m/s and a max acceleration of 2 m/s^2
        // for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName,
                new PathConstraints(Constants.DriveConstants.kMaxVelocity - 1, 2));

        addCommands(autoBuilder.fullAuto(pathGroup));
    }
}
