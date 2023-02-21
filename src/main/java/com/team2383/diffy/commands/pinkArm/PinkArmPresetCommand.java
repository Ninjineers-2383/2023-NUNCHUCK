package com.team2383.diffy.commands.pinkArm;

import com.team2383.diffy.commands.pinkArm.position.PivotPositionCommand;
import com.team2383.diffy.commands.pinkArm.position.TelescopePositionCommand;
import com.team2383.diffy.commands.pinkArm.position.WristPositionCommand;
import com.team2383.diffy.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.diffy.subsystems.pinkArm.wrist.WristSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PinkArmPresetCommand extends SequentialCommandGroup {
    public final PivotSubsystem m_pivot;
    public final TelescopeSubsystem m_telescope;
    public final WristSubsystem m_wrist;

    public double m_pivotAngle;
    public double m_telescopeExtension;
    public double m_wristAngle;

    public PinkArmPresetCommand(PivotSubsystem pivot, TelescopeSubsystem telescope, WristSubsystem wrist, 
        Rotation2d desiredPivotAngle, double desiredTelescopeExtension, Rotation2d desiredWristAngle) {
        m_pivot = pivot;
        m_telescope = telescope;
        m_wrist = wrist;

        m_pivotAngle = pivot.getAngle().getRadians();
        m_telescopeExtension = telescope.getExtensionInches();
        m_wristAngle = wrist.getAngle().getRadians();
        if (Math.signum(m_pivotAngle) != Math.signum(desiredPivotAngle.getRadians())) {
            addCommands(new ParallelCommandGroup(new TelescopePositionCommand(telescope, 0),
                                                 new WristPositionCommand(wrist, Rotation2d.fromDegrees(0))),
                        new PivotPositionCommand(pivot, desiredPivotAngle),
                        new ParallelCommandGroup(new TelescopePositionCommand(telescope, desiredTelescopeExtension),
                                                 new WristPositionCommand(wrist, desiredWristAngle))
                        );
        } 
        
        // else {
        //     addCommands(new ParallelCommandGroup(new TelescopePositionCommand(telescope, desiredTelescopeExtension),
        //                                          new WristPositionCommand(wrist, desiredWristAngle),
        //                                          new PivotPositionCommand(pivot, desiredPivotAngle)));
        // }

    }

}
