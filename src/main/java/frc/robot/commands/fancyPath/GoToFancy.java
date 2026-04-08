// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.fancyPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.FancyPathGeneration;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Superstructure;


public class GoToFancy extends SequentialCommandGroup {

    public GoToFancy(
        Pose2d cible,
        BasePilotable basePilotable,
        Superstructure superstructure,
        FancyPathGeneration fancyPath
    ) {

        addCommands(
            new FollowPathDistance(cible, basePilotable, superstructure, fancyPath),
            new DeplacementPID(cible, basePilotable, superstructure));
    }
}
