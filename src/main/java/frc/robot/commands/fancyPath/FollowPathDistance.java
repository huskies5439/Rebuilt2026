// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.fancyPath;

import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.FancyPathGeneration;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Superstructure;


public class FollowPathDistance extends ParallelRaceGroup {

  //Superstructure est là pour le isProche
  public FollowPathDistance(Pose2d cible, BasePilotable basePilotable, Superstructure superstructure, FancyPathGeneration fancyPath) {
    addRequirements(basePilotable);
    addCommands(
      Commands.defer(
        ()-> {
          basePilotable.resetPID();
          PathPlannerPath path = fancyPath.genererPath(cible);
          return basePilotable.followPath(path);
        },
        Set.of()),//Je suspecte qu'il faudra changer ça pour Set.of(basePilotable)

        new WaitUntilCommand(()-> superstructure.isProche(cible, 1.5))
    );
  }

   /*
     * Les méthodes sont effectuées au démarrage du robot, le defer fait en sorte qu'il appèle la fonction quand il en a besoin.
     * Dans followPath, nous utilisons la pose actuelle du robot pour faire le path (sinon restait à 0,0).
     */
}
