// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Carroussel;
import frc.robot.subsystems.Kickeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;

public class PostLancer extends SequentialCommandGroup {

  public PostLancer(Lanceur lanceur, Carroussel carroussel, Kickeur kickeur, Superstructure superstructure) {

    addCommands(
        Commands.runOnce(()-> superstructure.setLancerActif(false)), 
        Commands.runOnce(carroussel::stop, carroussel),
        new WaitCommand(0.5),
        Commands.runOnce(lanceur::stop, lanceur).alongWith(Commands.runOnce(kickeur::stop, kickeur))
        

    );
  }
}
