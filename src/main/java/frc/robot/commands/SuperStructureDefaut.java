// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Superstructure;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SuperStructureDefaut extends Command {

  Superstructure superstructure; 
  BasePilotable basePilotable; 

  public SuperStructureDefaut(Superstructure superstructure, BasePilotable basePilotable) {
    this.superstructure = superstructure;
    this.basePilotable = basePilotable;

    addRequirements(superstructure);
  }

  @Override
  public void initialize() {

  }


  @Override
  public void execute() {
    //Feed Superstructure les données de la BasePilotable nécessaire au calcul du lanceur
    superstructure.setPoseRobot(basePilotable.getPose());
    superstructure.setChassisSpeed(basePilotable.getChassisSpeeds());

    //Mets à jour la cible actuelle
    superstructure.setCible();
  }

 
  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
