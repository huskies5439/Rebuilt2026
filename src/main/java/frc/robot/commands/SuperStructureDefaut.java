// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.JsonSerializable.Base;

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
    superstructure.setPoseRobot(basePilotable.getPose());
    superstructure.setCible(); 
    superstructure.setChassisSpeed(basePilotable.getChassisSpeeds());
    
  }

 
  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
