// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RegimeLanceur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Hood;

public class AjusterHood extends Command {

  
  
  private BasePilotable basePilotable;
  private Hood hood;

  
  public AjusterHood(BasePilotable basePilotable, Hood hood) {
    this.basePilotable = basePilotable;
    this.hood = hood;
    addRequirements(hood);
}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double distance = basePilotable.getDistanceHub(); 
    double angleHood = distance*RegimeLanceur.facteurAngleProche; 
    if(distance > RegimeLanceur.distance){ 
      angleHood = distance*RegimeLanceur.facteurAngleLoin; 
    }
    hood.setPID(angleHood);
  }

  @Override
  public void end(boolean interrupted) {
    hood.stop(); 
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
