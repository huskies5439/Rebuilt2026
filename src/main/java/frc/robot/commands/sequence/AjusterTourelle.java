// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Tourelle;

public class AjusterTourelle extends Command {

  private BasePilotable basePilotable;
  private Tourelle tourelle;
  
  public AjusterTourelle(BasePilotable basePilotable, Tourelle tourelle) {

    this.basePilotable = basePilotable;
    this.tourelle =tourelle;
    addRequirements(tourelle);
  }

 
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    double angle = basePilotable.getAngleCible(basePilotable.getPositionHub()).getDegrees();
    tourelle.setPID(angle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
