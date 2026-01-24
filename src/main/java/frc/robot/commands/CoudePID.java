// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coude;

public class CoudePID extends Command {
 
  double angleCible;
  Coude coude;

  public CoudePID(double angleCible, Coude coude) {

    this.angleCible = angleCible;
    this.coude = coude;
    addRequirements(coude);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coude.resetPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coude.setPID(angleCible);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coude.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coude.atCible();
  }
}
