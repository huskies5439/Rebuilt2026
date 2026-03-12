// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.fancyPath;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Superstructure;

public class DeplacementPID extends Command {
  BasePilotable basePilotable;
  Superstructure superstructure; //juste pour le isProche()
  Pose2d cible;

  public DeplacementPID(Pose2d cible, BasePilotable basePilotable, Superstructure superstructure) {
    this.basePilotable = basePilotable;
    this.superstructure = superstructure;
    this.cible = cible;

    addRequirements(basePilotable);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    basePilotable.setPID(cible);
  }


  @Override
  public void end(boolean interrupted) {
    basePilotable.setX();
  }

  @Override
  public boolean isFinished() {
    return superstructure.isProche(cible, 0.05) && isAngleProche(5);
  }

   private boolean isAngleProche(double tolerance) {//Il y a probablement une façon plus clean de faire en soustrayant les Pose2D
    return Math.abs(basePilotable.getPose().getRotation().getDegrees()-cible.getRotation().getDegrees()) <= tolerance;
  }
}
