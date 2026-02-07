// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Tourelle;

public class ViserTourelle extends Command {

  BasePilotable basePilotable;
  Tourelle tourelle;
  double deltaAngleAbsolu;
  double deltaAngleCourt;

  double angleActuelAbsolu;
  double angleCibleAbsolu;

  double angleCibleReel;
  
  double limiteFil = 180; // à vérifier

  public ViserTourelle(Tourelle tourelle) {
    this.tourelle = tourelle;
    this.basePilotable = basePilotable;

    addRequirements(tourelle);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    getInitialInfos();
  }

  private void getInitialInfos() {
    angleCibleAbsolu = basePilotable.getAngleCible(basePilotable.getPositionHub()).getDegrees();
    angleActuelAbsolu = tourelle.getAngleAbsolu();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getInitialInfos();

    deltaAngleAbsolu = angleCibleAbsolu - angleActuelAbsolu;
    if (Math.abs(deltaAngleAbsolu) > 180.0) {

      deltaAngleCourt = trouverDeltaAngleCourt();
      angleCibleReel = tourelle.getAngleReel() + deltaAngleCourt;

      if (Math.abs(angleCibleReel) < limiteFil) {
        tourelle.setPID(angleCibleReel);
      } else {
        demi_tour();
      }
    } else {
      demi_tour();
    }

  }

  private void demi_tour() {
    angleCibleReel = angleActuelAbsolu + deltaAngleAbsolu;

    if (!(Math.abs(angleCibleReel) < limiteFil)) {
      angleCibleReel = (Math.abs(angleCibleReel) - 360) * Math.signum(angleCibleReel);
    }

    tourelle.setPID(angleCibleReel);
  }

  private double trouverDeltaAngleCourt() {
    return (Math.abs(deltaAngleAbsolu) - 360.0) * (Math.signum(deltaAngleAbsolu));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
