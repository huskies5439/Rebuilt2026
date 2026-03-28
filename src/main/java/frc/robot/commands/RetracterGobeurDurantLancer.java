// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coude;
import frc.robot.subsystems.Gobeur;

public class RetracterGobeurDurantLancer extends Command {
  Coude coude;
  Gobeur gobeur;

  double angleHaut = 90.0;

  public RetracterGobeurDurantLancer(Coude coude, Gobeur gobeur) {
    this.coude = coude;
    this.gobeur = gobeur;

    addRequirements(coude, gobeur);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (coude.getAngleDroit() <= angleHaut && coude.getAngleGauche() <= angleHaut) {
      coude.currentLimit(true);
      coude.monter();
    } else {
      coude.currentLimit(false);
      coude.hold();
    }
    gobeur.retractage();
  }

  @Override
  public void end(boolean interrupted) {
    gobeur.stop();
    coude.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
