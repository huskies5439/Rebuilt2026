// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coude;
import frc.robot.subsystems.Gobeur;

public class RetracterGobeurDurantLancer extends Command {
  Coude coude;
  Gobeur gobeur;

  double angleHaut = 45.0;

  ProfiledPIDController pid = new ProfiledPIDController(0.1, 0, 0,
      new TrapezoidProfile.Constraints(15, 150)); 

  public RetracterGobeurDurantLancer(Coude coude, Gobeur gobeur) {
    this.coude = coude;
    this.gobeur = gobeur;

    addRequirements(coude, gobeur);
  }

  @Override
  public void initialize() {
    // coude.currentLimit(true);
    pid.reset(coude.getAngleDroit());
    
  }

  @Override
  public void execute() {
    // if (coude.getAngleDroit() <= angleHaut && coude.getAngleGauche() <= angleHaut) {
    //   gobeur.retractage();
    //   coude.monter();
    // } else {
    //   coude.stop();
    // }
    double voltage = pid.calculate(coude.getAngleDroit(),angleHaut); 
    coude.setVoltage(voltage);
    gobeur.retractage();
  }

  @Override
  public void end(boolean interrupted) {
    // coude.currentLimit(false);
    gobeur.stop();
    coude.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
