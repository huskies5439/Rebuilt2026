// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coude;
import frc.robot.subsystems.Gobeur;

public class RetracterShooting extends Command {
  Coude coude; 
  Gobeur gobeur; 

  public RetracterShooting(Coude coude, Gobeur gobeur) {
    this.coude = coude; 
    this.gobeur = gobeur; 

    addRequirements(coude, gobeur);
  }

 
  @Override
  public void initialize() {
    coude.currentLimit(true);
  }

  
  @Override
  public void execute() {
    if(coude.getAngleDroit() <= Constants.kAngleCoudeDepart-10 && coude.getAngleGauche() <= Constants.kAngleCoudeDepart-10){
      coude.monter();
    }else{
      coude.stop();
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
