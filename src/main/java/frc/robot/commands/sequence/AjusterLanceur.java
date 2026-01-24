// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RegimeLanceur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Lanceur;


public class AjusterLanceur extends Command {
  BasePilotable basePilotable; 
  Lanceur lanceur; 
  public AjusterLanceur(BasePilotable basePilotable, Lanceur lanceur) {
    this.basePilotable = basePilotable; 
    this.lanceur = lanceur;
    addRequirements(lanceur);
  }

 
  @Override
  public void initialize() {}

 
  @Override
  public void execute() {
    double distance = basePilotable.getDistanceHub(); 
    double vitesse = RegimeLanceur.vitesseProche; 
    if(distance > RegimeLanceur.distance){
      vitesse = RegimeLanceur.vitesseLoin; 
    }
    
    lanceur.setPID(vitesse); 
  }

  @Override
  public void end(boolean interrupted) {
    lanceur.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
