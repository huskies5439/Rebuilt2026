// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Carroussel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kickeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Tourelle;

public class TournerCarroussel extends Command {

  Lanceur lanceur;
  Hood hood;
  Tourelle tourelle;
  BasePilotable basePilotable;
  Carroussel carroussel; 
  Kickeur kickeur; 
  boolean finish = false; 

  public TournerCarroussel(Lanceur lanceur, Hood hood, Tourelle tourelle, BasePilotable basePilotable, Carroussel carroussel, Kickeur kickeur) {
    this.lanceur = lanceur;
    this.hood = hood;
    this.tourelle = tourelle;
    this.basePilotable = basePilotable;
    this.carroussel = carroussel;
    this.kickeur = kickeur;
    addRequirements(carroussel);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    boolean condition = lanceur.atCible() && hood.atCible() && tourelle.atCible() && kickeur.atCible();
    if(condition){
      carroussel.tourner();
    }else{
      carroussel.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    carroussel.stop();
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}
