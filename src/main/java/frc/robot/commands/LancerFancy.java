// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Carroussel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kickeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Tourelle;

public class LancerFancy extends ParallelCommandGroup {

  public LancerFancy(BasePilotable basePilotable, Lanceur lanceur, Hood hood, Tourelle tourelle, Kickeur kickeur, Carroussel carroussel, Superstructure superstructure) {
 
    addCommands(
     lanceur.lancerPIDCommand(superstructure.normeVecteurLancer(basePilotable.getPose())/(Units.inchesToMeters(4) * 1)), //1 étant le facteur de friction)
      hood.goToAnglePIDCommand(
      superstructure.pitchVecteurLancer(basePilotable.getPose())),
      new ViserTourelle(tourelle,basePilotable,superstructure), 
      kickeur.tournerCommand(),
      new TournerCarroussel()
      
    );
  }
}
