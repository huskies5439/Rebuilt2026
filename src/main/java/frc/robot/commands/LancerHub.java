// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Carroussel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kickeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Tourelle;

public class LancerHub extends ParallelCommandGroup {

  public LancerHub(BasePilotable basePilotable, Lanceur lanceur, Hood hood, Tourelle tourelle, Kickeur kickeur, Carroussel carroussel) {

 
    addCommands(
      lanceur.lancerPIDCommand(lanceur.heuristic(basePilotable.getDistanceHub())),
      hood.goToAnglePIDCommand(
       hood.calculAngleHood(lanceur.getVitesse(), Constants.hauteurHub, basePilotable.getDistanceHub())),
      //tourelle.
      kickeur.tournerCommand(),
      new TournerCarroussel()
      
    );
  }
}
