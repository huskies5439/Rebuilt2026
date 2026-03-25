// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.ShotParams;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kickeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PreparerLancer extends Command {

  ShotParams shotParams;

  boolean dynamique;

  Superstructure superstructure;
  Kickeur kickeur;
  Lanceur lanceur;
  Hood hood;

  public PreparerLancer(Superstructure superstructure, Kickeur kickeur, Lanceur lanceur, Hood hood) {
    this.superstructure = superstructure;
    this.kickeur = kickeur;
    this.lanceur = lanceur;
    this.hood = hood;

    addRequirements(lanceur, hood, kickeur, superstructure);

  }

  @Override
  public void initialize() {
    dynamique = false;
  }

  @Override
  public void execute() {
    shotParams = superstructure.getShotParams(dynamique);

    lanceur.setPID(30);
    kickeur.setPID(20);

    hood.setPID(Constants.kAngleHoodDepart);

  }

  @Override
  public void end(boolean interrupted) {
    hood.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
