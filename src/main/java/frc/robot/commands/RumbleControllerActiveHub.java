// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;

public class RumbleControllerActiveHub extends Command {

  // Avant de descendre plus loin, veuillez noter que ce code est extrêmement beau!
  // Vos yeux vont vous remercier!
  // RIP code laid 2026/03/27 à 2026/03/28 :(

  CommandXboxController manette;
  boolean lastActive = false;
  boolean isStart;

  long startTime;

  public RumbleControllerActiveHub(boolean isStart, CommandXboxController manette, Superstructure superstructure) {
    this.manette = manette;
    this.isStart = isStart;

  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    manette.setRumble(isStart ? RumbleType.kLeftRumble : RumbleType.kRightRumble, 1.0);
  }

  @Override
  public void end(boolean interrupted) {
    manette.setRumble(isStart ? RumbleType.kLeftRumble : RumbleType.kRightRumble, 0.0);
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime >= Constants.VIBRATION_TIME * 1000;
  }

}
