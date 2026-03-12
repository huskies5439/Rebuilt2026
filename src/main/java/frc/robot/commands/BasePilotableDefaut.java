// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BasePilotable;

public class BasePilotableDefaut extends Command {

  private BasePilotable basePilotable;
  private DoubleSupplier joystickVX;
  private DoubleSupplier joystickVY;
  private DoubleSupplier joystickOmega;

  private double deadband = 0.05;

  private double vx;
  private double vy;
  private double omega;

  public BasePilotableDefaut(DoubleSupplier joystickVX, DoubleSupplier joystickVY, DoubleSupplier joystickOmega, BasePilotable basePilotable) {
    this.basePilotable = basePilotable;

    this.joystickVX = joystickVX;
    this.joystickVY = joystickVY;
    this.joystickOmega = joystickOmega;

    addRequirements(basePilotable);
  }

  @Override
  public void initialize() {
       basePilotable.resetSetpoint();
  }

  @Override
  public void execute() {
    //Lecture des joysticks
    vx = joystickVX.getAsDouble();
    vy = joystickVY.getAsDouble();
    omega = joystickOmega.getAsDouble();

    // appliquer une deadband sur les joysticks et corriger la direction
    vx = -MathUtil.applyDeadband(vx, deadband);
    vy = -MathUtil.applyDeadband(vy, deadband);
    omega = -MathUtil.applyDeadband(omega, deadband);

    // Mettre les joysticks "au carré" pour adoucir les déplacements
    vx = vx * Math.abs(vx);
    vy = vy * Math.abs(vy);
    omega = omega * Math.abs(omega);

    // Convertir les valeurs des Joysticks selon les vitesses maximales du robot en téléop
    vx = vx * Constants.maxVitesseLineaire;
    vy = vy * Constants.maxVitesseLineaire;
    omega = omega * Constants.maxVitesseRotation;
    
    // inversion du field oriented selon l'alliance
    double invert = 1;
    if (Constants.isRedAlliance()) {
      invert = -1;
    }

    //Création du ChassisSpeed en field Oriented.
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              vx * invert,
              vy * invert,
              omega,
              basePilotable.getPose().getRotation());

    //Envoyer les vitesses aux modules
    basePilotable.conduireChassisSetPoint(speeds);

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
