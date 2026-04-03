// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Superstructure;

public class BasePilotableDefaut extends Command {

  private BasePilotable basePilotable;
  private DoubleSupplier joystickVX;
  private DoubleSupplier joystickVY;
  private DoubleSupplier joystickOmega;

  private Superstructure superstructure;

  private double deadband = 0.05;

  private double vx;
  private double vy;
  private double omega;

  private double maxVitesseLineaire;
  private double maxVitesseRotation;

  public BasePilotableDefaut(DoubleSupplier joystickVX, DoubleSupplier joystickVY, DoubleSupplier joystickOmega,
      BasePilotable basePilotable, Superstructure superstructure) {
    this.basePilotable = basePilotable;
    this.superstructure = superstructure;

    this.joystickVX = joystickVX;
    this.joystickVY = joystickVY;
    this.joystickOmega = joystickOmega;

    addRequirements(basePilotable);
  }

  @Override
  public void initialize() {
    basePilotable.resetSetpoint();

    maxVitesseLineaire = Constants.maxVitesseLineaire;
    maxVitesseRotation = Constants.maxVitesseRotation;
  }

  @Override
  public void execute() {

    SmartDashboard.putBoolean("RED ALLIANCE ?", Constants.isRedAlliance());
    // Lecture des joysticks
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

    // Convertir les valeurs des Joysticks selon les vitesses maximales du robot en
    // téléop
    if (superstructure.getLancerActif() && superstructure.cibleIsHub()) {
      maxVitesseLineaire = Constants.maxVitesseLineaireLancer;
      maxVitesseRotation = Constants.maxVitesseRotationLancer;
    } else {
      maxVitesseLineaire = Constants.maxVitesseLineaire;
      maxVitesseRotation = Constants.maxVitesseRotation;
    }

    vx = vx * maxVitesseLineaire;
    vy = vy * maxVitesseLineaire;
    omega = omega * maxVitesseRotation;

    // inversion du field oriented selon l'alliance
    double invert = 1;
    if (Constants.isRedAlliance()) {
      invert = -1;
    }

    // Création du ChassisSpeed en field Oriented.
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        vx * invert,
        vy * invert,
        omega,
        basePilotable.getPose().getRotation());

    // Envoyer les vitesses aux modules
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
