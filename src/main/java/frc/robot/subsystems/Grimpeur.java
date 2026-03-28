// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Grimpeur extends SubsystemBase {
  private SparkFlex moteur = new SparkFlex(59, MotorType.kBrushless);
  private SparkFlexConfig config = new SparkFlexConfig();

  private double conversion;

  private double maxPosition = 87.0;

  private boolean grimpeurHaut = false;

  public Grimpeur() {

    config.inverted(true);
    config.idleMode(IdleMode.kBrake);
    conversion = 1;
    config.encoder.positionConversionFactor(conversion); //// à vérifier
    config.encoder.velocityConversionFactor(conversion / 60);
    // config.softLimit.forwardSoftLimit(maxPosition).reverseSoftLimit(angleMax);
    // config.softLimit.forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
    moteur.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    resetEncoder();

  }

  @Override
  public void periodic() {

  }

  public void setVoltage(double voltage) {
    moteur.setVoltage(voltage);
  }

  public double getPosition() {
    return moteur.getEncoder().getPosition();
  }

  public double getVelocity() {
    return moteur.getEncoder().getVelocity();
  }

  public void stop() {
    setVoltage(0);
  }

  public void monter() {
    setVoltage(6);
  }

  public void descendre() {
    setVoltage(-6);
  }

  public boolean grimpeurHaut() {
    return grimpeurHaut;
  }

  ///// ENCODEUR

  public void resetEncoder() {
    moteur.getEncoder().setPosition(0);
  }

  /// COMMANDES
  public Command monterCommand() {
    return Commands.runEnd(this::monter, this::stop, this);
  }

  public Command descendreCommand() {
    return Commands.runEnd(this::descendre, this::stop, this);
  }

  public Command goMaxHauteur() {

    return Commands.runOnce(() -> {
      grimpeurHaut = true;
    })
        .andThen(Commands.runEnd(this::monter, this::stop, this).until(() -> {
          return getPosition() > maxPosition;
        }));

  }

  public Command goMinHauteur() {

    return Commands.runOnce(() -> {
      grimpeurHaut = false;
    })
        .andThen(Commands.runEnd(this::descendre, this::stop, this).until(() -> {
          return getPosition() < 0.0;
        }));
  }

}
