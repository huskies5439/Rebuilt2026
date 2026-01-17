// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lanceur extends SubsystemBase {
  /** Creates a new Lanceur. */

  private SparkFlex lanceur1 = new SparkFlex(11, MotorType.kBrushless);
  private SparkFlex lanceur2 = new SparkFlex(12, MotorType.kBrushless);
  private SparkMax lanceurCapot = new SparkMax(13, MotorType.kBrushless);

  private SparkFlexConfig lanceurConfig1 = new SparkFlexConfig();
  private SparkFlexConfig lanceurConfig2 = new SparkFlexConfig();
  private SparkMaxConfig lanceurConfigCapot = new SparkMaxConfig();

  private double conversionLanceur = 1.0;
  private double conversionCapot = 1.0;

  public Lanceur() {
    lanceurConfig1.inverted(false);
    lanceurConfig1.idleMode(IdleMode.kCoast);
    lanceurConfig1.encoder.positionConversionFactor(conversionLanceur);
    lanceurConfig1.encoder.velocityConversionFactor(conversionLanceur / 60.0);
    lanceur1.configure(lanceurConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    lanceurConfig2.inverted(false);
    lanceurConfig2.idleMode(IdleMode.kCoast);
    lanceurConfig2.encoder.positionConversionFactor(conversionLanceur);
    lanceurConfig2.encoder.velocityConversionFactor(conversionLanceur / 60.0);
    lanceur2.configure(lanceurConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    lanceurConfigCapot.inverted(false);
    lanceurConfigCapot.idleMode(IdleMode.kBrake);
    lanceurConfigCapot.encoder.positionConversionFactor(conversionCapot);
    lanceurConfigCapot.encoder.velocityConversionFactor(conversionCapot / 60.0);
    lanceurCapot.configure(lanceurConfigCapot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVoltageLanceur(double voltage) {
    lanceur1.setVoltage(voltage);
    lanceur2.setVoltage(voltage);
  }

  public void lancer() {
    setVoltageLanceur(1);
  }

  public void stop() {
    setVoltageLanceur(0);
  }

  public void setVoltageCapot(double voltage) {
    lanceurCapot.setVoltage(voltage);
  }

  public void stopCapot() {
    lanceurCapot.setVoltage(0);
  }

  public void sortirCapot() {
    setVoltageCapot(2);
  }

  public void rentrerCapot() {
    setVoltageCapot(-2);
  }

  public double getPositionLanceur() {
    return (lanceur1.getEncoder().getPosition() + lanceur2.getEncoder().getPosition()) / 2.0;
  }

  public double getVitesseLanceur() {
    return (lanceur1.getEncoder().getVelocity() + lanceur2.getEncoder().getVelocity()) / 2.0;
  }

  public double getPositionCapot() {
    return lanceurCapot.getEncoder().getPosition();
  }

  public Command lancerSimpleCommand() {
    return Commands.runEnd(this::lancer, this::stop, this);
  }

  public Command rentrerCapotCommand() {
    return Commands.runEnd(this::rentrerCapot, this::stopCapot, this);
  }

  public Command sortirCapotCommand() {
    return Commands.runEnd(this::sortirCapot, this::stopCapot, this);
  }

}
