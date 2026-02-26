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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
@Logged
public class Grimpeur extends SubsystemBase {
  private SparkFlex moteur = new SparkFlex(59, MotorType.kBrushless);
  private SparkFlexConfig config = new SparkFlexConfig();

  private Servo servo = new Servo(3);

  private double conversion;

  private double maxPosition = 0.5; // à vérifier

  private double angleServoBarrer = 70.0; // à vérifier

  private double angleServoDebarrer = 105.0; // à vérifier

  private double voltageRapide = 3.0; // à vérifier
  private double volatgeLent = 0.25; // à vérifier

    
  public Grimpeur() {

    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    conversion = 1;
    config.encoder.positionConversionFactor(conversion); //// à vérifier
    config.encoder.velocityConversionFactor(conversion / 60);
    //config.softLimit.forwardSoftLimit(maxPosition).reverseSoftLimit(0);
    //config.softLimit.forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
    moteur.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    debarrer();

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
    setVoltage(3);
  }

  public void descendre() {
    setVoltage(-3);
  }

  ///// ENCODEUR

  public void resetEncoder() {
    moteur.getEncoder().setPosition(0);
  }

  /// SERVO

  public void barrer() {
    servo.setAngle(angleServoBarrer);
  }

  public void debarrer() {
    servo.setAngle(angleServoDebarrer);
  }

  public void setServoAngle(double angle) {
    servo.setAngle(angle);
  }

  /// COMMANDES

  public Command monterCommand() {
    return Commands.runEnd(this::monter, this::stop, this);
  }

  public Command descendreCommand() {
    return Commands.runEnd(this::descendre, this::stop, this);
  }

}
