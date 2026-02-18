// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


@Logged
public class Kickeur extends SubsystemBase {
  private SparkFlex moteur = new SparkFlex(51, MotorType.kBrushless);
  private SparkFlexConfig config = new SparkFlexConfig();

  double conversionKikeur = (15.0 / 36.0);

  public Kickeur() {
    config.inverted(false);
    config.idleMode(IdleMode.kCoast);
    config.encoder.positionConversionFactor(conversionKikeur);
    config.encoder.velocityConversionFactor(conversionKikeur / 60.0);
    moteur.configure(config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SmartDashboard.putNumber("voltage kickeur",  5);
    
    resetEncodeur();
  }

  @Override
  public void periodic() {}

  public void setVoltage(double voltage) {
    moteur.setVoltage(voltage);
  }


  @Logged (name = "Vitesse Kickeur")
  public double getVitesse() {
    return moteur.getEncoder().getVelocity();
  }

  @Logged (name = "Position Kickeur")
  public double getPosition() {
    return moteur.getEncoder().getPosition();
  }

  public void tourner() {
    setVoltage(SmartDashboard.getNumber("voltage kickeur", 0));
  }

  public void stop() {
    setVoltage(0);
  }

  public void resetEncodeur() {
    moteur.getEncoder().setPosition(0);
  }

  public Command tournerCommand() {
    return Commands.runEnd(this::tourner, this::stop, this);
  }
}
