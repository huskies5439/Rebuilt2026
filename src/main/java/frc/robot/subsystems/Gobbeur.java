// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gobbeur extends SubsystemBase {
  /** Creates a new Gobbeur. */

  private SparkFlex MoteurRouleau = new SparkFlex(0, MotorType.kBrushless);
  private SparkFlex MoteurCoude = new SparkFlex(0, MotorType.kBrushless);
  // changer noms pour MoteurQuiDessend et MoteurCouroie

  private SparkFlexConfig MoteurRouleauConfig = new SparkFlexConfig();
  private SparkFlexConfig MoteurCoudeConfig = new SparkFlexConfig();

  private double conversionCoude = 1.0;


  public Gobbeur() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
