// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Hood extends SubsystemBase {
  private SparkMax moteur = new SparkMax(41, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();

  // moteur avec maxPlanetary, enngrenage 40 vers 24, 16 * dent utiliser/nombre de dents théorique * 360 degrées
  private double maxPlanetary = (1.0/5.0)*(1.0/5.0);

  private double conversion = maxPlanetary*(40.0/24.0)*(16.0/240.0)*360.0;

  private DigitalInput limitSwitch = new DigitalInput(2);

  private ProfiledPIDController profiledPID = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(0, 0));

  public Hood() {

    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(conversion);
    config.encoder.velocityConversionFactor(conversion / 60.0);
    moteur.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

    if (isLimitSwitch()) {
      resetEncodeur();
    }

  }

  public void setVoltage(double voltage) {
    moteur.setVoltage(voltage);
  }

  public double getAngle() {
    return moteur.getEncoder().getPosition();
  }

  public double getVitesse() {
    return moteur.getEncoder().getVelocity();
  }

  public void resetEncodeur() {
    moteur.getEncoder().setPosition(0);// à determiner
  }

  public void stop() {
    resetPID();
    moteur.setVoltage(0);
  }

  public void sortir() {
    setVoltage(2);
  }

  public void rentrer() {
    setVoltage(-2);
  }

  public Command rentrerCommand() {
    return Commands.runEnd(this::rentrer, this::stop, this);
  }

  public Command sortirCommand() {
    return Commands.runEnd(this::sortir, this::stop, this);
  }

  /// PID

  public void setPID(double cible) {

    //Quand on rétracte le hood, on triche dans le derniers degrés pour s'accoter sur la switch
    if (cible >= Constants.angleHoodLimitSwitch && getAngle() >= (Constants.angleHoodLimitSwitch - 2)) { 
      if (isLimitSwitch()) {
        stop();
      } else {
        rentrer();
      }
      //PID Normal
    } else {
      double voltage = profiledPID.calculate(getAngle(), cible);
      setVoltage(voltage);
    }
  }

  public void resetPID() {
    profiledPID.reset(getAngle());
  }

  public boolean atCible() {
    return profiledPID.atGoal();
  }

  // Limit switch

  public boolean isLimitSwitch() {
    return !limitSwitch.get();
  }

  public Command goToAnglePIDCommand(double cible) {
    return Commands.runEnd(() -> setPID(cible), this::stop, this);
  }

}
