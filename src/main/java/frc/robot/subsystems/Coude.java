// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.dense.row.decomposition.chol.CholeskyDecomposition_FDRB_to_FDRM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Coude extends SubsystemBase {

  private SparkMax moteurGauche = new SparkMax(21, MotorType.kBrushless);
  private SparkMax moteurDroit = new SparkMax(22, MotorType.kBrushless);

  private SparkMaxConfig moteurConfigGauche = new SparkMaxConfig();
  private SparkMaxConfig moteurConfigDroit = new SparkMaxConfig();


  private DigitalInput limitSwitchGauche = new DigitalInput(0);
  private DigitalInput limitSwitchDroite = new DigitalInput(1);

  private double conversionCoude = (1 / 5.0) * (1 / 5.0) * (1 / 3.0) * 360.0;

  /// PID et feedForward
  
  private final double kp = 0.1;
  private final double maxVelocity = 360;
  private final double maxAcceleration = 720;

  private ProfiledPIDController pidGauche = new ProfiledPIDController(kp, 0, 0,
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

  private ProfiledPIDController pidDroit = new ProfiledPIDController(kp, 0, 0,
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

  private ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.2, 0.0);// valeur à déterminer

  private int stallLimit; 

  /** Creates a new Coude. */
  public Coude() {

    boolean inverted = true;
    moteurConfigGauche.inverted(inverted);
    moteurConfigGauche.idleMode(IdleMode.kBrake);
    moteurConfigGauche.encoder.positionConversionFactor(conversionCoude);
    moteurConfigGauche.encoder.velocityConversionFactor(conversionCoude / 60.0);

    moteurGauche.configure(moteurConfigGauche, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    moteurConfigDroit.inverted(!inverted);
    moteurConfigDroit.idleMode(IdleMode.kBrake);
    moteurConfigDroit.encoder.positionConversionFactor(conversionCoude);
    moteurConfigDroit.encoder.velocityConversionFactor(conversionCoude / 60.0);

    moteurDroit.configure(moteurConfigDroit, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetEncodeurStartUp();

    pidGauche.setTolerance(2);
    pidDroit.setTolerance(2);

    currentLimit(false);

  }

  @Override
  public void periodic() {

    if (isLimitSwitchGauche()) {
      resetEncodeurLimitSwitchGauche();
    }
    if (isLimitSwitchDroite()) {
      resetEncodeurLimitSwitchDroite();
    }
    

  }

  public void setVoltage(double voltageGauche, double voltageDroit) {
    moteurGauche.setVoltage(voltageGauche);
    moteurDroit.setVoltage(voltageDroit);
  }

  public void setVoltage(double voltage) {
    setVoltage(voltage, voltage);
  }

  public void monter() {
    setVoltage(1);
  }

  public void descendre() {
    setVoltage(-1);
  }

  public void stop() {
    setVoltage(0);
  }

  public void hold() {
    setVoltage(feedforward.calculate(Math.toRadians(getAngleGauche()), 0),
                feedforward.calculate(Math.toRadians(getAngleDroit()), 0));
  }

  /// Encodeur Gauche

  public double getAngleGauche() {
    return moteurGauche.getEncoder().getPosition();
  }

  public double getVitesseGauche() {
    return moteurGauche.getEncoder().getVelocity();
  }

  /// Encodeur Droit

  public double getAngleDroit() {
    return moteurDroit.getEncoder().getPosition();
  }

  public double getVitesseDroit() {
    return moteurDroit.getEncoder().getVelocity();
  }


  /// Encodeurs

  public void resetEncodeurLimitSwitchGauche() {// Quand on clique la limit switch
    moteurGauche.getEncoder().setPosition(0); // À determiner
  }

  public void resetEncodeurLimitSwitchDroite() {// Quand on clique la limit switch
    moteurDroit.getEncoder().setPosition(0); // À determiner
  }

  public void resetEncodeurStartUp() {
    moteurDroit.getEncoder().setPosition(Constants.kAngleCoudeDepart);// à déterminer
    moteurGauche.getEncoder().setPosition(Constants.kAngleCoudeDepart);// à déterminer
  }

  /// PID + feedForward
  public void setPID(double cible) {
    double voltagePIDDroit = pidDroit.calculate(getAngleDroit(), cible);

    double voltageFFDroit = feedforward.calculate(
        Math.toRadians(getAngleDroit()),
        pidDroit.getSetpoint().velocity);

    double voltagePIDGauche = pidGauche.calculate(getAngleGauche(), cible);

    double voltageFFGauche = feedforward.calculate(
        Math.toRadians(getAngleGauche()),
        pidGauche.getSetpoint().velocity);

    setVoltage(voltagePIDGauche + voltageFFGauche, voltagePIDDroit + voltageFFDroit);

  }

  public void resetPID() {
    pidDroit.reset(getAngleDroit());
    pidGauche.reset(getAngleGauche());
  }

  public boolean atCible() {
    return pidDroit.atGoal() && pidGauche.atGoal();
  }

  //smart current limit 
   public void currentLimit(boolean isLimited){
    if(isLimited){
       stallLimit = 24; //24
      }else{
        stallLimit = 999999999; //Pas assez
      }
      moteurConfigGauche.smartCurrentLimit(stallLimit); 
      moteurConfigDroit.smartCurrentLimit(stallLimit);
      
      moteurGauche.configure(moteurConfigGauche, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      moteurDroit.configure(moteurConfigDroit, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

   public double getCurrentLimitGauche(){
    return moteurGauche.getOutputCurrent(); 
   }

   public double getCurrentLimitDroite(){
    return moteurDroit.getOutputCurrent(); 
   }

  /// limit switch

  public boolean isLimitSwitchGauche() {
    return !limitSwitchGauche.get();
  }

  public boolean isLimitSwitchDroite() {
    return !limitSwitchDroite.get();
  }

  /// commandes

  public Command monterCommand() {
    return Commands.runEnd(this::monter, this::stop, this);
  }

  public Command descendreCommand() {
    return Commands.runEnd(this::descendre, this::stop, this);
  }

  public Command holdCommand() {
    return Commands.run(this::hold, this);
  }

  public Command PIDCommand(double cible) {
    return Commands.runOnce(this::resetPID, this).andThen(Commands.run(()->this.setPID(cible), this));
  }

}
