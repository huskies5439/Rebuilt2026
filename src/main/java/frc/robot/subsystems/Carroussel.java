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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Carroussel extends SubsystemBase {

  private SparkFlex moteur = new SparkFlex(11, MotorType.kBrushless);

  private SparkFlexConfig config = new SparkFlexConfig();

  // moteur avec maxPlanetary, poulie 36 vers 72, 360 degrées
  private double maxPlanetary = 9.0;
  private double conversion = (1.0 / maxPlanetary) * (36.0 / 72.0) * 360;

  private PIDController pid = new PIDController(0, 0, 0);
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);
  private SlewRateLimiter limiter = new SlewRateLimiter(10);
  private double vraieCible = 0.0;

  
  public Carroussel() {

    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(conversion);
    config.encoder.velocityConversionFactor(conversion / 60.0);
    moteur.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    SmartDashboard.putNumber("voltage carroussel",  5);//Initialise input open loop dans le dashboard
    SmartDashboard.putNumber("cible carroussel", 0);////Initialise input PID dans le dashboard

  }

  @Override
  public void periodic() {
  
  }

  //////////MOTEUR
  public void setVoltage(double voltage) {
    moteur.setVoltage(voltage);
  }

  public void tourner() {
    setVoltage(SmartDashboard.getNumber("voltage carroussel", 0));
  }

  public void debloquer() {
    setVoltage(-2);
  }

  public void stop() {
    setVoltage(0);
    resetPID();
  }


  ////////ENCODEUR
  
  public double getPosition(){/////nécessaire ??
    return (moteur.getEncoder().getPosition());
  }
  @Logged (name = "Vitesse Caroussel")
  public double getVitesse(){
    return moteur.getEncoder().getVelocity();
  }


  /////PID
  
  private void setVraieCible(double cible){
    vraieCible = cible;
  }
  public double getVraieCible() {
      return vraieCible;
  }
  public void setPID(double cible){
    setVraieCible(cible);
    double cibleCorriger = limiter.calculate(cible);
     setVoltage(
        ff.calculate(cibleCorriger) + pid.calculate(getVitesse(), cibleCorriger));
  }

  @Logged(name = "At Cible Carroussel")
  public boolean atCible(){
    return Math.abs(getVitesse() - getVraieCible()) <= 1;
  }

  public void resetPID() {
    setVraieCible(0);
    pid.reset();
  }

  ////// Commandes

  public Command tournerCommand() {
    return Commands.runEnd(this::tourner, this::stop, this);
  }

  public Command debloquerCommand() {
    return Commands.runEnd(this::debloquer, this::stop, this);
  }

  
  public Command tournerPIDCommand(double cible){
    return Commands.runOnce(()-> limiter.reset(getVitesse()))
    .andThen(Commands.runEnd(() -> setPID(cible), this::stop,this ));
  }

  public Command tournerPIDCommand(){
    return tournerPIDCommand(SmartDashboard.getNumber("cible carroussel", 0));
  }

  

}
