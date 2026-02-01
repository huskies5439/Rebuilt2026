// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PidBasePilotable;
import frc.robot.Constants.PositionYTrench;
import frc.robot.subsystems.BasePilotable;

public class SnapTrench extends Command {

  BasePilotable basePilotable;
  ProfiledPIDController pidY;
  ProfiledPIDController pidAngle;
  double cibleY;
  double cibleAngle;
  ChassisSpeeds chassisSpeeds;

  public SnapTrench(BasePilotable basePilotable) {
    this.basePilotable = basePilotable;
    addRequirements(basePilotable);

    pidY = new ProfiledPIDController(PidBasePilotable.kPLineaire, 0, 0, 
            new TrapezoidProfile.Constraints(PidBasePilotable.kMaxVitesseLineaire, PidBasePilotable.kMaxAccelLineaire));

    pidAngle = new ProfiledPIDController(PidBasePilotable.kPRot, 0, PidBasePilotable.kDRot, 
            new TrapezoidProfile.Constraints(PidBasePilotable.kMaxVitesseRot, PidBasePilotable.kMaxAccelRot));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    pidY.reset(basePilotable.getPose().getY(), basePilotable.getChassisSpeeds().vyMetersPerSecond);

    pidAngle.reset(basePilotable.getPose().getRotation().getRadians(), basePilotable.getChassisSpeeds().omegaRadiansPerSecond);

    if (basePilotable.getPose().getY() >= 4.0) {
      cibleY = PositionYTrench.trenchHaut;
    } else {
      cibleY = PositionYTrench.trenchBas;
    }

    if (basePilotable.isRedAlliance()) {
      cibleAngle = Math.toRadians(0);
    } else {
      cibleAngle = Math.toRadians(180);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassisSpeeds.vyMetersPerSecond = pidY.calculate(basePilotable.getPose().getY(), cibleY);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
