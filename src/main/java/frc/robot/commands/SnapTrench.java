// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PidBasePilotable;
import frc.robot.Constants.PositionYTrench;
import frc.robot.subsystems.BasePilotable;

@Logged
public class SnapTrench extends Command {

  BasePilotable basePilotable;
  PIDController pidY;
  PIDController pidAngle;
  double cibleY;
  double cibleAngle;
  ChassisSpeeds chassisSpeeds;
  private DoubleSupplier vx;

  public SnapTrench(DoubleSupplier vx, BasePilotable basePilotable) {
    this.vx = vx;
    this.basePilotable = basePilotable;
    addRequirements(basePilotable);

    pidY = new PIDController(PidBasePilotable.kPLineaire, 0, PidBasePilotable.kDLineaire);
    pidY.setTolerance(0.02);

    pidAngle = new PIDController(PidBasePilotable.kPRot, 0, PidBasePilotable.kDRot);
    pidAngle.enableContinuousInput(-Math.PI, Math.PI);
    pidAngle.setTolerance(Math.toRadians(2.0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidY.reset();

    pidAngle.reset();

    if (basePilotable.getPose().getY() >= 4.0) {
      cibleY = PositionYTrench.trenchHaut;
    } else {
      cibleY = PositionYTrench.trenchBas;
    }

    if (Constants.isRedAlliance()) {
      cibleAngle = Math.toRadians(0);
    } else {
      cibleAngle = Math.toRadians(180);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // appliquer une deadband sur les joysticks et corriger la direction
    double vxMetersPerSecond = -MathUtil.applyDeadband(vx.getAsDouble(), 0.05);

    double invert = 1;
    if (Constants.isRedAlliance()) {
      invert = -1;
    }

    vxMetersPerSecond = vxMetersPerSecond * Math.abs(vxMetersPerSecond) * Constants.maxVitesseLineaire * invert;

    double vyMetersPerSecond = pidY.calculate(basePilotable.getPose().getY(), cibleY);

    double omegaRadiansPerSecond = pidAngle.calculate(basePilotable.getPose().getRotation().getRadians(), cibleAngle);

    if (pidY.atSetpoint()) {
      vyMetersPerSecond = 0.0;
    }

    if (pidAngle.atSetpoint()) {
      omegaRadiansPerSecond = 0.0;
    }

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond,
        basePilotable.getPose().getRotation());

    basePilotable.conduireChassisSetPoint(chassisSpeeds);

    SmartDashboard.putNumber("vy", vyMetersPerSecond);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
