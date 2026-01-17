// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Gobeur;
import frc.robot.subsystems.Indexeur;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Gobeur gobeur = new Gobeur();
  private final Indexeur indexeur = new Indexeur();

  private final CommandXboxController manette =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

 
  private void configureBindings() {
    manette.a().whileTrue(gobeur.avalerCommand());
    manette.b().whileTrue(indexeur.tournerAntiHoraireCarousselCommand());
    manette.x().whileTrue(indexeur.AccelerateurCommand());
    manette.y().whileTrue(gobeur.descendreCoudeCommand());
    manette.povLeft().whileTrue(gobeur.monterCoudeCommand());
      
  }


  
  public Command getAutonomousCommand() {
   return null; 
  }
}
