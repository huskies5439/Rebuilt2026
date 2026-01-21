// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Gobeur;
import frc.robot.subsystems.Indexeur;
import frc.robot.commands.BasePilotableDefaut;
import frc.robot.lib.FancyPathGeneration;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Tourelle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {//coucou

  private final CommandXboxController manette = new CommandXboxController(0);

  private final BasePilotable basePilotable;
  private final Tourelle tourelle;
  private final Lanceur lanceur;
  private final Gobeur gobeur;
  private final Indexeur indexeur;

  private final FancyPathGeneration fancyPathGeneration;

  public RobotContainer() {
    basePilotable = new BasePilotable();
    tourelle = new Tourelle();
    lanceur = new Lanceur();
    gobeur = new Gobeur();
    indexeur = new Indexeur();

    fancyPathGeneration = new FancyPathGeneration(basePilotable.getPoseSupplier(),
        basePilotable.getChassisSpeedsSupplier());

    configureBindings();

    basePilotable.setDefaultCommand(new BasePilotableDefaut(manette::getLeftY,
        manette::getLeftX, manette::getRightX, basePilotable));

  }

  private void configureBindings() {
    manette.a().whileTrue(gobeur.avalerCommand());
    manette.povRight().whileTrue(gobeur.descendreCoudeCommand());
    manette.povLeft().whileTrue(gobeur.monterCoudeCommand());

    manette.x().whileTrue(indexeur.tournerAntiHoraireCarousselCommand());
    manette.b().whileTrue(indexeur.AccelerateurCommand());

    manette.leftBumper().whileTrue(tourelle.tournerAntiHoraire());
    manette.rightBumper().whileTrue(tourelle.tournerHoraire());

    manette.rightTrigger(0.5).whileTrue(lanceur.lancerSimpleCommand()); 
    manette.povUp().whileTrue(lanceur.sortirCapotCommand()); 
    manette.povDown().whileTrue(lanceur.rentrerCapotCommand()); 

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
