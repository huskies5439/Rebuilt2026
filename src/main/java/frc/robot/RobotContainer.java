// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.lang3.time.StopWatch;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.epilogue.Logged;
import frc.robot.commands.BasePilotableDefaut;
import frc.robot.commands.LancerFancy;
import frc.robot.commands.PostShooting;
import frc.robot.commands.PreparerLancer;
import frc.robot.commands.RetracterShooting;
import frc.robot.commands.SnapTrench;
import frc.robot.commands.ViserTourelle;
import frc.robot.lib.FancyPathGeneration;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Carroussel;
import frc.robot.subsystems.Coude;
import frc.robot.subsystems.Gobeur;
import frc.robot.subsystems.Grimpeur;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kickeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Tourelle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged
public class RobotContainer {

  private final CommandXboxController manette = new CommandXboxController(0);

  private final BasePilotable basePilotable;
  private final Superstructure superstructure;
  private final Tourelle tourelle;
  private final Lanceur lanceur;
  private final Gobeur gobeur;
  private final Carroussel carroussel;
  private final Coude coude;
  private final Hood hood;
  private final Kickeur kickeur;
  private final Grimpeur grimpeur;

  private final FancyPathGeneration fancyPath;

  private Trigger protectionTrench;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    basePilotable = new BasePilotable();
    superstructure = new Superstructure(basePilotable::getPose, basePilotable::getChassisSpeeds, basePilotable::getOmegaGyro);
    tourelle = new Tourelle();
    carroussel = new Carroussel();
    lanceur = new Lanceur();
    gobeur = new Gobeur();
    coude = new Coude();
    hood = new Hood();
    kickeur = new Kickeur();
    grimpeur = new Grimpeur();

    protectionTrench = new Trigger(superstructure::isProcheTrench).negate();

    fancyPath = new FancyPathGeneration(basePilotable::getPose,
        basePilotable::getChassisSpeeds);

    configureBindings();

    basePilotable.setDefaultCommand(new BasePilotableDefaut(manette::getLeftY,
        manette::getLeftX, manette::getRightX, basePilotable));

    coude.setDefaultCommand(coude.holdCommand()); // A REMETTRE

    hood.setDefaultCommand(hood.goToAnglePIDCommand(Constants.kAngleHoodDepart)); // mauvaise intéraction avec les defer
                                                                                  // commande

    FollowPathCommand.warmupCommand().schedule();

    NamedCommands.registerCommand("gober", new WaitCommand(1));
    NamedCommands.registerCommand("preShoot", new PreparerLancer(superstructure, kickeur, lanceur, hood));
    NamedCommands.registerCommand("shoot", new LancerFancy(basePilotable, lanceur, hood, tourelle, kickeur, carroussel, superstructure));
    NamedCommands.registerCommand("grimper", new WaitCommand(1));

    // L'auto chooser doit être mis APRÈS les named commands
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  private void configureBindings() {

    // manette.povRight().whileTrue(coude.descendreCommand());
    // manette.povLeft().whileTrue(coude.monterCommand());

    manette.a().whileTrue(carroussel.tournerCommand());// .alongWith(new RetracterShooting(coude,gobeur)));

    manette.b().toggleOnTrue(
        kickeur.kickerPIDCommand()
            .alongWith(lanceur.lancerPIDCommand())
            .alongWith(hood.goToAnglePIDCommand()));

    // manette.y().whileTrue(new SnapTrench(manette::getLeftY,basePilotable));

    manette.rightTrigger().and(protectionTrench)
        .whileTrue(new LancerFancy(basePilotable, lanceur, hood, tourelle, kickeur, carroussel,
            superstructure).finallyDo(() -> new PostShooting(lanceur, carroussel, kickeur)));

    // Gober
    //manette.leftBumper().whileTrue(coude.PIDCommand(0).alongWith(gobeur.goberCommand())).onFalse(coude.PIDCommand(10.0));
    // //à déterminer s'il faut lever légerment le gobeur

    // Protection coude
    //manette.x().onTrue(coude.PIDCommand(90));

    // Grimpeur
    manette.povUp().whileTrue(grimpeur.monterCommand());
    manette.povDown().whileTrue(grimpeur.descendreCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
