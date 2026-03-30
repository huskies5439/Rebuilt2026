// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.epilogue.Logged;
import frc.robot.commands.BasePilotableDefaut;
import frc.robot.commands.LancerFancy;
import frc.robot.commands.PostLancer;
import frc.robot.commands.PreLancer;
import frc.robot.commands.RetracterGobeurDurantLancer;
import frc.robot.commands.RumbleControllerActiveHub;
import frc.robot.commands.SnapTrench;
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

  private Trigger protectionTrench;

  private Trigger isHubActive;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    basePilotable = new BasePilotable();
    superstructure = new Superstructure(basePilotable::getPose, basePilotable::getChassisSpeeds,
        basePilotable::getOmegaGyro);
    tourelle = new Tourelle();
    carroussel = new Carroussel();
    lanceur = new Lanceur();
    gobeur = new Gobeur();
    coude = new Coude();
    hood = new Hood();
    kickeur = new Kickeur();
    grimpeur = new Grimpeur();

    isHubActive = new Trigger(superstructure::isHubActive);

    protectionTrench = new Trigger(superstructure::isProcheTrench).negate();

    configureBindings();

    basePilotable.setDefaultCommand(new BasePilotableDefaut(manette::getLeftY,
        manette::getLeftX, manette::getRightX, basePilotable, superstructure));

    coude.setDefaultCommand(coude.holdCommand());

    hood.setDefaultCommand(hood.goToAnglePIDCommand(Constants.kAngleHoodDepart));

    FollowPathCommand.warmupCommand().schedule();

    NamedCommands.registerCommand("gober", coude.PIDCommand(0).alongWith(gobeur.goberCommand()));
    NamedCommands.registerCommand("preShoot", new PreLancer(superstructure, kickeur, lanceur, hood));
    NamedCommands.registerCommand("shoot",
        new LancerFancy(basePilotable, lanceur, hood, tourelle, kickeur, carroussel, superstructure));
    NamedCommands.registerCommand("grimper", new WaitCommand(1));

    // L'auto chooser doit être mis APRÈS les named commands
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  private void configureBindings() {

    manette.a().whileTrue(carroussel.tournerCommand().alongWith(new RetracterGobeurDurantLancer(coude, gobeur)))
    .onFalse(carroussel.debloquerCommand().withTimeout(0.5));

    manette.b().toggleOnTrue(
    kickeur.kickerPIDCommand()
    .alongWith(lanceur.lancerPIDCommand())
    .alongWith(hood.goToAnglePIDCommand()));

    manette.y().whileTrue(new SnapTrench(manette::getLeftY, basePilotable));

    manette.rightBumper().and(protectionTrench)
        .whileTrue(
            new LancerFancy(basePilotable, lanceur, hood, tourelle, kickeur, carroussel,
                superstructure)

        )
        .onFalse(new PostLancer(lanceur, carroussel, kickeur, superstructure));

    manette.rightBumper().and(protectionTrench).and(manette.leftBumper().negate())
        .whileTrue(new RetracterGobeurDurantLancer(coude, gobeur));

    // Gober
    manette.leftBumper().whileTrue(coude.PIDCommand(-5).alongWith(gobeur.goberCommand()));
    // //à déterminer s'il faut lever légerment le gobeur

    // Protection coude
    manette.x().onTrue(coude.PIDCommand(90));

    // Grimpeur
    manette.povUp().whileTrue(grimpeur.monterCommand());
    manette.povDown().whileTrue(grimpeur.descendreCommand());

    // manette.b().onTrue(new ConditionalCommand(
    //     grimpeur.goMinHauteur(),
    //     grimpeur.goMaxHauteur().alongWith(coude.PIDCommand(90).withTimeout(1)),
    //     grimpeur::grimpeurHaut));

    manette.povRight().whileTrue(coude.monterCommand()); 
    manette.povLeft().whileTrue(coude.descendreCommand()); 



    // isHubActive.onTrue(new RumbleControllerActiveHub(true, manette, superstructure))
        // .onFalse(new RumbleControllerActiveHub(false, manette, superstructure));

    
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
