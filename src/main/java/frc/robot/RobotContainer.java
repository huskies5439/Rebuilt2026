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
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.BasePilotableDefaut;
import frc.robot.commands.LancerFancy;
import frc.robot.commands.PostLancer;
import frc.robot.commands.PreLancerAutonome;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged
public class RobotContainer {

    private final CommandXboxController manettePilote = new CommandXboxController(0);

    private final CommandXboxController manetteCopilote = new CommandXboxController(1);

    private final BasePilotable basePilotable;
    private final Superstructure superstructure;
    private final Tourelle tourelle;
    private final Lanceur lanceur;
    private final Gobeur gobeur;
    private final Carroussel carroussel;
    private final Coude coude;
    private final Hood hood;
    private final Kickeur kickeur;

    private Trigger protectionTrench;

    private Trigger isHubActive;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        basePilotable = new BasePilotable();
        superstructure =
            new Superstructure(basePilotable::getPose, basePilotable::getChassisSpeeds, basePilotable::getOmegaGyro);
        tourelle = new Tourelle();
        carroussel = new Carroussel();
        lanceur = new Lanceur();
        gobeur = new Gobeur();
        coude = new Coude();
        hood = new Hood();
        kickeur = new Kickeur();


        isHubActive = new Trigger(() -> superstructure.isHubActive(Constants.PREVISION_TIME));

        protectionTrench = new Trigger(superstructure::isProcheTrench).negate();

        configureBindings();

        basePilotable.setDefaultCommand(new BasePilotableDefaut(
            manettePilote::getLeftY,
            manettePilote::getLeftX,
            manettePilote::getRightX,
            basePilotable,
            superstructure));

        coude.setDefaultCommand(coude.holdCommand());

        hood.setDefaultCommand(hood.goToAnglePIDCommand(Constants.kAngleHoodDepart));

        lanceur.setDefaultCommand(Commands.run(lanceur::stop, lanceur));

        kickeur.setDefaultCommand(Commands.run(kickeur::stop, kickeur));

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        NamedCommands.registerCommand("gober", coude.PIDCommand(0).alongWith(gobeur.goberCommand()));

        NamedCommands.registerCommand("preShoot", new PreLancerAutonome(superstructure, kickeur, lanceur));

        NamedCommands.registerCommand(
            "shoot",
            new LancerFancy(
                basePilotable,
                lanceur,
                hood,
                tourelle,
                kickeur,
                carroussel,
                superstructure));

        NamedCommands.registerCommand("retracter", new RetracterGobeurDurantLancer(coude, gobeur));

        NamedCommands.registerCommand("viser depot", tourelle.PIDCommand(100));

        NamedCommands.registerCommand("viser outpost", tourelle.PIDCommand(-100));

        NamedCommands.registerCommand("retracterHood", hood.goToAnglePIDCommand(Constants.kAngleHoodDepart));

        NamedCommands.registerCommand("recracher ballon", gobeur.cracherCommand().withTimeout(0.5));

        // L'auto chooser doit être mis APRÈS les named commands
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

        // manette.a().whileTrue(carroussel.tournerCommand().alongWith(new
        // RetracterGobeurDurantLancer(coude, gobeur)))
        // .onFalse(carroussel.debloquerCommand().withTimeout(0.5));

        // manette.b().toggleOnTrue(
        // kickeur.kickerPIDCommand()
        // .alongWith(lanceur.lancerPIDCommand())
        // .alongWith(hood.goToAnglePIDCommand()));

        manettePilote.a().whileTrue(new SnapTrench(manettePilote::getLeftY, basePilotable));

        manettePilote
            .rightBumper()
            .and(protectionTrench)
            .whileTrue(new LancerFancy(basePilotable, lanceur, hood, tourelle, kickeur, carroussel, superstructure))
            .onFalse(new PostLancer(lanceur, carroussel, kickeur, superstructure));

        manettePilote
            .rightBumper()
            .and(protectionTrench)
            .and(manettePilote.leftBumper().negate())
            .whileTrue(new WaitCommand(1).andThen(new RetracterGobeurDurantLancer(coude, gobeur)));

        // Gober
        manettePilote.leftBumper().whileTrue(coude.PIDCommand(7).alongWith(gobeur.goberCommand()));

        // Protection coude
        manettePilote.x().onTrue(coude.PIDCommand(120));
      
        isHubActive
            .onTrue(new RumbleControllerActiveHub(true, manettePilote, manetteCopilote))
            .onFalse(new RumbleControllerActiveHub(false, manettePilote, manetteCopilote));

        manetteCopilote.povUp().onTrue(superstructure.plusTrimLanceur().alongWith(superstructure.plusTrimKickeur()));
        manetteCopilote
            .povDown()
            .onTrue(superstructure.moinsTrimLanceur().alongWith(superstructure.moinsTrimKickeur()));

        manetteCopilote.povLeft().onTrue(superstructure.plusTrimTourelle());
        manetteCopilote.povRight().onTrue(superstructure.moinsTrimTourelle());

        //top secret

        manetteCopilote.a().onTrue(Commands.runOnce(basePilotable :: MegaTag1));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}