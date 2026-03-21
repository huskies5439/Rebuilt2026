// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.epilogue.Logged;
import frc.robot.commands.BasePilotableDefaut;
import frc.robot.commands.LancerFancy;
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

  public RobotContainer() {
    basePilotable = new BasePilotable();
    superstructure = new Superstructure(basePilotable.getPoseSupplier(), basePilotable.getChassisSpeedsSupplier()); 
    tourelle = new Tourelle();
    carroussel = new Carroussel(); 
    lanceur = new Lanceur();
    gobeur = new Gobeur();
    coude = new Coude();
    hood = new Hood(); 
    kickeur = new Kickeur(); 
    grimpeur = new Grimpeur();


    protectionTrench = new Trigger(superstructure::isProcheTrench).negate();
    

    fancyPath = new FancyPathGeneration(basePilotable.getPoseSupplier(),
        basePilotable.getChassisSpeedsSupplier());

    configureBindings();

    basePilotable.setDefaultCommand(new BasePilotableDefaut(manette::getLeftY,
        manette::getLeftX, manette::getRightX, basePilotable));

    coude.setDefaultCommand(coude.holdCommand());

    //hood.setDefaultCommand(hood.goToAnglePIDCommand(Constants.kAngleHoodDepart)); //mauvaise intéraction avec les defer commande


    FollowPathCommand.warmupCommand().schedule();

    NamedCommands.registerCommand("gober", new WaitCommand(10.0));
    NamedCommands.registerCommand("preShoot", new WaitCommand(10.0));
    NamedCommands.registerCommand("shoot", new WaitCommand(10.0));
    NamedCommands.registerCommand("grimper", new WaitCommand(10.0));




  }

  private void configureBindings() {

    //manette.a().whileTrue(gobeur.goberCommand());

    manette.povRight().whileTrue(coude.descendreCommand());
    manette.povLeft().whileTrue(coude.monterCommand());
    

    manette.a().whileTrue(carroussel.tournerCommand());

     manette.b().toggleOnTrue(kickeur.kickerPIDCommand().alongWith(lanceur.lancerPIDCommand()).alongWith(hood.goToAnglePIDCommand(64)));
    manette.b().whileTrue(hood.goToAnglePIDCommand());

    // manette.leftBumper().whileTrue(tourelle.tournerAntiHoraire());
    // manette.rightBumper().whileTrue(tourelle.tournerHoraire());

    // manette.a().whileTrue(new ViserTourelle(tourelle, superstructure));

    //manette.rightTrigger(0.5).whileTrue(lanceur.lancerSimpleCommand()); 
   // manette.povUp().whileTrue(hood.sortirCommand()); 
   // manette.povDown().whileTrue(hood.rentrerCommand());

   // manette.a().whileTrue(new SnapTrench(manette::getLeftY,basePilotable)); 

    //manette.rightBumper().and(protectionTrench).whileTrue(new LancerFancy(basePilotable, lanceur, hood, null, kickeur, carroussel, superstructure));

  
    //Gober
    manette.leftBumper().whileTrue(coude.PIDCommand(0).alongWith(gobeur.goberCommand())).onFalse(coude.PIDCommand(10.0)); //à déterminer s'il faut lever légerment le gobeur

    //Protection coude
    manette.x().onTrue(coude.PIDCommand(90));

    //Grimpeur
     manette.povUp().whileTrue(grimpeur.monterCommand());
     manette.povDown().whileTrue(grimpeur.descendreCommand());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
