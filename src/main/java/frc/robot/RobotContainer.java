// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import frc.robot.commands.BasePilotableDefaut;
import frc.robot.lib.FancyPathGeneration;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Lanceur;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

@Logged 
public class RobotContainer {

  private final CommandXboxController manette = new CommandXboxController(0);

  private final BasePilotable basePilotable;
  //private final Tourelle tourelle;
  private final Lanceur lanceur;
  //private final Gobeur gobeur;
  //private final Carroussel indexeur;
  //private final Coude coude;
  //private final Hood hood; 
  //private final Kickeur kickeur; 

  private final FancyPathGeneration fancyPathGeneration;

  public RobotContainer() {
    basePilotable = new BasePilotable();
    //tourelle = new Tourelle();
    lanceur = new Lanceur();
    //gobeur = new Gobeur();
    //indexeur = new Carroussel();
    //coude = new Coude();
    //hood = new Hood(); 
    //kickeur = new Kickeur(); 

    fancyPathGeneration = new FancyPathGeneration(basePilotable.getPoseSupplier(),
        basePilotable.getChassisSpeedsSupplier());

    configureBindings();

    basePilotable.setDefaultCommand(new BasePilotableDefaut(manette::getLeftY,
        manette::getLeftX, manette::getRightX, basePilotable));

    //coude.setDefaultCommand(coude.holdCommand());

  }

  private void configureBindings() {
    //manette.a().whileTrue(gobeur.goberCommand());
    //manette.povRight().whileTrue(coude.descendreCommand());
    //manette.povLeft().whileTrue(coude.monterCommand());

    //manette.x().whileTrue(indexeur.tournerAntiHoraireCommand());
    //manette.b().whileTrue(kickeur.tournerCommand());

    //manette.leftBumper().whileTrue(tourelle.tournerAntiHoraire());
    //manette.rightBumper().whileTrue(tourelle.tournerHoraire());

    manette.rightTrigger(0.5).whileTrue(lanceur.lancerSimpleCommand()); 
    //manette.povUp().whileTrue(hood.sortirCommand()); 
    //manette.povDown().whileTrue(hood.rentrerCommand());
  

    //Gober
    //manette.leftBumper().whileTrue(new CoudePID(0, coude).alongWith(gobeur.goberCommand())); //à déterminer s'il faut lever légerment le gobeur

    //Protection coude
    //manette.x().onTrue(new CoudePID(Constants.kAngleCoudeDepart, coude));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
