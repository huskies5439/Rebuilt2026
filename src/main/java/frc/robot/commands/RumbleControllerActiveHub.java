// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleControllerActiveHub extends Command {

  //Avant de descendre plus loin, veuillez noter que le code si-dessous est extrêmement laid!
  //Si vous ne voulez point devenir aveugle pour le restant de votre vie, ne descendez pas plus loin!

  CommandXboxController manette = new CommandXboxController(0); 
  Boolean hasChanged = false; 

  int changeBufferMAX = 30;
  int changeBuffer = changeBufferMAX; 

  int step = 1; 

  int heartBuffer1MAX = 30; 
  int heartBuffer1 = heartBuffer1MAX; 
  boolean beatInit = false; 

  int heartBuffer2MAX = 20;
  int heartBuffer2 = heartBuffer2MAX;

  int heartBuffer4MAX = 60; 
  int heartBuffer4 = heartBuffer4MAX; 



  public RumbleControllerActiveHub(CommandXboxController manette) {
    this.manette = manette; 
  }

  @Override
  public void initialize() {

  }
 
  @Override
   public void execute() {
  //  if(isHubActive()){
  //     if(!hasChanged){
       setBeat();
  //      if(changeBuffer > 0){
  //       changeBuffer--;
  //      }else{
  //        hasChanged = true; 
  //      }
  //     }
  //   }else{
  //      manette.setRumble(RumbleType.kBothRumble,1.0); 
  //      changeBuffer = changeBufferMAX; 
  //      hasChanged = false; 
  //   }

 }


  @Override
  public void end(boolean interrupted) {}

 
  @Override
  public boolean isFinished() {
    return false;
  }


  public void setBeat(){
      manette.setRumble(RumbleType.kBothRumble,1);   
    if(step == 1){
      if(!beatInit){
        heartBuffer1 = heartBuffer1MAX; 
        beatInit = true; 
      }
      manette.setRumble(RumbleType.kBothRumble,0.5);   
      if(heartBuffer1 > 0){
        heartBuffer1--; 
      }else{
        beatInit = false; 
        step = 2; 
      }

    }else if(step == 2){
      if(!beatInit){
        heartBuffer2 = heartBuffer2MAX; 
        beatInit = true; 
      }
      manette.setRumble(RumbleType.kBothRumble,0.0);
      if(heartBuffer2 > 0){
        heartBuffer2--;
      }else{
        beatInit = false; 
        step = 1; 
      }
    }else if(step == 3){
       if(!beatInit){
        heartBuffer1 = heartBuffer1MAX; 
        beatInit = true; 
      }
      manette.setRumble(RumbleType.kBothRumble,0.8);   
      if(heartBuffer1 > 0){
        heartBuffer1--; 
      }else{
        beatInit = false; 
        step = 4; 
      }
    }else if(step == 4){
      if(!beatInit){
        heartBuffer4 = heartBuffer4MAX; 
        beatInit = true; 
      }
      manette.setRumble(RumbleType.kBothRumble,0.0);   
      if(heartBuffer4 > 0){
        heartBuffer4--; 
      }else{
        beatInit = false; 
        step = 1; 
      }
    }
  }



  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }
}
