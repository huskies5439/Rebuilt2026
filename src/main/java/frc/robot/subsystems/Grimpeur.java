// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged(strategy = Strategy.OPT_IN)
public class Grimpeur extends SubsystemBase {

    private SparkFlex moteur = new SparkFlex(59, MotorType.kBrushless);
    private SparkFlexConfig config = new SparkFlexConfig();

    private double conversion;

    private double maxPosition = 80.0;

    private boolean grimpeurHaut = false;

    public Grimpeur() {

        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        conversion = 1;
        config.encoder.positionConversionFactor(conversion);
        config.encoder.velocityConversionFactor(conversion / 60);
        moteur.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        resetEncoder();

    }

    @Override
    public void periodic() {

    }

    //Moteur
    public void setVoltage(double voltage) {
        moteur.setVoltage(voltage);
    }

    public void stop() {
        setVoltage(0);
    }

    //Encodeur
    public void resetEncoder() {
        moteur.getEncoder().setPosition(0);
    }

    @Logged
    public double getPosition() {
        return moteur.getEncoder().getPosition();
    }

    public double getVelocity() {
        return moteur.getEncoder().getVelocity();
    }

    @Logged
    public boolean grimpeurHaut() {
        return grimpeurHaut;
    }

    //Commandes
    public Command monterPitCommand() {
        return Commands.runEnd(() -> setVoltage(2), this::stop, this);
    }

    public Command descendrePitCommand() {
        return Commands.runEnd(() -> setVoltage(-2), this::stop, this);
    }

    public Command goMaxHauteur() {
        return Commands.runOnce(() -> {
            grimpeurHaut = true;
        }).andThen(Commands.runEnd(() -> setVoltage(4), this::stop, this).until(() -> getPosition() > maxPosition));

    }

    public Command goMinHauteur() {
        return Commands.runOnce(() -> {
            grimpeurHaut = false;
        }).andThen(Commands.runEnd(() -> setVoltage(-7), this::stop, this).until(() -> getPosition() < 0.0));
    }

}
