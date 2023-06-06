// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveTrainCommand extends CommandBase {
  /** Creates a new DriveTrainCommand. */
  private final DrivetrainSubsystem drivetrain;
  private final Supplier<Double> speedFunction, turnFunction;
  private final Supplier<Boolean> slowMode;
  private Timer timer;
  private double time = -1.0;


  public DriveTrainCommand(DrivetrainSubsystem dt, Supplier<Double> sFunction, Supplier<Double> tFunction, Supplier<Boolean> sMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = dt;
    speedFunction = sFunction;
    turnFunction = tFunction;
    slowMode = sMode;
    timer = new Timer();
    addRequirements(drivetrain);
  }

  public DriveTrainCommand(DrivetrainSubsystem dt, Supplier<Double> sFunction, Supplier<Double> tFunction, Supplier<Boolean> sMode, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = dt;
    speedFunction = sFunction;
    turnFunction = tFunction;
    slowMode = sMode;
    timer = new Timer();
    this.time = time;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = speedFunction.get();
    double turn = turnFunction.get();
    boolean slow = slowMode.get();

    if(slow){
      speed *= 0.5;
      turn *= 0.5;
    }
    drivetrain.arcadeDrive(speed, turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Drivetrain Stopped.");
    //look
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time == -1 ? false : timer.get() >= time;
    //Basically, if we don't pass a time in, time will be -1. So, the method will return false
  }
}
