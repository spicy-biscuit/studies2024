// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final Servo gripper;
  private final Servo claw;
  private final Servo wrist;
  private final Servo elbow;
  private final Servo servo5;
  private final Servo swivel;
  public double distance = 0;
  public double height = 12.875;
  public double q2 = 1;
  public double q1 = 0.5;

  public ExampleSubsystem() {
    gripper = new Servo(9);
    claw = new Servo(8);
    wrist = new Servo(7);
    elbow = new Servo(6);
    servo5 = new Servo(5);
    swivel = new Servo(4);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand(double val) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          gripper.set(val);
          claw.set(val);
          wrist.set(val);
          elbow.set(val);
          servo5.set(val);
          swivel.set(val);
          /* one-time action goes here */
        });
  }

  public CommandBase swivelMove(double val) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          swivel.set(swivel.get() + val);
          /* one-time action goes here */
        });
  }
  public CommandBase elbowMove(double val) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          elbow.set(elbow.get() + val);
          /* one-time action goes here */
        });
  }
  public CommandBase servoMove(double val) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          servo5.set(servo5.get() + val);
          /* one-time action goes here */
        });
  }
  public CommandBase wristMove(double val) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          wrist.set(wrist.get()+val);
  
          /* one-time action goes here */
        });
  }
  public CommandBase gripperMove(double val) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          gripper.set(gripper.get()+val);
          /* one-time action goes here */
        });
  }

  public CommandBase clawMove(double val) {
    return run(
        () -> {
          claw.set(claw.get()+val);
        });
  }

  public CommandBase update() {
    return run(
        () -> {
          q1 = -Math.acos((height*height+distance*distance-84.203125)/(81.5625));
          q2 = Math.atan(height/distance) + (Math.atan(-17.25*Math.sin(q1))/(5.625+7.25*Math.cos(q1)));
          
          elbow.set(q1/Math.PI);
          wrist.set(1-q2/Math.PI);
        });
  }

  public CommandBase xMove(double val) {
    return run(
        () -> {
          distance += val;
        });
  }
  
  public CommandBase yMove(double val) {
    return run(
        () -> {
          height += val;
        });
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
