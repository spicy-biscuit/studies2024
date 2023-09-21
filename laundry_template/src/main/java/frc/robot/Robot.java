// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.subsystems.LaundryArm;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final Talon m_leftMotor = new Talon(0);
  private final Talon m_rightMotor = new Talon(1);
  private final CANSparkMax m_armMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Constraints m_constraints = new Constraints(50, 20);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(10, 0, 0, m_constraints);
  private final LaundryArm m_arm = new LaundryArm(m_controller, m_armMotor, false);
  private final Joystick m_stick = new Joystick(0);
  private final EventLoop m_loop = new EventLoop();
  private final double up_degrees = 0;
  private final double down_degrees = 25;

  @Override
  public void robotInit() {
    m_leftMotor.setInverted(true);
    m_rightMotor.setInverted(false);
    // lower left on top of stick
    BooleanEvent topButton1 = m_stick.button(3, m_loop);
    // lower right on top of stick
    BooleanEvent topButton2 = m_stick.button(4, m_loop);
    topButton1.ifHigh(() -> {
      m_arm.disable();
      m_arm.zeroSet();
      System.out.println("disable");
    });
    topButton1.negate().ifHigh(() -> {
      m_arm.enable();
      System.out.println("enable");
    });
    topButton2.ifHigh(() -> m_arm.limitSet());
  }

  @Override
  public void teleopPeriodic() {

    m_robotDrive.arcadeDrive(-.8 * m_stick.getY(), -.65 * m_stick.getX(), false);
    if (m_stick.getRawButton(1)) {
      // go out
      m_arm.setDegrees(-.25);
    } else {
      // go in
      m_arm.setDegrees(-.01);
    }
    m_loop.poll();
    m_arm.periodic();
  }

  @Override
  public void robotPeriodic() {
    m_arm.print();
  }
}
