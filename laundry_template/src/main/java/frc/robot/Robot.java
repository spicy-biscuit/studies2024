package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.subsystems.LaundryArm;

public class Robot extends TimedRobot {
    private final Talon m_leftMotor = new Talon(0);
    private final Talon m_rightMotor = new Talon(1);
    private final CANSparkMax m_armMotor = new CANSparkMax(2, MotorType.kBrushless);
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    private final Constraints m_constraints = new Constraints(50, 20);
    private final ProfiledPIDController m_controller = new ProfiledPIDController(10, 0, 0, m_constraints);
    private final LaundryArm m_arm = new LaundryArm(m_controller, m_armMotor);
    private final Joystick m_stick = new Joystick(0);

    @Override
    public void robotInit() {
        m_leftMotor.setInverted(true);
        m_rightMotor.setInverted(false);
    }

    @Override
    public void teleopInit() {
        m_arm.enable();
    }

    @Override
    public void teleopPeriodic() {
        m_robotDrive.arcadeDrive(-0.8 * m_stick.getY(), -0.65 * m_stick.getX(), false);
        if (m_stick.getTrigger()) {
            m_arm.dump();
        } else {
            m_arm.level();
        }
    }

    @Override
    public void teleopExit() {
        m_arm.disable();
    }

    @Override
    public void robotPeriodic() {
        m_arm.periodic();
    }
}
