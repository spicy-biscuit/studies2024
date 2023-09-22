package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.control.LaundryStick;
import frc.robot.subsystems.LaundryArm;
import frc.robot.subsystems.LaundryDrive;

/**
 * An example of functional design: subsystems "pull" commands from suppliers.
 * This is the reverse of the common imperative style, where commands are
 * "pushed" to consumers.
 */
public class Robot extends TimedRobot {
    private final LaundryStick m_stick;
    private final LaundryDrive m_drive;
    private final LaundryArm m_arm;

    public Robot() {
        Joystick joystick = new Joystick(0);
        m_stick = new LaundryStick(joystick);

        Talon m_leftMotor = new Talon(0);
        Talon m_rightMotor = new Talon(1);
        m_leftMotor.setInverted(true);
        m_rightMotor.setInverted(false);
        DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
        m_drive = new LaundryDrive(m_stick::xSpeed1_1, m_stick::zSpeed1_1, m_robotDrive);

        CANSparkMax m_armMotor = new CANSparkMax(2, MotorType.kBrushless);
        // these constraints are effectively infinite.
        Constraints m_constraints = new Constraints(50, 20);
        ProfiledPIDController m_controller = new ProfiledPIDController(10, 0, 0, m_constraints);
        m_arm = new LaundryArm(m_stick::dump, m_controller, m_armMotor);
    }

    @Override
    public void teleopInit() {
        m_drive.enable();
        m_arm.enable();
    }

    @Override
    public void teleopExit() {
        m_drive.disable();
        m_arm.disable();
    }

    @Override
    public void robotPeriodic() {
        m_drive.periodic();
        m_arm.periodic();
    }
}
