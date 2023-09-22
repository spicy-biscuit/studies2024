package frc.robot;

import org.team100.lib.sensors.LSM6DSOX_I2C;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.control.LaundryStick;
import frc.robot.subsystems.DirectLaundryDrive;
import frc.robot.subsystems.LaundryArm;
import frc.robot.subsystems.LaundryDrive;
import frc.robot.subsystems.StabilizedLaundryDrive;

/**
 * An example of functional design: subsystems "pull" commands from suppliers.
 * This is the reverse of the common imperative style, where commands are
 * "pushed" to consumers.
 */
public class Robot extends TimedRobot {
    /** To try yaw stabilization, make this true. */
    private static final boolean kStabilize = false;

    private final LaundryStick m_stick;
    private final LaundryDrive m_drive;
    private final LaundryArm m_arm;

    public Robot() {
        Joystick joystick = new Joystick(0);
        m_stick = new LaundryStick(joystick);

        Talon leftMotor = new Talon(0);
        Talon rightMotor = new Talon(1);
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

        if (kStabilize) {
            LSM6DSOX_I2C gyro = new LSM6DSOX_I2C();
            // TODO: tune yaw stabilizer PID
            PIDController yawController = new PIDController(1, 0, 0);
            m_drive = new StabilizedLaundryDrive(
                    gyro,
                    m_stick::xSpeed1_1,
                    m_stick::zSpeed1_1,
                    yawController,
                    drive);
        } else {
            m_drive = new DirectLaundryDrive(
                    m_stick::xSpeed1_1,
                    m_stick::zSpeed1_1,
                    drive);

        }
        CANSparkMax armMotor = new CANSparkMax(2, MotorType.kBrushless);
        ProfiledPIDController armController = new ProfiledPIDController(
                10, // P
                0, // I
                0, // D
                new Constraints(
                        50, // max velocity (infinite)
                        20)); // max accel (infinite)
        m_arm = new LaundryArm(m_stick::dump, armController, armMotor);
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
