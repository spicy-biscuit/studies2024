package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class FieldRelativeLaundryDrive implements LaundryDrive {
    private final DoubleSupplier m_xSpeed1_1;
    private final DoubleSupplier m_ySpeed1_1;
    private final DifferentialDrive m_drive;

    private final DoublePublisher xSpeedPub;
    private final DoublePublisher ySpeedPub;

    private boolean m_enabled;

    /**
     * @param xSpeed1_1 supplies desired field x speed in [-1,1] interval.
     *                  TODO: speed in meters per second.
     * @param ySpeed1_1 supplies desired field y rate in [-1,1] interval.
     *                  TODO: rotation rate in rad/s
     * @param drive     provides "arcade" mode
     */
    public FieldRelativeLaundryDrive(
            DoubleSupplier xSpeed1_1,
            DoubleSupplier ySpeed1_1,
            DifferentialDrive drive) {
        m_xSpeed1_1 = xSpeed1_1;
        m_ySpeed1_1 = ySpeed1_1;
        m_drive = drive;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("drive");
        xSpeedPub = table.getDoubleTopic("xSpeed1_1").publish();
        ySpeedPub = table.getDoubleTopic("ySpeed1-1").publish();
    }

    @Override
    public void enable() {
        m_enabled = true;
    }

    @Override
    public void disable() {
        m_enabled = false;
        m_drive.stopMotor();
    }

    @Override
    public void periodic() {
        double xSpeed1_1 = m_xSpeed1_1.getAsDouble();
        xSpeedPub.set(xSpeed1_1);

        double ySpeed1_1 = m_ySpeed1_1.getAsDouble();
        ySpeedPub.set(ySpeed1_1);

        if (m_enabled) {
            m_drive.arcadeDrive(xSpeed1_1, ySpeed1_1, false);
        }
    }
}
