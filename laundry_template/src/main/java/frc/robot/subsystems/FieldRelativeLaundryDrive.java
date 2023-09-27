package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.team100.lib.sensors.AHRS;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class FieldRelativeLaundryDrive implements LaundryDrive {
    public static class Control {
        public final double x;
        public final double y;

        public Control(double x, double y) {
            this.x = x;
            this.y = y;
        }

        /**
         * x and y refer to field coords from driver perspective, so x is straight
         * ahead, y is to the left
         */
        public static Control rotate(double headingRad, double x, double y) {
            double cos = Math.cos(headingRad);
            double sin = Math.sin(headingRad);
            double fwd = x * cos + y * sin;
            double rot = -x * sin + y * cos;
            return new Control(fwd, rot);
        }
    }

    private final AHRS m_ahrs;
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
            AHRS ahrs,
            DoubleSupplier xSpeed1_1,
            DoubleSupplier ySpeed1_1,
            DifferentialDrive drive) {
        m_ahrs = ahrs;
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
        m_ahrs.update();
        double xSpeed1_1 = m_xSpeed1_1.getAsDouble();
        xSpeedPub.set(xSpeed1_1);

        double ySpeed1_1 = m_ySpeed1_1.getAsDouble();
        ySpeedPub.set(ySpeed1_1);

        if (m_enabled) {
            Control control = Control.rotate(m_ahrs.getHeadingNWURad(), xSpeed1_1, ySpeed1_1);
            m_drive.arcadeDrive(control.x, control.y, false);
        }
    }
}
