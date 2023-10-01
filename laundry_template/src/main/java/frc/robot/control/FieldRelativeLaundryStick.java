package frc.robot.control;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;

public class FieldRelativeLaundryStick {

    private final Joystick m_stick;

    private final DoublePublisher yPub;
    private final DoublePublisher xPub;
    private final BooleanPublisher triggerPub;
    private final BooleanPublisher topPub;

    public FieldRelativeLaundryStick(Joystick stick) {
        m_stick = stick;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("FieldRelativeLaundryStick");
        yPub = table.getDoubleTopic("scaledX").publish();
        xPub = table.getDoubleTopic("scaledY").publish();
        triggerPub = table.getBooleanTopic("trigger").publish();
        topPub = table.getBooleanTopic("top").publish();
    }

    public boolean dump() {
        boolean dump = m_stick.getTrigger();
        triggerPub.set(dump);
        return dump;
    }

    public boolean reset() {
        boolean top = m_stick.getTop();
        topPub.set(top);
        return top;
    }

    // TODO: change this to meters/sec
    public double xSpeed1_1() {
        double scaledY = -0.8 * m_stick.getY();
        yPub.set(scaledY);
        return scaledY;
    }

    // TODO: change this to meters/sec
    public double ySpeed1_1() {
        double scaledX = -0.65 * m_stick.getX();
        xPub.set(scaledX);
        return scaledX;
    }
    
}
