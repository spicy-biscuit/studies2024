package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LaundryArm extends Subsystem {
    private static final int kGearRatio = 125;
    private final ProfiledPIDController m_controller;
    private final CANSparkMax m_motor;

    private final DoublePublisher goalPub;
    private final DoublePublisher measurementPub;
    private final DoublePublisher outputPub;

    private double m_goalTurns;
    private boolean m_enabled;

    public LaundryArm(ProfiledPIDController controller, CANSparkMax motor) {
        m_controller = controller;
        m_motor = motor;
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setSmartCurrentLimit(10);
        m_motor.setIdleMode(IdleMode.kCoast);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("arm");
        goalPub = table.getDoubleTopic("goal").publish();
        measurementPub = table.getDoubleTopic("measurement").publish();
        outputPub = table.getDoubleTopic("output").publish();

        m_goalTurns = 0;
        m_enabled = false;
    }

    public void enable() {
        m_motor.getEncoder().setPosition(0);
        m_enabled = true;
    }

    public void disable() {
        m_enabled = false;
        m_motor.set(0);
        outputPub.set(0);
    }

    public void level() {
        setGoalTurns(0);
    }

    public void dump() {
        setGoalTurns(-0.25);
    }

    public void setGoalTurns(double goalTurns) {
        m_goalTurns = goalTurns;
        goalPub.set(m_goalTurns);
    }

    public double getMeasurementTurns() {
        double absolute = m_motor.getEncoder().getPosition();
        double turns = absolute / kGearRatio;
        measurementPub.set(turns);
        return turns;
    }

    @Override
    public void periodic() {
        double measurementTurns = getMeasurementTurns();
        double output = m_controller.calculate(measurementTurns, m_goalTurns);
        if (m_enabled) {
            m_motor.set(output);
            outputPub.set(output);
        }
    }
}
