package frc.robot.subsystems;

import java.time.OffsetDateTime;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LaundryArm extends Subsystem {
    private ProfiledPIDController m_controller;
    private CANSparkMax m_motor;
    //private double kOffset;
   // private double kLimit;
    private double kGoal;
    private boolean kInverted;
    //private boolean zeroCalibrated = false;
    //private boolean limitCalibrated = false;
    private double m_output = 0;
    private boolean m_enabled = false;

    public LaundryArm(ProfiledPIDController controller, CANSparkMax motor, boolean inverted) {
        m_controller = controller;
        m_motor = motor;
        kInverted = inverted;
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setSmartCurrentLimit(10);
        m_motor.setIdleMode(IdleMode.kCoast);
    }

    public void zeroSet() {
        //kOffset = getAbsolute();
        //zeroCalibrated = true;
        m_motor.getEncoder().setPosition(0);
    }

    public void limitSet() {
       // kLimit = getAbsolute();
        // limitCalibrated = true;
    }

    public void enable() {
        m_enabled = true;
    }

    public void disable() {
        m_enabled = false;
    }

    public void setMotor() {
        if (m_enabled) {
            m_motor.set(m_output);
            System.out.println("motor set");
        }
        else {
            m_motor.set(0);
            m_motor.setIdleMode(IdleMode.kCoast);
            System.out.println("coast");
        }
    }

    public void setOutput(double output) {
        m_output = (kInverted ? -1 : 1) * output;
    }

    public double getAbsolute() {
        double absolute = m_motor.getEncoder().getPosition();
        double ret = absolute / 125;
        return (kInverted ? -1 : 1) * ret;
    }

    public double calculate(double goal) {
        return (kInverted ? -1 : 1) * m_controller.calculate(getAbsolute(), goal);
    }

    public void setDegrees(double goal) {
        kGoal = goal;
    }

    public void print() {
        System.out.println("rotations:" + getAbsolute());
    }

    @Override
    public void periodic() {
        // if (zeroCalibrated && limitCalibrated) {
        // System.out.println("goal: " + kGoal);
        // System.out.println("rotations:" + getAbsolute());
        setOutput(calculate(kGoal));
        setMotor();
        // } else {
        // setOutput(0);
        // }
    }
}
