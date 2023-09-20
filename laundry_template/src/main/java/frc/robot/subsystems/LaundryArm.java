package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LaundryArm extends Subsystem {
    private ProfiledPIDController m_controller;
    private CANSparkMax m_motor;
    private double kOffset;
    private double kLimit;
    private double kGoal;
    private boolean kInverted;
    private boolean zeroCalibrated = false;
    private boolean limitCalibrated = false;

    public LaundryArm(ProfiledPIDController controller, CANSparkMax motor, boolean inverted) {
        m_controller = controller;
        m_motor = motor;
        kInverted = inverted;
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setSmartCurrentLimit(1);
        m_motor.setIdleMode(IdleMode.kCoast);
    }

    public void zeroSet() {
        kOffset = getAbsolute();
        zeroCalibrated = true;
    }

    public void limitSet() {
        kLimit = getAbsolute();
        limitCalibrated = true;
    }

    public void setOutput(double output) {
        m_motor.set((kInverted ? -1 : 1) * output);
    }

    public double getAbsolute() {
        double absolute = m_motor.getEncoder().getPosition();
        double ret = (absolute - Math.floor(absolute))/25;
        return (kInverted ? -1 : 1) * ret;
    }

    public double calculate(double goal) {
        return (kInverted ? -1 : 1) * m_controller.calculate(getAbsolute(), goal);
    }

    public void setDegrees(double goal) {
        kGoal = 25 * MathUtil.clamp((goal / 360) + kOffset, kOffset, kLimit);
    }

    @Override
    public void periodic() {
        if (zeroCalibrated && limitCalibrated) {
            setOutput(calculate(kGoal));
        } else {
            setOutput(0);
        }
    }
}
