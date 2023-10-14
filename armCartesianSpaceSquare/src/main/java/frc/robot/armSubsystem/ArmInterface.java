package frc.robot.armSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.armMotion.ArmAngles;

public interface ArmInterface {

    Subsystem subsystem();

    boolean getCubeMode();

    void setCubeMode(boolean b);

    void setReference(ArmAngles reference);

    ArmAngles getMeasurement();

    void setControlNormal();

    void setControlSafe();

    void close();

    void setUpperSpeed(double x);

    void setLowerSpeed(double x);

    void setDefaultCommand(Command command);
}
