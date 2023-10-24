package frc.robot;

import org.team100.telemetry.Telemetry;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private final Telemetry t = Telemetry.get();

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        long now = RobotController.getFPGATime();
        t.log("/time", now);
    }
}
