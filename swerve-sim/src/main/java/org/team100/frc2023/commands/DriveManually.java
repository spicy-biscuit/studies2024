package org.team100.frc2023.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

public class DriveManually extends Command {
    private static final boolean fieldRelative = true;
    private final Drivetrain m_swerve;
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final DoubleSupplier rotSpeed;

    public DriveManually(Drivetrain m_swerve, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed) {
        this.m_swerve = m_swerve;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;
        addRequirements(m_swerve);
    }

    @Override
    public void execute() {
        final var x = xSpeed.getAsDouble() * Drivetrain.kMaxSpeed;
        final var y = ySpeed.getAsDouble() * Drivetrain.kMaxSpeed;
        final var rot = rotSpeed.getAsDouble() * Drivetrain.kMaxAngularSpeed;
        m_swerve.drive(x, y, rot, fieldRelative);
    }
}
