package frc.robot.commands;

import frc.robot.subsystems.drive.Drivetrain;

import java.net.http.HttpResponse.BodySubscriber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private final Drivetrain m_drivetrain;
    private final DoubleSupplier m_xSpeedSupplier;
    private final DoubleSupplier m_ySpeedSupplier;
    private final DoubleSupplier m_rotSupplier;
    private final BooleanSupplier m_SlowMode;

    public TeleopSwerve(Drivetrain drivetrain, DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier rotSupplier, BooleanSupplier SlowMode) {
        m_drivetrain = drivetrain;
        m_xSpeedSupplier = xSpeedSupplier;
        m_ySpeedSupplier = ySpeedSupplier;
        m_rotSupplier = rotSupplier;
        m_SlowMode = SlowMode;

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        if (m_SlowMode.getAsBoolean()) {
            double xSpeed = MathUtil.applyDeadband(m_xSpeedSupplier.getAsDouble(), 0.3) * .5;
            double ySpeed = -MathUtil.applyDeadband(m_ySpeedSupplier.getAsDouble(), 0.3) * .5;
            double rot = MathUtil.applyDeadband(m_rotSupplier.getAsDouble(), 0.3) * .5;
            m_drivetrain.drive(xSpeed, ySpeed, rot, true);
        } else {
            double xSpeed = MathUtil.applyDeadband(m_xSpeedSupplier.getAsDouble(), 0.3);
            double ySpeed = -MathUtil.applyDeadband(m_ySpeedSupplier.getAsDouble(), 0.3);
            double rot = MathUtil.applyDeadband(m_rotSupplier.getAsDouble(), 0.3);
            m_drivetrain.drive(xSpeed, ySpeed, rot, true);
        }
    }
}
