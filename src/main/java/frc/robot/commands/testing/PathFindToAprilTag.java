package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class PathFindToAprilTag extends Command {
    @SuppressWarnings("unused")
    private final Vision vision;
    private final DrivetrainIO driveSubsystem;
    private Command cmd;

    public PathFindToAprilTag(Vision vision, DrivetrainIO driveSubsystem) {
        this.vision = vision;
        this.driveSubsystem = driveSubsystem;
        addRequirements(vision, driveSubsystem);
    }

    @Override
    public void initialize() {
        cmd = new PathFindToPose(driveSubsystem, () -> null); // removing for now
        CommandScheduler.getInstance().schedule(cmd);
    }

    @Override
    public boolean isFinished() {
        return cmd == null || cmd.isFinished();
    }
}
