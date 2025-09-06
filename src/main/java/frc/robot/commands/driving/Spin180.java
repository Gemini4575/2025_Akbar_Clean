package frc.robot.commands.driving;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class Spin180 extends Command {
    DrivetrainIO d;
    boolean isFinished;

    private ProfiledPIDController rotation = new ProfiledPIDController(
            0.007,
            0,
            0,
            new TrapezoidProfile.Constraints(2, 2));

    public Spin180(DrivetrainIO d) {
        this.d = d;
    }

    @Override
    public void initialize() {
        rotation.enableContinuousInput(-180, 180);
        isFinished = false;
    }

    double startAngle = 0.0;
    double Rotate_Rot = 0.0;
    boolean first = true;

    private void first() {
        if (first) {
            startAngle = d.getAngle();
            first = false;
        }
    }

    public void end() {
        first = true;
    }

    // This is origanly with -180 to 180 bounds but we found that we ran into a
    // problem when it would flip and the robot would ocsolate due to the signum
    // calculation so we just made it contiues and just scalled down the values we
    // gave the PID loop
    public boolean rotate(Rotation2d targetAngle) {
        first();
        @SuppressWarnings("static-access")
        Rotation2d currentAngle = new Rotation2d().fromDegrees(d.getAngle());
        @SuppressWarnings("static-access")
        double distance_to_target = targetAngle.minus(new Rotation2d().fromDegrees(startAngle).minus(currentAngle))
                .getDegrees();
        SmartDashboard.putNumber("[DriveTrain]Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("[DriveTrain]Start Angle", startAngle);
        SmartDashboard.putNumber("[DriveTrain]currentAngle", currentAngle.getDegrees());
        SmartDashboard.putNumber("[DriveTrain]distance_to_target", distance_to_target);
        if (Math.abs(distance_to_target) < 10.0) {
            Rotate_Rot = 0.0;
            first = true;
        }
        Rotate_Rot = Math.signum(distance_to_target)
                * rotation.calculate((d.getAngle() - 180) - d.getAngle(), targetAngle.getDegrees());
        d.Rotate_Rot(Rotate_Rot);
        return Math.abs(distance_to_target) < 10.0;
    }

    @SuppressWarnings("static-access")
    @Override
    public void execute() {
        isFinished = rotate(new Rotation2d().fromDegrees(180));
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean ds) {
        if (ds) {
            end();
        }
    }
}
