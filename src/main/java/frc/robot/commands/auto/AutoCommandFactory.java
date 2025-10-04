package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driving.DriveToLocation;

// TODO
public class AutoCommandFactory {

    public static Command dropOneCenter() {
        return new SequentialCommandGroup(new DriveToLocation(null, null, null));
    }

}
