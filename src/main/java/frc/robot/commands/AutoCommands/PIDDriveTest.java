package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DrivetrainCommands.PIDDrive;
import frc.robot.subsystems.Drivetrain;

public class PIDDriveTest extends SequentialCommandGroup{
    private Drivetrain drivetrain;
    public PIDDriveTest(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addCommands(new PIDDrive(Units.inchesToMeters(60), Units.inchesToMeters(60), drivetrain));
    }
}
