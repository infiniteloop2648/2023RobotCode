package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DrivetrainCommands.TankDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Wrist;

public class DriveBack extends SequentialCommandGroup{

    public DriveBack(Drivetrain drivetrain, Arm arm, Wrist wrist, Manipulator manipulator){
        addCommands(
            //Commands.runOnce(() -> arm.setPower(0.75), arm).withTimeout(1),
            new TankDrive(drivetrain,- 0.4,- 0.4).withTimeout(2)
            //Commands.runOnce(() -> wrist.powerWrist(1), wrist).withTimeout(1), 
            //new OpenManipulator(manipulator),
            //Commands.parallel(Commands.runOnce(() -> arm.setPower(-0.75), arm).withTimeout(1), Commands.runOnce(() -> wrist.powerWrist(-1), wrist).withTimeout(1), new TankDrive(drivetrain, 0.75, 0.75).withTimeout(3))
        );
    }
}
