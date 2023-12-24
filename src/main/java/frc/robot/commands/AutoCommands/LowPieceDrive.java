package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DrivetrainCommands.TankDrive;
import frc.robot.commands.ManipulatorCommands.OpenManipulator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Wrist;

public class LowPieceDrive extends SequentialCommandGroup{

    public LowPieceDrive(Drivetrain drivetrain, Arm arm, Wrist wrist, Manipulator manipulator){

        addCommands(
            Commands.runEnd(
                () -> wrist.powerWrist(0.5), 
                () -> {}, 
                wrist).withTimeout(0.5),
            Commands.runOnce(() -> manipulator.OpenManipulator(), manipulator),
            new WaitCommand(1),
            new TankDrive(drivetrain,- 0.4,- 0.4).withTimeout(6)
        );
    }
    
}
