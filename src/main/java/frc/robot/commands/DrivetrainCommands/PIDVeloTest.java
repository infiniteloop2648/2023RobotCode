package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class PIDVeloTest extends CommandBase{
    
    
    private Drivetrain drivetrain;

    public PIDVeloTest( Drivetrain drivetrain){
        this.drivetrain = drivetrain;

        this.addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.PIDVeloDrive(0.5, 0.5);
    }
}
