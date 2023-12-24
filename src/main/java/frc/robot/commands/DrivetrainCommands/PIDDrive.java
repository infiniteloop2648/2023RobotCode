package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PIDDrive extends CommandBase{
    
    private Drivetrain drivetrain;
    private double leftSetPoint;
    private double rightSetPoint;

    public PIDDrive(double leftSetPoint, double rightSetPoint, Drivetrain drivetrain){
        this.leftSetPoint = leftSetPoint;
        this.rightSetPoint = rightSetPoint;
        this.drivetrain = drivetrain;
        this.addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.resetEncoders();
    }

    @Override
    public void execute(){
        drivetrain.pidDrive(leftSetPoint, rightSetPoint);
    }

    @Override
    public boolean isFinished(){
        return drivetrain.pidReached();
    }
    

    
}
