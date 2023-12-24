package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

// control drivetrain sides independantly
public class TankDrive extends CommandBase{
    private Drivetrain drivetrain;
    private double leftSpeed;
    private double rightSpeed;

    public TankDrive(Drivetrain drivetrain, double leftSpeed, double rightSpeed){
        this.drivetrain = drivetrain;
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        this.addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        drivetrain.tankDrive(leftSpeed, rightSpeed);
    }
}
