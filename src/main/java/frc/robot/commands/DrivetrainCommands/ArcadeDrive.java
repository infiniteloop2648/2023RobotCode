package frc.robot.commands.DrivetrainCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
    private Drivetrain drivetrain;
    private Supplier<Double> xSpeed;
    private Supplier<Double> zRotation;

    public ArcadeDrive(Supplier<Double> xSpeed, Supplier<Double> zRotation, Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        this.xSpeed = xSpeed;
        this.zRotation = zRotation;
        this.addRequirements(drivetrain);
    }

    // arcade drive from double input
    @Override
    public void execute(){
        drivetrain.arcadeDrive(xSpeed.get(), zRotation.get());
    }
}