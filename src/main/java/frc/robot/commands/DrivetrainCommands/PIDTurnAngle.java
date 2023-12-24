package frc.robot.commands.DrivetrainCommands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;

public class PIDTurnAngle extends PIDCommand {
    private Drivetrain drivetrain;
    public PIDTurnAngle(double angleTarget, Drivetrain drivetrain){
        super(new PIDController(Constants.kTurnAngleP, Constants.kTurnAngleI, Constants.kTurnAngleD),
        drivetrain::getGyroAngle,
        angleTarget,
        output -> drivetrain.outputVolts(MathUtil.clamp(output, -7, 7), -MathUtil.clamp(output, -7, 7)),
        drivetrain);

        this.drivetrain = drivetrain;
        getController().setTolerance(2);

        this.addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.resetOdometry(new Pose2d());
    }




}
