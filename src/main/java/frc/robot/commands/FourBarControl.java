package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class FourBarControl extends ParallelCommandGroup{
    
    private double angle2Horizontal;

    public FourBarControl(double armControl, double wristControl, Arm arm, Wrist wrist){


        addCommands(Commands.runEnd(
            () -> {
                if(armControl>0.05 || armControl<-0.05) {
                    arm.setPower(armControl);
                }else{
                    arm.feedForward(arm.armPosition(), 0);
                }
            }, 
            () -> {}, 
            arm), Commands.runEnd(
            () -> {
                if(wristControl>0.05 || wristControl<-0.05){
                    wrist.powerWrist(wristControl);
                    angle2Horizontal = wrist.getWristAngle()+arm.armPosition();
                }else{
                    wrist.setGoal(angle2Horizontal-arm.armPosition());
                }
            },
            () -> {},
            wrist
            ));
    }
}
