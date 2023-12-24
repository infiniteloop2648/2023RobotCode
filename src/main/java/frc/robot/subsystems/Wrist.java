package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Wrist extends ProfiledPIDSubsystem{
    
    private MotorController wristMotor;
    private Encoder wristEncoder;

    private ArmFeedforward wristFeedForward;

    private DigitalInput wristSwitch;

    private DoubleSupplier armPositionSupplier;
    private DoubleSupplier armSetpoint;

    public Wrist(DoubleSupplier armPositionSupplier, DoubleSupplier armSetpoint){
        super(new ProfiledPIDController(Constants.kWristP, Constants.kWristI, Constants.kWristD,
        new TrapezoidProfile.Constraints(Constants.kWristMaxVelocityPerSecond, Constants.kWristMaxAccelerationPerSecond2)));
        getController().setTolerance(Units.degreesToRadians(2));
        wristMotor = new WPI_VictorSPX(Constants.kWrist);

        wristFeedForward = new ArmFeedforward(Constants.kWristFeedS, Constants.kWristFeedG, Constants.kArmFeedV, Constants.kWristFeedA);

        wristEncoder = new Encoder(Constants.kSourceChannelWristEncoder1, Constants.kSourceChannelWristEncoder2);
        wristEncoder.setDistancePerPulse(Constants.kWristEncoderMultiplier);
        wristEncoder.setSamplesToAverage(10);

        wristSwitch = new DigitalInput(Constants.kSourceChannelWristSwitch);

        this.armPositionSupplier = armPositionSupplier;
        this.armSetpoint = armSetpoint;
    }

    @Override
    public void periodic(){
        super.periodic();
        
        getController().setGoal(getController().getSetpoint());

        /*if(!wristSwitch.get()){
            wristEncoder.reset();
        }*/
        
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        wristMotor.setVoltage(-(output + wristFeedForward.calculate(setpoint.position, setpoint.velocity)));
    }

    @Override
    protected double getMeasurement() {

        return wristEncoder.getDistance()+Constants.kWristStowedAngle+armPositionSupplier.getAsDouble();
    }

    public void powerWrist(double throttle){
        if(!wristSwitch.get()){
            System.out.println("manual control");
            wristMotor.set(MathUtil.clamp(throttle, 0, 1));
        }else{
            wristMotor.set(throttle);
        }
         
    }

    public void voltsWrist(double throttle){
        wristMotor.setVoltage(throttle);
    }

    public boolean atSetpoint(){
        return getController().atSetpoint();
    }

    public double getWristAngle(){
        return Units.radiansToDegrees(wristEncoder.getDistance()+Constants.kWristStowedAngle+armPositionSupplier.getAsDouble());
    }

    public double getMotorSet() {
        return wristMotor.get();
    }

}
