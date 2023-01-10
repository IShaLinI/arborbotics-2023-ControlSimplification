package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase{

    private final Spark mLeftClaw;
    private final Spark mRightClaw;

    public Claw() {
        
        mLeftClaw = new Spark(Constants.CAN.kLeftClaw);
        mRightClaw = new Spark(Constants.CAN.kRightClaw);

        //Add pistons

    }

    public void intake() {

        mLeftClaw.set(0.7);
        mRightClaw.set(-0.7);
        
    }
    
}

