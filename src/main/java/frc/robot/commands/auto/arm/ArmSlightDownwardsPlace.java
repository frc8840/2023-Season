package frc.robot.commands.auto.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmSlightDownwardsPlace extends CommandBase {
    //Positive is back towards starting position, negative is away from starting position

    /**                                      /\ - (elbow)
     *                                   /[][][]|||||
     *               /\ - (base)   /][][/    \/ + (elbow)
     *          [][][][][][][]•[][/          
     *        / |    \/ + (base)
     *      /   |
     *    /     |
     *  /       |
     * ==========|
     */
    public static final double speed = 0.2; 
    public static final double time = 300;

    public ArmSlightDownwardsPlace() {
        
    }

    private double startTime = 0;

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() - startTime < time) {
            ArmSubsystem.getInstance().elbowOpenLoop(speed);
        } else {
            ArmSubsystem.getInstance().stop();
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= time;
    }
}
