package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

/**
 * CALIBRATION COMMAND: Disturbance rejection test
 * Holds arm at target position - manually push arm to test recovery
 */
public class CAL_DisturbanceTest extends Command {
    private final ArmSubsystem armSubsystem;
    private final double holdAngle;
    private double maxDisturbance = 0.0;
    
    public CAL_DisturbanceTest(ArmSubsystem armSubsystem, double holdAngle) {
        this.armSubsystem = armSubsystem;
        this.holdAngle = holdAngle;
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
        armSubsystem.setTargetAngle(holdAngle);
        maxDisturbance = 0.0;
        
        System.out.println(">>> CALIBRATION MODE: Disturbance Test <<<");
        System.out.println("Holding at " + holdAngle + "°");
        System.out.println("Manually push arm and observe recovery");
        
        SmartDashboard.putString("CAL Status", "DISTURBANCE TEST - Push arm gently");
    }
    
    @Override
    public void execute() {
        armSubsystem.updateMotionProfile();
        
        double error = Math.abs(holdAngle - armSubsystem.getArmAngle());
        if (error > maxDisturbance) {
            maxDisturbance = error;
        }
        
        SmartDashboard.putNumber("CAL Max Disturbance", maxDisturbance);
        SmartDashboard.putNumber("CAL Current Error", error);
    }
    
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("CAL Status", "DISTURBANCE TEST COMPLETE");
        System.out.println(">>> Disturbance Test Complete <<<");
        System.out.println("Max disturbance: " + maxDisturbance + "°");
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until button released
    }
}