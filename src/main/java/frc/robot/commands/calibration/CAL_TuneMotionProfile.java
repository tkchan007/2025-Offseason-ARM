package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

/**
 * CALIBRATION COMMAND: Motion profile tuning
 * Tests different velocity and acceleration limits
 * Performs full-range motion to evaluate smoothness
 */
public class CAL_TuneMotionProfile extends Command {
    private final ArmSubsystem armSubsystem;
    private final double startAngle;
    private final double endAngle;
    private double startTime;
    
    public CAL_TuneMotionProfile(ArmSubsystem armSubsystem, double startAngle, double endAngle) {
        this.armSubsystem = armSubsystem;
        this.startAngle = startAngle;
        this.endAngle = endAngle;
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        armSubsystem.setTargetAngle(endAngle);
        
        System.out.println(">>> CALIBRATION MODE: Motion Profile Test <<<");
        System.out.println("Moving from " + startAngle + "° to " + endAngle + "°");
        
        SmartDashboard.putString("CAL Status", "MOTION PROFILE TEST");
        SmartDashboard.putNumber("CAL Test Start Angle", startAngle);
        SmartDashboard.putNumber("CAL Test End Angle", endAngle);
    }
    
    @Override
    public void execute() {
        armSubsystem.updateMotionProfile();
        
        double elapsed = (System.currentTimeMillis() - startTime) / 1000.0;
        SmartDashboard.putNumber("CAL Elapsed Time", elapsed);
        SmartDashboard.putNumber("CAL Current Velocity", armSubsystem.getArmVelocity());
    }
    
    @Override
    public void end(boolean interrupted) {
        double totalTime = (System.currentTimeMillis() - startTime) / 1000.0;
        SmartDashboard.putNumber("CAL Total Move Time", totalTime);
        SmartDashboard.putString("CAL Status", "MOTION TEST COMPLETE - Time: " + 
            String.format("%.2f", totalTime) + "s");
        System.out.println(">>> Motion Profile Test Complete: " + totalTime + "s <<<");
    }
    
    @Override
    public boolean isFinished() {
        return armSubsystem.atTargetPosition(2.0);
    }
}