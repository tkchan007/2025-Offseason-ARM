package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

/**
 * CALIBRATION COMMAND: Live PID tuning
 * Reads PID values from dashboard and commands arm to target position
 * Allows real-time PID adjustment while observing arm behavior
 * 
 * Usage: Set target angle, press button, adjust PID values on dashboard
 */
public class CAL_TunePID extends Command {
    private final ArmSubsystem armSubsystem;
    private double targetAngle;
    
    public CAL_TunePID(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
        // Read target from dashboard
        targetAngle = SmartDashboard.getNumber("CAL Target Angle", 45.0);
        armSubsystem.setTargetAngle(targetAngle);
        
        System.out.println(">>> CALIBRATION MODE: PID Tuning <<<");
        System.out.println("Target: " + targetAngle + "°");
        System.out.println("Adjust CAL kP, CAL kI, CAL kD on dashboard");
        
        SmartDashboard.putString("CAL Status", "PID TUNING - Target: " + targetAngle + "°");
    }
    
    @Override
    public void execute() {
        // This uses the live PID values being updated in ArmSubsystem.periodic()
        armSubsystem.updateMotionProfile();
        
        // Calculate and display error
        double error = targetAngle - armSubsystem.getArmAngle();
        SmartDashboard.putNumber("CAL Position Error", error);
        SmartDashboard.putBoolean("CAL At Target", Math.abs(error) < 2.0);
    }
    
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("CAL Status", "PID TUNING ENDED");
        System.out.println(">>> PID Tuning Ended <<<");
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until button released
    }
}