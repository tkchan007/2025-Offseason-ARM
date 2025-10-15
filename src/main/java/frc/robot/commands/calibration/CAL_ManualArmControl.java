package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

/**
 * CALIBRATION COMMAND: Manual arm control for positioning
 * Allows slow manual movement of arm for setup and positioning
 * 
 * Usage: Use left/right triggers or specified axis for control
 */
public class CAL_ManualArmControl extends Command {
    private final ArmSubsystem armSubsystem;
    private final double speed;
    
    public CAL_ManualArmControl(ArmSubsystem armSubsystem, double speed) {
        this.armSubsystem = armSubsystem;
        this.speed = speed;
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
        SmartDashboard.putString("CAL Status", "MANUAL CONTROL ACTIVE");
    }
    
    @Override
    public void execute() {
        armSubsystem.setManualSpeed(speed);
        SmartDashboard.putNumber("CAL Current Angle", armSubsystem.getArmAngle());
    }
    
    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
        SmartDashboard.putString("CAL Status", "MANUAL CONTROL ENDED");
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}