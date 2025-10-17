package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Horizontal Arm Subsystem with Motion Profiling
 * 
 * ARM SPECIFICATIONS:
 * - Total length: 23.25" (22.25" from pivot + 1" before pivot)
 * - Gear ratio: 20:1
 * - Pivot location: 2.5" above robot top, right side
 * - Hook: 7" fixed perpendicular piece at end
 * - Material: Lightweight wood/plastic
 * 
 * POSITIONS:
 * - REST: 180° (horizontal, pointing back, on bumper)
 * - ENGAGED: 40° (above horizontal, pointing forward/up)
 * - Total rotation: 140° (smooth motion with no "smack")
 * 
 * CONTROL STRATEGY:
 * - Trapezoidal motion profiling for smooth acceleration/deceleration
 * - PID position control
 * - Gravity feedforward (kG) for holding against gravity
 */
public class HorizontalArmSubsystem extends SubsystemBase {
    
    // Hardware
    private final SparkMax armMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
    
    // Motion Profile
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;
    
    // Feedforward
    private final ArmFeedforward feedforward;
    
    // Constants - ARM GEOMETRY
    private static final double GEAR_RATIO = 20.0;
    private static final double ARM_LENGTH_INCHES = 22.25; // From pivot to end
    
    // Constants - POSITION TARGETS (in degrees)
    public static final double REST_ANGLE = 180.0;      // Horizontal, pointing back
    public static final double ENGAGED_ANGLE = 40.0;    // 40° above horizontal
    
    // Constants - MOTION PROFILE LIMITS
    // Start conservative, tune based on testing
    private static final double MAX_VELOCITY_DEG_PER_SEC = 120.0;  // Tune this!
    private static final double MAX_ACCELERATION_DEG_PER_SEC_SQ = 240.0;  // Tune this!
    
    // Constants - CONTROL GAINS (tune these on dashboard)
    private double kP = 0.02;    // Start small, increase until responsive
    private double kI = 0.0;     // Usually not needed
    private double kD = 0.0;     // Add if oscillation occurs
    private double kG = 0.05;    // Gravity compensation - tune this!
    
    // Constants - HARDWARE
    private static final int ARM_MOTOR_CAN_ID = 30;
    private static final double POSITION_TOLERANCE = 2.0; // degrees
    
    // State tracking
    private boolean isCalibrationMode = false;
    
    public HorizontalArmSubsystem() {
        // Initialize motor
        armMotor = new SparkMax(ARM_MOTOR_CAN_ID, MotorType.kBrushless);
        encoder = armMotor.getEncoder();
        pidController = armMotor.getClosedLoopController();
        
        // Configure motor
        configureMotor();
        
        // Initialize motion profile
        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                MAX_VELOCITY_DEG_PER_SEC,
                MAX_ACCELERATION_DEG_PER_SEC_SQ
            )
        );
        
        // Initialize feedforward (kS and kV typically small for arms)
        feedforward = new ArmFeedforward(0.0, kG, 0.0, 0.0);
        
        // Start at rest position
        setpoint = new TrapezoidProfile.State(REST_ANGLE, 0.0);
        goal = new TrapezoidProfile.State(REST_ANGLE, 0.0);
        
        // Dashboard values
        initializeDashboard();
    }
    
    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Motor configuration
        config.inverted(false);  // TODO: Check direction in testing
        config.idleMode(SparkMaxConfig.IdleMode.kBrake);
        config.smartCurrentLimit(30);  // Amps - tune based on stall current
        
        // Encoder configuration (NEO has 42 counts per revolution)
        // Position = motor rotations * gear ratio = arm degrees
        // 1 motor rotation = 360 / GEAR_RATIO degrees of arm movement
        double positionConversionFactor = 360.0 / GEAR_RATIO;  // degrees per motor rotation
        double velocityConversionFactor = positionConversionFactor / 60.0;  // degrees per second
        
        config.encoder
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(velocityConversionFactor);
        
        // PID configuration
        config.closedLoop
            .pid(kP, kI, kD)
            .outputRange(-1.0, 1.0);
        
        // Apply configuration
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    @Override
    public void periodic() {
        // Update PID gains from dashboard if in calibration mode
        if (isCalibrationMode) {
            updateGainsFromDashboard();
        }
        
        // Calculate next setpoint using motion profile
        setpoint = profile.calculate(0.02, setpoint, goal);  // 20ms loop time
        
        // Calculate feedforward based on current angle
        // Arm angle measured from horizontal (0° = horizontal)
        // Need to convert to angle from vertical for cosine calculation
        double armAngleFromHorizontal = setpoint.position;
        double gravityFF = calculateGravityFeedforward(armAngleFromHorizontal);
        
        // Send position + feedforward to motor controller
        pidController.setReference(
            setpoint.position,
            SparkMax.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            gravityFF
        );
        
        // Update telemetry
        updateTelemetry();
    }
    
    /**
     * Calculate gravity feedforward based on arm angle
     * Compensates for gravitational torque on the arm
     * 
     * For horizontal arm lifting up:
     * - At 180° (horizontal back): cos(180-90) = cos(90) = 0 (no gravity effect perpendicular)
     * - At 90° (pointing straight up): cos(0) = 1 (maximum support needed)
     * - At 40° (engaged): cos(140) = moderate support
     */
    private double calculateGravityFeedforward(double angleDegrees) {
        // Convert angle: horizontal = 180° in our system, vertical up = 90°
        // For gravity FF: 0° = horizontal, 90° = vertical up
        double angleFromHorizontal = 180.0 - angleDegrees;
        double angleRadians = Math.toRadians(angleFromHorizontal);
        return kG * Math.cos(angleRadians);
    }
    
    // ===== POSITION COMMANDS =====
    
    public void goToRest() {
        goal = new TrapezoidProfile.State(REST_ANGLE, 0.0);
        SmartDashboard.putString("Arm/Target", "REST");
    }
    
    public void goToEngaged() {
        goal = new TrapezoidProfile.State(ENGAGED_ANGLE, 0.0);
        SmartDashboard.putString("Arm/Target", "ENGAGED");
    }
    
    public boolean atGoal() {
        return Math.abs(setpoint.position - goal.position) < POSITION_TOLERANCE &&
               Math.abs(setpoint.velocity) < 5.0;  // Near zero velocity
    }
    
    // ===== CALIBRATION & TUNING =====
    
    public void enableCalibrationMode() {
        isCalibrationMode = true;
        SmartDashboard.putBoolean("Arm/Calibration Mode", true);
    }
    
    public void disableCalibrationMode() {
        isCalibrationMode = false;
        SmartDashboard.putBoolean("Arm/Calibration Mode", false);
    }
    
    public void resetEncoder() {
        encoder.setPosition(REST_ANGLE);  // Manually position at rest, then call this
    }
    
    private void updateGainsFromDashboard() {
        double newKP = SmartDashboard.getNumber("Arm/Tune/kP", kP);
        double newKI = SmartDashboard.getNumber("Arm/Tune/kI", kI);
        double newKD = SmartDashboard.getNumber("Arm/Tune/kD", kD);
        double newKG = SmartDashboard.getNumber("Arm/Tune/kG", kG);
        
        if (newKP != kP || newKI != kI || newKD != kD) {
            kP = newKP;
            kI = newKI;
            kD = newKD;
            pidController.setP(kP, ClosedLoopSlot.kSlot0);
            pidController.setI(kI, ClosedLoopSlot.kSlot0);
            pidController.setD(kD, ClosedLoopSlot.kSlot0);
        }
        
        if (newKG != kG) {
            kG = newKG;
        }
    }
    
    private void initializeDashboard() {
        SmartDashboard.putBoolean("Arm/Calibration Mode", false);
        SmartDashboard.putNumber("Arm/Tune/kP", kP);
        SmartDashboard.putNumber("Arm/Tune/kI", kI);
        SmartDashboard.putNumber("Arm/Tune/kD", kD);
        SmartDashboard.putNumber("Arm/Tune/kG", kG);
        SmartDashboard.putString("Arm/Target", "REST");
    }
    
    private void updateTelemetry() {
        SmartDashboard.putNumber("Arm/Current Angle", encoder.getPosition());
        SmartDashboard.putNumber("Arm/Current Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Arm/Setpoint Angle", setpoint.position);
        SmartDashboard.putNumber("Arm/Setpoint Velocity", setpoint.velocity);
        SmartDashboard.putNumber("Arm/Goal Angle", goal.position);
        SmartDashboard.putNumber("Arm/Position Error", goal.position - encoder.getPosition());
        SmartDashboard.putBoolean("Arm/At Goal", atGoal());
        SmartDashboard.putNumber("Arm/Motor Current", armMotor.getOutputCurrent());
        SmartDashboard.putNumber("Arm/Gravity FF", calculateGravityFeedforward(setpoint.position));
    }
    
    public void stop() {
        armMotor.stopMotor();
    }
    
    // ===== COMMAND FACTORIES (Modern WPILib approach) =====
    
    /**
     * Command to move arm to REST position (180°)
     * Returns when arm reaches target
     */
    public Command moveToRestCommand() {
        return runOnce(() -> goToRest())
            .andThen(run(() -> {}).until(this::atGoal))
            .withName("ArmToRest");
    }
    
    /**
     * Command to move arm to ENGAGED position (40°)
     * Returns when arm reaches target
     */
    public Command moveToEngagedCommand() {
        return runOnce(() -> goToEngaged())
            .andThen(run(() -> {}).until(this::atGoal))
            .withName("ArmToEngaged");
    }
    
    /**
     * Instant command to reset encoder
     * Use this when arm is manually positioned at REST
     */
    public Command resetEncoderCommand() {
        return runOnce(this::resetEncoder)
            .withName("ResetArmEncoder");
    }
    
    /**
     * Command to enable calibration mode
     */
    public Command enableCalibrationCommand() {
        return runOnce(this::enableCalibrationMode)
            .withName("EnableCalibration");
    }
    
    /**
     * Command to disable calibration mode
     */
    public Command disableCalibrationCommand() {
        return runOnce(this::disableCalibrationMode)
            .withName("DisableCalibration");
    }
}
