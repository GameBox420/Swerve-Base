package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
    
    private final CANSparkMax MOTOR_TURN;
    private final CANSparkMax MOTOR_DRIVE;

    private final RelativeEncoder ENCODER_TURN;
    private final RelativeEncoder ENCODER_DRIVE;

    private final PIDController PID_TURNING;








}
