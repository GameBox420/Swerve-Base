package frc.robot.subsystems;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
//pheonix
import com.ctre.phoenix.sensors.CANCoder;
//rev
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//wpi
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//robot
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
    
    private final CANSparkMax MOTOR_TURN;
    private final CANSparkMax MOTOR_DRIVE;

    private final RelativeEncoder ENCODER_TURN;
    private final RelativeEncoder ENCODER_DRIVE;

    private final PIDController PID_TURNING;

    private final CANCoder ENCODER_ABSOLUTE;


    public SwerveModule(
        int ID_MOTOR_DRIVE,
        boolean REVERSE_MOTOR_DRIVE,
        int ID_MOTOR_TURN,
        boolean REVERSE_MOTOR_TURN,
        int ID_ENCODER_ABSOLUTE,
        boolean REVERSE_ENCODER_ABSOLUTE,
        double OFFSET_ENCODER_ABSOLUTE
    ) {

        //init the drive motor and encoder
        this.MOTOR_DRIVE =new CANSparkMax(ID_MOTOR_DRIVE, MotorType.kBrushless);
        MOTOR_DRIVE.setInverted(REVERSE_MOTOR_DRIVE);

        this.ENCODER_DRIVE = MOTOR_DRIVE.getEncoder();
        ENCODER_DRIVE.setPositionConversionFactor(ModuleConstants.DriveEncoderRot2Meter);


        //init the turning motor and encoder
        this.MOTOR_TURN = new CANSparkMax(ID_MOTOR_TURN, MotorType.kBrushless);
        MOTOR_TURN.setInverted(REVERSE_MOTOR_TURN);

        this.ENCODER_TURN = MOTOR_TURN.getEncoder();
        ENCODER_TURN.setPositionConversionFactor(ModuleConstants.TurningEncoderRot2Rad);

        //init absolute encoder
        this.ENCODER_ABSOLUTE = new CANCoder(ID_ENCODER_ABSOLUTE);
        ENCODER_ABSOLUTE.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        //init PID for turning
        this.PID_TURNING = new PIDController(Constants.ModuleConstants.TURNING_Proportional,Constants.ModuleConstants.TURNING_Integral ,Constants.ModuleConstants.TURNING_Derivitive);
        PID_TURNING.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void resetEncoders() {
        ENCODER_DRIVE.setPosition(0);
        ENCODER_TURN.setPosition(getAbsoluteEncoderRad());
    }

    public void setDesiredState(SwerveModuleState state) {

    }

    public void stop() {

    }

    public double getDrivePosition(){
        return ENCODER_DRIVE.getPosition();
    }

    public double getTurningPosition(){
        return ENCODER_TURN.getPosition();
    }

    public double getDriveVelocity(){
        return ENCODER_DRIVE.getVelocity();
    }

    public double getTurningVelocity(){
        return ENCODER_DRIVE.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        return Math.toRadians(ENCODER_ABSOLUTE.getAbsolutePosition());
    }

}
