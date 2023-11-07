package frc.robot.subsystems;
//kauai
import com.kauailabs.navx.frc.AHRS;
//wpi
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
//robot
import frc.robot.Constants;



public class SwerveDrive extends SubsystemBase {
    
    private final SwerveModule MODULE_FRONT_LEFT = new SwerveModule(
        Constants.SwerveSubsystemConstants.ID_FRONT_LEFT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_DRIVE,
        Constants.SwerveSubsystemConstants.ID_FRONT_LEFT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_FRONT_LEFT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_FRONT_LEFT_ENCODER_ABSOLUTE
        ); 
    private final SwerveModule MODULE_BACK_LEFT = new SwerveModule(
        Constants.SwerveSubsystemConstants.ID_BACK_LEFT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_DRIVE,
        Constants.SwerveSubsystemConstants.ID_BACK_LEFT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_BACK_LEFT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_BACK_LEFT_ENCODER_ABSOLUTE
        );
    private final SwerveModule MODULE_FRONT_RIGHT = new SwerveModule(
        Constants.SwerveSubsystemConstants.ID_FRONT_RIGHT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_DRIVE,
        Constants.SwerveSubsystemConstants.ID_FRONT_RIGHT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_FRONT_RIGHT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_FRONT_RIGHT_ENCODER_ABSOLUTE
        );  
    private final SwerveModule MODULE_BACK_RIGHT = new SwerveModule(
        Constants.SwerveSubsystemConstants.ID_BACK_RIGHT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_DRIVE,
        Constants.SwerveSubsystemConstants.ID_BACK_RIGHT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_BACK_RIGHT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_BACK_RIGHT_ENCODER_ABSOLUTE
        );  

    private final AHRS navX = new AHRS();

    public SwerveDrive() {
        
    }
    @Override
    public void periodic() {

    }

    public double getRobotHeading() {
        return Math.IEEEremainder(Constants.SwerveSubsystemConstants.REVERSED_GYRO ? -navX.getAngle() : navX.getAngle() , 360);
    }
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getRobotHeading());
    }
    public void setChassisSpeed(ChassisSpeeds speed) {
        SwerveModuleState states[] = Constants.Kinematics.kDriveKinematics.toSwerveModuleStates(speed);
        
    }
    public void setModuleStates(SwerveModuleState states[]) {
        MODULE_FRONT_LEFT.setDesiredState(states[0]);
        MODULE_FRONT_RIGHT.setDesiredState(states[1]);
        MODULE_BACK_LEFT.setDesiredState(states[2]);
        MODULE_BACK_RIGHT.setDesiredState(states[3]);
    }


    public Command zeroRobotHeading() {
        return Commands.runOnce(() -> navX.reset());
    }
}
