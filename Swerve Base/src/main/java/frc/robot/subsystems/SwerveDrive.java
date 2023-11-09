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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//robot
import frc.robot.Constants;

import java.util.Timer;
import java.util.concurrent.TimeUnit;


public class SwerveDrive extends SubsystemBase {
    
    private final SwerveModule MODULE_FRONT_LEFT = new SwerveModule(
        Constants.SwerveSubsystemConstants.ID_FRONT_LEFT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_FRONT_LEFT_MOTOR_DRIVE,
        Constants.SwerveSubsystemConstants.ID_FRONT_LEFT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_FRONT_LEFT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_FRONT_LEFT_ENCODER_ABSOLUTE
        ); 
    private final SwerveModule MODULE_BACK_LEFT = new SwerveModule(
        Constants.SwerveSubsystemConstants.ID_BACK_LEFT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_BACK_LEFT_MOTOR_DRIVE,
        Constants.SwerveSubsystemConstants.ID_BACK_LEFT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_BACK_LEFT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_BACK_LEFT_ENCODER_ABSOLUTE
        );
    private final SwerveModule MODULE_FRONT_RIGHT = new SwerveModule(
        Constants.SwerveSubsystemConstants.ID_FRONT_RIGHT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_FRONT_RIGHT_MOTOR_DRIVE,
        Constants.SwerveSubsystemConstants.ID_FRONT_RIGHT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_FRONT_RIGHT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_FRONT_RIGHT_ENCODER_ABSOLUTE
        );  
    private final SwerveModule MODULE_BACK_RIGHT = new SwerveModule(
        Constants.SwerveSubsystemConstants.ID_BACK_RIGHT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_BACK_RIGHT_MOTOR_DRIVE,
        Constants.SwerveSubsystemConstants.ID_BACK_RIGHT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_BACK_RIGHT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_BACK_RIGHT_ENCODER_ABSOLUTE
        );  

    private final AHRS navX = new AHRS();

    public SwerveDrive() {
        try {TimeUnit.SECONDS.sleep(1);}
        catch(InterruptedException e){}
        MODULE_FRONT_LEFT.resetEncoders();
        MODULE_FRONT_RIGHT.resetEncoders();
        MODULE_BACK_LEFT.resetEncoders();
        MODULE_BACK_RIGHT.resetEncoders();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getRobotHeading());
        //SmartDashboard.putNumber("/timer", Timer.getMatchTime());

        SmartDashboard.putNumber("FL ABS ENC", MODULE_FRONT_LEFT.getAbsoluteEncoder());


        SmartDashboard.putNumber("FR ABS ENC", MODULE_FRONT_RIGHT.getAbsoluteEncoder());


        SmartDashboard.putNumber("BL ABS ENC", MODULE_BACK_LEFT.getAbsoluteEncoder());


        SmartDashboard.putNumber("BR ABS ENC", MODULE_BACK_RIGHT.getAbsoluteEncoder());


    }

    public double getRobotHeading() {
        return Math.IEEEremainder(Constants.SwerveSubsystemConstants.REVERSED_GYRO ? -navX.getAngle() : navX.getAngle() , 360);
    }
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getRobotHeading());
    }
    public void setChassisSpeed(ChassisSpeeds speed) {
        SwerveModuleState states[] = Constants.Kinematics.kDriveKinematics.toSwerveModuleStates(speed);
        setModuleStates(states);
    }
    public void setModuleStates(SwerveModuleState states[]) {
        MODULE_FRONT_LEFT.setDesiredState(states[0]);
        MODULE_FRONT_RIGHT.setDesiredState(states[1]);
        MODULE_BACK_LEFT.setDesiredState(states[2]);
        MODULE_BACK_RIGHT.setDesiredState(states[3]);
    }
    public void zeroModules(){
        MODULE_FRONT_LEFT.resetEncoders();
        MODULE_FRONT_RIGHT.resetEncoders();
        MODULE_BACK_LEFT.resetEncoders();
        MODULE_BACK_RIGHT.resetEncoders();   
    }


    public Command zeroRobotHeading() {
        return Commands.runOnce(() -> navX.reset());
    }
    public Command zeroModuleAngles() {
        return Commands.runOnce(()-> zeroModules());
    }
}
