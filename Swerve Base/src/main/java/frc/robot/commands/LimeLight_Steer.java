package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.LimeLight;
//import frc.robot.lib.Constants.Kinematics;
import frc.robot.subsystems.SwerveDrive;

public class LimeLight_Steer extends CommandBase {

    static SwerveDrive SUBSYSTEM_SWERVEDRIVE;
    static LimeLight SUBSYSTEM_LIMELIGHT;
    static PIDController TurnPID;

    public LimeLight_Steer(SwerveDrive drive, LimeLight limeLight){
        addRequirements(drive,limeLight);
        SUBSYSTEM_LIMELIGHT = limeLight;
        SUBSYSTEM_SWERVEDRIVE = drive;

        TurnPID = new PIDController(0, 0, 0);
    }

    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        double LL_Angle = SUBSYSTEM_LIMELIGHT.getLimeLightTX();
        double currentAngle = SUBSYSTEM_SWERVEDRIVE.getRobotHeading();

        double targetAngle = currentAngle + LL_Angle;

        ChassisSpeeds targetState = ChassisSpeeds.fromFieldRelativeSpeeds(0.0,0.0,TurnPID.calculate(currentAngle,targetAngle),SUBSYSTEM_SWERVEDRIVE.getRotation2d());

        SUBSYSTEM_SWERVEDRIVE.setChassisSpeed(targetState);


    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished(){
        return false;
    }
}