package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.plays.DoNothing;
import frc.robot.auto.plays.TestAutoPlay;
import frc.robot.auto.plays.Blue.BBottomLeaveCommunity;
import frc.robot.auto.plays.Blue.BMidBalance;
import frc.robot.auto.plays.Blue.BMidLeaveCommunity;
import frc.robot.auto.plays.Blue.BSetBottomPose;
import frc.robot.auto.plays.Blue.BSetMidPose;
import frc.robot.auto.plays.Blue.BSetTopPose;
import frc.robot.auto.plays.Blue.BTopLeaveCommunity;
import frc.robot.auto.plays.Blue.BbP3P4_Dock;
import frc.robot.auto.plays.Blue.BbP3P4_NoDock;
import frc.robot.auto.plays.Blue.BbP3_Dock;
import frc.robot.auto.plays.Blue.BbP4_Dock;
import frc.robot.auto.plays.Blue.BbP4_NoDock;
import frc.robot.auto.plays.Blue.BbScoreGiven;
import frc.robot.auto.plays.Blue.BbScoreGivenLeave;
import frc.robot.auto.plays.Blue.BtP1_Dock;
import frc.robot.auto.plays.Blue.BtP1_NoDock;
import frc.robot.auto.plays.Blue.BtP2_Dock;
import frc.robot.auto.plays.Blue.BtP2_NoDock;
import frc.robot.auto.plays.Blue.BtScoreGiven;
import frc.robot.auto.plays.Blue.BtScoreGivenLeave;
import frc.robot.auto.plays.Red.RBottomLeaveCommunity;
import frc.robot.auto.plays.Red.RMidLeaveCommunity;
import frc.robot.auto.plays.Red.RSetBottomPose;
import frc.robot.auto.plays.Red.RSetMidPose;
import frc.robot.auto.plays.Red.RSetTopPose;
import frc.robot.auto.plays.Red.RTopLeaveCommunity;
import frc.robot.auto.plays.Red.RbP3P4_NoDock;
import frc.robot.auto.plays.Red.RbP3_Dock;
import frc.robot.auto.plays.Red.RbP4_Dock;
import frc.robot.auto.plays.Red.RbP4_NoDock;
import frc.robot.auto.plays.Red.RbScoreGiven;
import frc.robot.auto.plays.Red.RtP1_Dock;
import frc.robot.auto.plays.Red.RtP1_NoDock;
import frc.robot.auto.plays.Red.RtP2_Dock;
import frc.robot.auto.plays.Red.RtP2_NoDock;
import frc.robot.auto.plays.Red.RtScoreGiven;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoChooser {
    private ShuffleboardTab preMatch;
    private SendableChooser<Command> autoChooser;
    
    public AutoChooser(DrivetrainSubsystem mDrivetrainSubsystem){
        preMatch = Shuffleboard.getTab("Pre-Match");
        autoChooser = new SendableChooser<>();

        //auto plays
        autoChooser.setDefaultOption("Do Nothing", new DoNothing());
        autoChooser.addOption("BLUE-BOTTOM: Set Pose", new BSetBottomPose(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-BOTTOM: Leave Community", new BBottomLeaveCommunity(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-BOTTOM: Score Piece 3, Dock", new BbP3_Dock(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-BOTTOM: Score Piece 3, No Dock", new BbP3P4_NoDock(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-BOTTOM: Score Piece 4, Dock", new BbP4_Dock(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-BOTTOM: Score Piece 4, No Dock", new BbP4_NoDock(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-BOTTOM: Score Given Piece", new BbScoreGiven(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-BOTTOM: Score Given Piece, Leave Community", new BbScoreGivenLeave(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-MIDDLE: Set Pose", new BSetMidPose(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-MIDDLE: Leave Community", new BMidLeaveCommunity(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-MIDDLE: Balance", new BMidBalance(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-TOP: Set Pose", new BSetTopPose(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-TOP: Leave Community", new BTopLeaveCommunity(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-TOP: Score Piece 1, Dock", new BtP1_Dock(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-TOP: Score Piece 1, No Dock", new BtP1_NoDock(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-TOP: Score Piece 2, Dock", new BtP2_Dock(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-TOP: Score Piece 2, No Dock", new BtP2_NoDock(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-TOP: Score Given Piece", new BtScoreGiven(mDrivetrainSubsystem));
        autoChooser.addOption("BLUE-TOP: Score Given Piece, Leave Community", new BtScoreGivenLeave(mDrivetrainSubsystem));
        autoChooser.addOption("RED-BOTTOM: Set Pose", new RSetBottomPose(mDrivetrainSubsystem));
        autoChooser.addOption("RED-BOTTOM: Leave Community", new RBottomLeaveCommunity(mDrivetrainSubsystem));
        autoChooser.addOption("RED-BOTTOM: Score Piece 3, Dock", new RbP3_Dock(mDrivetrainSubsystem));
        autoChooser.addOption("RED-BOTTOM: Score Piece 3, No Dock", new RbP3P4_NoDock(mDrivetrainSubsystem));
        autoChooser.addOption("RED-BOTTOM: Score Piece 4, Dock", new RbP4_Dock(mDrivetrainSubsystem));
        autoChooser.addOption("RED-BOTTOM: Score Piece 4, No Dock", new RbP4_NoDock(mDrivetrainSubsystem));
        autoChooser.addOption("RED-BOTTOM: Score Given Piece", new RbScoreGiven(mDrivetrainSubsystem));
        autoChooser.addOption("RED-BOTTOM: Score Given Piece, Leave Community", new BbScoreGivenLeave(mDrivetrainSubsystem));
        autoChooser.addOption("RED-MIDDLE: Set Pose", new RSetMidPose(mDrivetrainSubsystem));
        autoChooser.addOption("RED-MIDDLE: Leave Community", new RMidLeaveCommunity(mDrivetrainSubsystem));
        autoChooser.addOption("RED-TOP: Set Pose", new RSetTopPose(mDrivetrainSubsystem));
        autoChooser.addOption("RED-TOP: Leave Community", new RTopLeaveCommunity(mDrivetrainSubsystem));
        autoChooser.addOption("RED-TOP: Score Piece 1, Dock", new RtP1_Dock(mDrivetrainSubsystem));
        autoChooser.addOption("RED-TOP: Score Piece 1, No Dock", new RtP1_NoDock(mDrivetrainSubsystem));
        autoChooser.addOption("RED-TOP: Score Piece 2, Dock", new RtP2_Dock(mDrivetrainSubsystem));
        autoChooser.addOption("RED-TOP: Score Piece 2, No Dock", new RtP2_NoDock(mDrivetrainSubsystem));
        autoChooser.addOption("RED-TOP: Score Given Piece", new RtScoreGiven(mDrivetrainSubsystem));
        autoChooser.addOption("RED-TOP: Score Given Piece, Leave Community", new BtScoreGivenLeave(mDrivetrainSubsystem));


        
        preMatch.add(autoChooser).withSize(2, 1).withPosition(0, 0);
    }

    public Command getSelected(){
        return autoChooser.getSelected();
    }

    public static Trajectory openTrajectoryFile(String name){
        try{
            Trajectory t = new Trajectory();
            Path path = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/" + name);
            t = TrajectoryUtil.fromPathweaverJson(path);
            return t;
        }
        catch(IOException ex){
            DriverStation.reportError("Unable to open trajectory: " + name, ex.getStackTrace());
            return null;
        }
    }
}
