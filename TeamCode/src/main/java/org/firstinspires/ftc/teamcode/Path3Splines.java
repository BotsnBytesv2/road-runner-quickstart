package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous(name = "Path3Splines", group = "Concept")
public class Path3Splines extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-40, 30, 0);
        drivetrain.setPoseEstimate(startPose);

        //if(path3) { //based on Vuforia
        Trajectory traj2 = drivetrain.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(54, 63), 0)
                .build();
        drivetrain.followTrajectory(traj2);
        //drop wobble goal
        sleep(1000);

        Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-10, 35), 0)
                .build();
        drivetrain.followTrajectory(traj3);

        //PAUSE TO SHOOT
        drivetrain.turn(Math.toRadians(180));
        sleep(1000);
        //shoot rings into goal

        Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(-48, 24), 0)
                .build();
        drivetrain.followTrajectory(traj4);
        //pick up wobble goal
        sleep(1000);

        Trajectory traj5 = drivetrain.trajectoryBuilder(traj4.end())
                .splineTo(new Vector2d(50, 63), 0)
                .build();
        drivetrain.followTrajectory(traj5);

        Trajectory traj6 = drivetrain.trajectoryBuilder(traj5.end())
                .splineTo(new Vector2d(54, 63), 0)
                .build();
        drivetrain.followTrajectory(traj6);
        //drop wobble goal
        sleep(1000);

        Trajectory traj7 = drivetrain.trajectoryBuilder(traj6.end())
                .splineTo(new Vector2d(12, 35), 0)
                .build();
        drivetrain.followTrajectory(traj7);
        //}
    }
}
