package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous(name = "Path2", group = "Concept")
public class Path2 extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        /*
        Blue Alliance Left side
         */

        Pose2d startPose = new Pose2d(-60, 48, 0);
        drivetrain.setPoseEstimate(startPose);

        Trajectory traj1 = drivetrain.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-40, 30), 0)
                .build();
        drivetrain.followTrajectory(traj1);

        //PAUSE TO SENSE
        sleep(10000);
        //use Vurforia to figure out # of rings

        //if(path2) { //based on Vuforia
        Trajectory traj2 = drivetrain.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(30, 35), 0)
                .build();
        drivetrain.followTrajectory(traj2);
        //drop wobble goal
        sleep(10000);

        Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-10, 35), 0)
                .build();
        drivetrain.followTrajectory(traj3);

        //PAUSE TO SHOOT
        sleep(10000);
        //shoot rings into goal

        Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(-48, 24), 0)
                .build();
        drivetrain.followTrajectory(traj4);
        //pick up wobble goal
        sleep(10000);

        Trajectory traj5 = drivetrain.trajectoryBuilder(traj4.end())
                .splineTo(new Vector2d(25, 35), 0)
                .build();
        drivetrain.followTrajectory(traj5);

        drivetrain.turn(Math.toRadians(180));

        Trajectory traj6 = drivetrain.trajectoryBuilder(traj5.end())
                .splineTo(new Vector2d(30, 35), 0)
                .build();
        drivetrain.followTrajectory(traj6);

        //drop wobble goal
        sleep(10000);

        Trajectory traj7 = drivetrain.trajectoryBuilder(traj6.end())
                .splineTo(new Vector2d(12, 35), 0)
                .build();
        drivetrain.followTrajectory(traj7);
        //}
    }
}
