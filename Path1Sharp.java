package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous(name = "Path1Sharp", group = "Concept")
public class Path1Sharp extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        /*
        Blue Alliance Left side
         */

        //forward 1.5 tiles
        Trajectory traj1 = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(30)
                .build();
        //strafe right 0.5 tile
        Trajectory traj2 = drivetrain.trajectoryBuilder(traj1.end())
                .strafeRight(10)
                .build();

        drivetrain.followTrajectory(traj1);
        drivetrain.followTrajectory(traj2);
        //PAUSE TO SENSE
        //use Vurforia to figure out # of rings

        /*
        Path 1
         */
        //if(path1) { //based on Vuforia
        //strafe left 1 tile
        Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                .strafeLeft(20)
                .build();
        drivetrain.followTrajectory(traj3);

        //strafe forward 2 tiles
        Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                .forward(42)
                .build();
        drivetrain.followTrajectory(traj4);

        //strafe right 1 tile
        Trajectory traj5 = drivetrain.trajectoryBuilder(traj4.end())
                .strafeRight(20)
                .build();
        drivetrain.followTrajectory(traj5);

        //strafe back 1 tile
        Trajectory traj6 = drivetrain.trajectoryBuilder(traj5.end())
                .back(21)
                .build();
        drivetrain.followTrajectory(traj6);

        //PAUSE TO SHOOT
        //shoot rings into goal

        //strafe forward 1 tile
        Trajectory traj7 = drivetrain.trajectoryBuilder(traj6.end())
                .forward(21)
                .build();
        drivetrain.followTrajectory(traj7);
        //}
    }
}
