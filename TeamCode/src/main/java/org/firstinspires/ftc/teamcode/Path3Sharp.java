package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous(name = "Path3Sharp", group = "Concept")
public class Path3Sharp extends LinearOpMode {
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

        //else{
        //strafe left 1 tile
        Trajectory traj13 = drivetrain.trajectoryBuilder(traj2.end())
                .strafeLeft(20)
                .build();
        drivetrain.followTrajectory(traj13);

        //forward 4 tiles
        Trajectory traj14 = drivetrain.trajectoryBuilder(traj13.end())
                .forward(84)
                .build();
        drivetrain.followTrajectory(traj14);

        //strafe right 1 tile
        Trajectory traj15 = drivetrain.trajectoryBuilder(traj14.end())
                .strafeRight(20)
                .build();
        drivetrain.followTrajectory(traj15);

        //back 3 tiles
        Trajectory traj16 = drivetrain.trajectoryBuilder(traj15.end())
                .back(63)
                .build();
        drivetrain.followTrajectory(traj16);

        //PAUSE TO SHOOT
        //shoot rings into goal

        //forward 1 tile
        Trajectory traj17 = drivetrain.trajectoryBuilder(traj16.end())
                .forward(21)
                .build();
        drivetrain.followTrajectory(traj17);
        //}
    }
}
