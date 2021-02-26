package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous(name = "Path2Sharp", group = "Concept")
public class Path2Sharp extends LinearOpMode {
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

        //else if(path2) {
        //strafe left 1 tile
        Trajectory traj8 = drivetrain.trajectoryBuilder(traj2.end())
                .strafeLeft(20)
                .build();
        drivetrain.followTrajectory(traj8);

        //forward 3 tiles
        Trajectory traj9 = drivetrain.trajectoryBuilder(traj8.end())
                .forward(63)
                .build();
        drivetrain.followTrajectory(traj9);

        //strafe right 1 tile
        Trajectory traj10 = drivetrain.trajectoryBuilder(traj9.end())
                .strafeRight(20)
                .build();
        drivetrain.followTrajectory(traj10);

        //back 2 tiles
        Trajectory traj11 = drivetrain.trajectoryBuilder(traj10.end())
                .back(42)
                .build();
        drivetrain.followTrajectory(traj11);

        //PAUSE TO SHOOT
        //shoot rings into goal

        //forward 1 tile
        Trajectory traj12 = drivetrain.trajectoryBuilder(traj11.end())
                .forward(21)
                .build();
        drivetrain.followTrajectory(traj12);
        //}
    }
}
