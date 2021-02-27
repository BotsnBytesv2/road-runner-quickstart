package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous(name = "Test", group = "Concept")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        /*
        Blue Alliance Left side
         */

        //forward 1 tile
        Trajectory traj1 = drivetrain.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(185)))
                .back(24)
                .build();

        //back 1 tile
        Trajectory traj3 = drivetrain.trajectoryBuilder(traj1.end())
                .forward(24)
                .build();

        //strafe right 1 tile
        Trajectory traj2 = drivetrain.trajectoryBuilder(traj3.end())
                .strafeRight(24)
                .build();

        //strafe right 1 tile
        Trajectory traj4 = drivetrain.trajectoryBuilder(traj2.end())
                .strafeLeft(24)
                .build();

        //turn right 180 degrees
        drivetrain.turn(Math.toRadians(185));

        drivetrain.followTrajectory(traj1);
        drivetrain.followTrajectory(traj3);
        //drivetrain.turn(Math.toRadians(185));
        drivetrain.followTrajectory(traj2);
        drivetrain.followTrajectory(traj4);
    }
}
