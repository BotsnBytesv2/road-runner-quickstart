package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous(name = "RRTrajectories1", group = "Concept")
public class RRTrajectories extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        //R and L sides are in relation to when the person is facing the goals
        //if(were blue alliance left side){
        //Pose2d startPose = new Pose2d(-60, 48, 0.17453292519943295);
        //drivetrain.setPoseEstiamte(startPose);

        /*Trajectory traj1 = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(30)
                .build();
        drivetrain.followTrajectory(traj1);*/

        Trajectory traj1 = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(30)
                .build();
        Trajectory traj2 = drivetrain.trajectoryBuilder(traj1.end())
                .strafeRight(13)
                .build();

        drivetrain.followTrajectory(traj1);
        drivetrain.followTrajectory(traj2);
        //PAUSE TO SENSE
        //use vuforia to figure out # of rings

        //if(path1) {
        //Trajectories for Path1
        Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                .strafeLeft(25)
                .build();
        drivetrain.followTrajectory(traj3);

        Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                .forward(42)
                .build();
        drivetrain.followTrajectory(traj4);

        Trajectory traj5 = drivetrain.trajectoryBuilder(traj4.end())
                .strafeRight(25)
                .build();
        drivetrain.followTrajectory(traj5);

        Trajectory traj6 = drivetrain.trajectoryBuilder(traj5.end())
                .back(11)
                .build();
        drivetrain.followTrajectory(traj6);

        //PAUSE TO SHOOT
        //shoot rings into goal

        Trajectory traj7 = drivetrain.trajectoryBuilder(traj6.end())
                .forward(11)
                .build();

        //Path1
        drivetrain.followTrajectory(traj7);
        //}
            /*else if(path2) {
                //Trajectories for Path2
                Trajectory traj2 = drivetrain.trajectoryBuilder(traj1.end())
                        .forward(65)
                        .build();
                Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                        .back(35)
                        .build();
                //PAUSE TO SHOOT
                //shoot rings into goal
                Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                        .forward(11)
                        .build();
                //Path2
                drivetrain.followTrajectory(traj2);
                drivetrain.followTrajectory(traj3);
                drivetrain.followTrajectory(traj4);
            }
            else {
                //Trajectories for Path3
                Trajectory traj2 = drivetrain.trajectoryBuilder(traj1.end())
                        .strafeLeft(25)
                        .forward(90)
                        .build();
                Trajectory traj3 = drivetrain.trajectoryBuilder(traj2.end())
                        .strafeRight(25)
                        .back(60)
                        .build();
                //PAUSE TO SHOOT
                //shoot rings into goal
                Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                        .forward(11)
                        .build();
                //Path3
                drivetrain.followTrajectory(traj2);
                drivetrain.followTrajectory(traj3);
                drivetrain.followTrajectory(traj4);
            }*/

    }
}
