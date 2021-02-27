package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous(name = "Path1Sharp", group = "Concept")
public class Path1Sharp extends LinearOpMode {
    private DcMotor input = null;
    private DcMotor output = null;
    private DcMotor wobbleArm = null;
    private DcMotor input2 = null;
    private Servo topClaw = null;
    private Servo bottomClaw = null;
    private Servo transfer = null;
    @Override
    public void runOpMode(){
        input = hardwareMap.get(DcMotor.class, "input");
        output = hardwareMap.get(DcMotor.class, "output");
        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");
        transfer = hardwareMap.get(Servo.class, "transferServo");
        topClaw = hardwareMap.get(Servo.class, "topClaw");
        bottomClaw = hardwareMap.get(Servo.class, "bottomClaw");
        input2 = hardwareMap.get(DcMotor.class, "input2");
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        transfer.setPosition(0.8);
        topClaw.setPosition(0.23);
        bottomClaw.setPosition(0.4);
        waitForStart();

        /*
        Blue Alliance Left side
         */

        //forward 1.5 tiles
        wobbleArm.setPower(1.0);
        Trajectory traj1 = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(21)
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
                .strafeLeft(22)
                .build();
        drivetrain.followTrajectory(traj3);

        //forward 2 tiles
        Trajectory traj4 = drivetrain.trajectoryBuilder(traj3.end())
                .forward(36)
                .build();
        drivetrain.followTrajectory(traj4);

        //drop wobble 1
        wobbleArm.setPower(0.0);
        topClaw.setPosition(0.6);
        bottomClaw.setPosition(0.1);
        sleep(100);

        //strafe right 1 tile
        Trajectory traj5 = drivetrain.trajectoryBuilder(traj4.end())
                .strafeRight(18)
                .build();
        drivetrain.followTrajectory(traj5);
        output.setPower(1.0);
        drivetrain.turn(Math.toRadians(177));
        sleep(100);
        transfer.setPosition(0.7);
        sleep(100);
        transfer.setPosition(0.8);
        sleep(100);
        drivetrain.turn(Math.toRadians(-5));
        sleep(100);
        transfer.setPosition(0.7);
        sleep(100);
        transfer.setPosition(0.8);
        drivetrain.turn(Math.toRadians(-5));
        transfer.setPosition(0.7);
        sleep(100);
        transfer.setPosition(0.8);
        sleep(100);
        drivetrain.turn(Math.toRadians(-5));
        transfer.setPosition(0.7);
        sleep(100);
        transfer.setPosition(0.8);
        sleep(100);

        drivetrain.turn(Math.toRadians(10));
        output.setPower(0.0);

        //forward 3.5 tiles
        Trajectory back = drivetrain.trajectoryBuilder(new Pose2d(traj5.end().getX(), traj5.end().getY(), Math.toRadians(185)))
                .strafeLeft(19)
                .build();
        drivetrain.followTrajectory(back);

        //forward 2 tiles
        Trajectory traj6 = drivetrain.trajectoryBuilder(back.end())
                .forward(37)
                .build();
        drivetrain.followTrajectory(traj6);

        drivetrain.turn(Math.toRadians(185));

        Trajectory traj7 = drivetrain.trajectoryBuilder(new Pose2d(traj6.end().getX(), traj6.end().getY(), traj6.end().getHeading()+Math.toRadians(185)))
                .forward(43)
                .build();
        drivetrain.followTrajectory(traj7);

        //pick up wobble 2
        topClaw.setPosition(0.23);
        bottomClaw.setPosition(0.4);
        wobbleArm.setPower(-1.0);
        sleep(2000);
        wobbleArm.setPower(0.0);
        //strafe forward 1 tile
        Trajectory traj8 = drivetrain.trajectoryBuilder(traj7.end())
                .strafeLeft(43)
                .build();
        drivetrain.followTrajectory(traj8);
        //}
        //drop wobble 2
        wobbleArm.setPower(1.0);
        sleep(2000);
        wobbleArm.setPower(0.0);
        sleep(100);
    }
}
