package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.Robot;

import java.lang.Math;

@Config
@Autonomous(name = "Zak", group = "Auto")
public class Zak extends LinearOpMode {
    double GOAL_HEADING = Math.toRadians(135);
    final Vector2d GOAL = new Vector2d(-20, 22);
    final Pose2d GOAL_POSE = new Pose2d(GOAL, GOAL_HEADING);
    final Vector2d PARK_2 = new Vector2d(30, 60);
    final Vector2d PARK_1 = new Vector2d(-34, 30);
    final Vector2d SPIKE_3 = new Vector2d(-12, 20);
    final Vector2d SPIKE_2 = new Vector2d(12, 20);
    final Vector2d SPIKE_1 = new Vector2d(36, 25);
    final Vector2d SPIKE_3_FINAL = new Vector2d(-12, 50);
    final Vector2d SPIKE_2_FINAL = new Vector2d(12, 50);
    final Vector2d SPIKE_1_FINAL = new Vector2d(36, 50);
    //        final Pose2d START_POSE = new Pose2d(60, -12, Math.toRadians(180));
    final Pose2d startPose = new Pose2d(-60, 10, Math.toRadians(0));
    final Pose2d START_POSE2 = new Pose2d(55, 10, Math.toRadians(180));
    double GOAL_HEADING2 = Math.toRadians(135);
    final Vector2d GOAL2 = new Vector2d(-35, 20);
    final Pose2d GOAL_POSE2 = new Pose2d(GOAL2, GOAL_HEADING);

    //        final Pose2d Gate = new Pose2d(-2, 45, Math.toRadians(90));
    final Vector2d Gate = new Vector2d(10.5, 55.5);
    double GOAL_BACk_HEADING = Math.toRadians(156);
    final Vector2d GOAL_BACK = new Vector2d(54, 12);
    final Pose2d GOAL_BACK_POSE = new Pose2d(GOAL_BACK, GOAL_BACk_HEADING);

    PoseMap poseMap = Robot.alliance == Robot.Alliance.RED
            ? (pose -> pose)
            : (pose -> new Pose2dDual<>(
            pose.position.x,
            pose.position.y.unaryMinus(),
            pose.heading.inverse()
    ));

    Pose2d mappedStartPose = Robot.alliance == Robot.Alliance.RED
            ? startPose
            : new Pose2d(
            startPose.position.x,
            -startPose.position.y,
            -startPose.heading.toDouble()
    );

    /* ---------------- ACTIONS ---------------- */

    @Override
    public void runOpMode() {

        /* ---------------- INIT ---------------- */
        Robot robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);

        AutoActions autoActions = new AutoActions(robot);

        MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, mappedStartPose);

        /* ---------------- TRAJECTORIES ---------------- */
        waitForStart();

        if (isStopRequested()) return;

        TrajectoryActionBuilder tab = drive.actionBuilder(startPose, poseMap)
                // Preload score
                .afterTime(0, autoActions.shooterIdle())
//                .setTangent(Math.toRadians(315))
                .afterTime(0, autoActions.fire(1100))
                .afterTime(5, autoActions.stopShooter())
                .waitSeconds(5)

                .afterTime(0, autoActions.intake())
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(-61,56), Math.toRadians(90))
                .stopAndAdd(new ParallelAction(
                        autoActions.intake(),
                        new SleepAction(2)
                ))
                .afterTime(0, autoActions.stopIntake())

                //Park
                .splineTo(new Vector2d(-30,50), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(new Vector2d(65,10), Math.toRadians(180)), -1)
                .afterTime(0, autoActions.fire(1100))
                .afterTime(5, autoActions.stopShooter())
                .waitSeconds(3)
                .splineTo(PARK_2, Math.toRadians(0))
;

        /* ---------------- RUN AUTO ---------------- */
// In THINGALING.java, change your runBlocking to:
        Actions.runBlocking(
                new RaceAction( // Ends when the first action (the driving) finishes
                        new SequentialAction(tab.build()),
                        new ParallelAction(
                                autoActions.updateLL(),
                                autoActions.turretTrack(),
                                autoActions.updateShooter(),

                                (packet) -> {
                                    // Add any custom telemetry here
                                    packet.put("Robot Pose X", drive.localizer.getPose().position.x);
                                    packet.put("Robot Pose Y", drive.localizer.getPose().position.y);
                                    packet.put("Turret Pos", robot.turret.getPosition());

                                    // Return true to keep this "telemetry loop" running
                                    return true;
                                }
                        )
                )
        );

    }
}