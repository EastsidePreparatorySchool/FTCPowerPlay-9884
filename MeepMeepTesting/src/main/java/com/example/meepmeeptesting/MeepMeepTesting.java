package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // 16.5, 17.5
                .setDimensions(16.5,17.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(54.7, 54.7, Math.toRadians(232), Math.toRadians(232), 13.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.25, -61.75  , Math.toRadians(90)))
                                .addDisplacementMarker(() -> {
                                    // robot.setClawRot(robot.CLAW_CLOSED_POSITION, robot.CLAW_CLOSED_POSITION)
                                })
                                .strafeTo(new Vector2d(-35.25,-61.75+26.5))
                                .addDisplacementMarker(() -> {
                                    // robot.ArmMotor.setTargetPosition(robot.HIGH_JUNCTION_ENCODER_CONSTANT)
                                })
                                .strafeTo(new Vector2d(0,-61.75+26.5))
                                .strafeTo(new Vector2d(0,-61.75+26.5+3))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(()->{
                                    // robot.setClawRot(robot.LEFT_CLAW_OPEN_POSITION, robot.RIGHT_CLAW_OPEN_POSITION)
                                })
                                .waitSeconds(0.5)
                                .splineToConstantHeading(new Vector2d(0, -61.75+26.5),Math.toRadians(90))
                                .addDisplacementMarker(() -> {
                                    // robot.ArmMotor.setTargetPosition(0)
                                })
                                .waitSeconds(2)
                                .splineToConstantHeading(new Vector2d(-11.75,-61.75+26.5),Math.toRadians(90))
                                .splineTo(new Vector2d(-11.75,-11.75),Math.toRadians(180))
                                .splineTo(new Vector2d(-61.75, -11.75), Math.toRadians(180))
                                .addDisplacementMarker(() -> {
                                    // robot.setClawRot(robot.CLAW_CLOSED_POSITION, robot.CLAW_CLOSED_POSITION)
                                })
                                .waitSeconds(0.5)
                                .addDisplacementMarker(()->{
                                    // robot.ArmMotor.setTargetPosition(robot.HIGH_JUNCTION_ENCODER_CONSTANT)
                                })
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(new Vector2d(0, -11.75), Math.toRadians(0))
                                .setReversed(false)
                                .turn(Math.toRadians(90))
                                .splineTo(new Vector2d(0,-14.75), Math.toRadians(270))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(()->{
                                    // robot.setClawRot(robot.LEFT_CLAW_OPEN_POSITION, robot.RIGHT_CLAW_OPEN_POSITION)
                                })
                                .waitSeconds(0.5)
                                .splineToConstantHeading(new Vector2d(0, -11.75),Math.toRadians(270))
                                .addDisplacementMarker(() -> {
                                    // robot.ArmMotor.setTargetPosition(0)
                                })
                                .turn(Math.toRadians(-90))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-35.25, -11.75), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}