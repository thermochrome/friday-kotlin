package org.firstinspires.ftc.teamcode.auton

import com.arcrobotics.ftclib.hardware.SimpleServo
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.robot.Constants
import org.firstinspires.ftc.teamcode.robot.Hardware
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "BLUE") @Suppress("unused")
class MainAuto : OpMode() {

    // Hardware
    private lateinit var hardware: Hardware

    // Pathing
    private lateinit var follower: Follower
    private lateinit var driveStarttoCurry: PathChain
    private lateinit var driveCurrytoBye: PathChain

    // Timers
    private lateinit var pathTimer: Timer
    private lateinit var opModeTimer: Timer

    private var state: UInt = 0u

    // Poses
    private val startPose = Pose(
        21.34629404617254,
        125.62818955042528,
        Math.toRadians(147.0)
    )

    private val shootPose = Pose(
        61.939246658566226,
        82.06075334143378,
        Math.toRadians(135.0)
    )

    private val byeBye = Pose(
        61.76427703523694,
        48.46658566221142,
        Math.toRadians(135.0)
    )

    private fun buildPaths() {
        driveStarttoCurry = follower.pathBuilder()
            .addPath(BezierLine(startPose, shootPose))
            .setLinearHeadingInterpolation(
                startPose.heading,
                shootPose.heading
            )
            .build()

        driveCurrytoBye = follower.pathBuilder()
            .addPath(BezierLine(shootPose, byeBye))
            .setLinearHeadingInterpolation(
                shootPose.heading,
                byeBye.heading
            )
            .build()
    }

    private fun statePathUpdate() {
        when (state) {
            0u -> {
                follower.followPath(driveStarttoCurry, true)
                state++
            }

            1u -> {
                hardware.get<MotorGroup>("outtake").set(0.75)
                hardware.get<CRServo>("upper_intake").set(1.0)
                hardware.get<SimpleServo>("feeder").position = 1.0 // 45.0 / 57.2958

                if (!follower.isBusy && pathTimer.elapsedTime.seconds > 2.seconds) {
                    hardware.get<SimpleServo>("feeder").position = 0.0 // 22.0 / 57.2958
                    hardware.get<MotorGroup>("outtake").set(0.0)
                    state++
                }
            }

            2u -> {
                follower.followPath(driveCurrytoBye, true)
            }
        }
    }

    private fun setPathState(newState: UInt) {
        state = newState
        pathTimer.resetTimer()
    }

    override fun init() {
        hardware = Hardware(hardwareMap)

        // Pathing + timers
        follower = Constants.createFollower(hardwareMap)
        pathTimer = Timer()
        opModeTimer = Timer()

        state = 0u

        buildPaths()
        follower.pose = startPose
    }

    override fun start() {
        opModeTimer.resetTimer()
        setPathState(state)
    }

    override fun loop() {
        follower.update()
        statePathUpdate()
    }
}
