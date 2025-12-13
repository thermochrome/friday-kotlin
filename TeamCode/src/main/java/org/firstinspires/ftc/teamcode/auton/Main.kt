package org.firstinspires.ftc.teamcode.auton

import com.arcrobotics.ftclib.hardware.motors.Motor
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.Constants

@Autonomous(name = "BLUE")
class MainAuto : OpMode() {

    // Hardware
    private lateinit var right: Motor
    private lateinit var left: Motor
    private lateinit var feeder: CRServo
    private lateinit var pusher: Servo

    // Pathing
    private lateinit var follower: Follower
    private lateinit var driveStarttoCurry: PathChain
    private lateinit var driveCurrytoBye: PathChain

    // Timers
    private lateinit var pathTimer: Timer
    private lateinit var opModeTimer: Timer

    enum class PathState {
        START_SHOOT,
        CURRYONTHETHREE,
        SHOOT_LEAVE
    }

    private lateinit var pathState: PathState

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
        when (pathState) {
            PathState.START_SHOOT -> {
                follower.followPath(driveStarttoCurry, true)
                pathState = PathState.CURRYONTHETHREE
            }

            PathState.CURRYONTHETHREE -> {
                right.set(-0.75)
                left.set(0.75)
                feeder.power = 1.0
                pusher.position = 45.0 / 57.2958

                if (!follower.isBusy && pathTimer.elapsedTime > 5) {
                    pusher.position = 22.0 / 57.2958
                    right.set(0.0)
                    left.set(0.0)
                    pathState = PathState.SHOOT_LEAVE
                }
            }

            PathState.SHOOT_LEAVE -> {
                follower.followPath(driveCurrytoBye, true)
            }
        }
    }

    private fun setPathState(newState: PathState) {
        pathState = newState
        pathTimer.resetTimer()
    }

    override fun init() {
        // Hardware init
        right = Motor(hardwareMap, "outtake_r")
        left = Motor(hardwareMap, "outtake_l")
        feeder = hardwareMap.get(CRServo::class.java, "upper_intake")
        pusher = hardwareMap.get(Servo::class.java, "feeder")

        // Pathing + timers
        follower = Constants.createFollower(hardwareMap)
        pathTimer = Timer()
        opModeTimer = Timer()

        pathState = PathState.START_SHOOT

        buildPaths()
        follower.pose = startPose
    }

    override fun start() {
        opModeTimer.resetTimer()
        setPathState(pathState)
    }

    override fun loop() {
        follower.update()
        statePathUpdate()
    }
}
