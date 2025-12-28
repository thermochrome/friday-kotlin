package org.firstinspires.ftc.teamcode.auton

import com.arcrobotics.ftclib.hardware.SimpleServo
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathBuilder
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.Constants
import org.firstinspires.ftc.teamcode.robot.Setup
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "Red") @Suppress("unused")
class AutonomousRed: Setup() {
    private lateinit var follower: Follower

    private lateinit var pathTimer: Timer
    private lateinit var modeTimer: Timer

    private var step = 0

    private val steps: List<Pose> = listOf(
        Pose(144 - 21.34629404617254, 125.62818955042528, Math.toRadians(360 - 147.0)),
        Pose(144 - 61.939246658566226, 82.06075334143378, Math.toRadians(360 - 135.0)),
        Pose(144 - 61.76427703523694, 48.46658566221142, Math.toRadians(360 - 135.0))
    )

    private lateinit var pathChain: List<PathChain>

    override fun init() {
        super.init()

        pathTimer = Timer(); modeTimer = Timer()
        follower = Constants.createFollower(hardwareMap)

        fun make(apply: PathBuilder.() -> Unit): PathChain {
            return follower.pathBuilder().apply(apply).build()
        }

        val pathProperties: List<(Pose, Pose) -> PathChain> = listOf(
            { start, end -> make {
                addPath(BezierLine(start, end))
                setLinearHeadingInterpolation(start.heading, end.heading)
            }},

            { start, end -> make {
                addPath(BezierLine(start, end))
                setLinearHeadingInterpolation(start.heading, end.heading)
            }}
        )

        pathChain = steps.zipWithNext().mapIndexed { index, (start, end) ->
            pathProperties.getOrElse(index) { pathProperties.last() }.invoke(start, end)
        }
    }

    override fun start() {
        super.start()

        pathTimer.resetTimer(); modeTimer.resetTimer()
    }

    override fun loop() {
        super.loop()

        follower.update()

        when (step) {
            0 -> {
                follower.followPath(pathChain[step], true)
                step++
            }

            1 -> {
                hardware.get<MotorGroup>("outtake").set(0.75)
                hardware.get<CRServo>("upper_intake").set(1.0)
                hardware.get<SimpleServo>("feeder").position = 1.0 // 45.0 / 57.2958

                if (!follower.isBusy && pathTimer.elapsedTime.seconds > 2.seconds) {
                    hardware.get<SimpleServo>("feeder").position = 0.0 // 22.0 / 57.2958
                    hardware.get<MotorGroup>("outtake").set(0.0)
                    step++
                }
            }

            2 -> {
                follower.followPath(pathChain[step], true)
            }
        }
    }
}