package org.firstinspires.ftc.teamcode.auton

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathBuilder
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.Setup

private typealias Applicant<T> = T.() -> Unit

@Autonomous(name = "Blue") @Suppress("unused")
class Main2: Setup() {
    private lateinit var follower: Follower

    private var step = 0u

    private val steps: List<Pose> = listOf(
        Pose(21.34629404617254, 125.62818955042528, Math.toRadians(147.0)),
        Pose(61.939246658566226, 82.06075334143378, Math.toRadians(135.0)),
        Pose(61.76427703523694, 48.46658566221142, Math.toRadians(135.0))
    )

    private lateinit var pathChain: List<PathChain>

    override fun init() {
        super.init()

        fun make(apply: Applicant<PathBuilder>): PathChain {
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
}