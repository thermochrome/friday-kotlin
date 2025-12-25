package org.firstinspires.ftc.teamcode.auton

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.Setup

@Autonomous(name = "Blue") @Suppress("unused")
class Main2: Setup() {
    private lateinit var follower: Follower

    private var step = 0u

    private var steps: Map<Pose, (UInt) -> UInt> = mapOf(
        Pose(21.34629404617254, 125.62818955042528, Math.toRadians(147.0)) to {
            step++
        }
    )
}