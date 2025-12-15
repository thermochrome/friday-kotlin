package org.firstinspires.ftc.teamcode.teleop

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.robot.ButtonData
import org.firstinspires.ftc.teamcode.robot.OpDevices
import org.firstinspires.ftc.teamcode.robot.registerControls
import kotlin.math.sqrt

enum class Activation {
    TOGGLE, PRESS, HELD
}

@TeleOp @Suppress("unused")
class Main: OpDevices() {
    private var powers = mutableMapOf(
        "outtake_power" to 0.75
    )

    private fun drive(gamepad: GamepadEx) {
        drive.driveFieldCentric(gamepad.leftX, gamepad.leftY, gamepad.rightX,
            hardware.get<GoBildaPinpointDriver>("odometry").getHeading(AngleUnit.DEGREES)
        )
    }

    override fun init() {
        super.init()

        val buttons = listOf(
            ButtonData(GamepadKeys.Button.X, Activation.TOGGLE,
                activated = InstantCommand({
                    powers["outtake_power"] = 0.75 /* powers["outtake_power"]!![1] */
                }),

                deactivated = InstantCommand({
                    powers["outtake_power"] = 0.0
                })
            ),

            ButtonData(GamepadKeys.Button.A, Activation.TOGGLE,
                activated = InstantCommand({
                    hardware.get<MotorEx>("intake").set(1.0)
                }),

                deactivated = InstantCommand({
                    hardware.get<MotorEx>("intake").set(0.0)
                })
            ),

            ButtonData(GamepadKeys.Button.Y, Activation.TOGGLE,
                activated = InstantCommand({
                    hardware.get<CRServo>("upper_intake").set(1.0)
                }),

                deactivated = InstantCommand({
                    hardware.get<CRServo>("upper_intake").set(0.0)
                })
            ),

            ButtonData(GamepadKeys.Button.DPAD_UP, Activation.HELD,
                InstantCommand({
                    hardware.get<MotorEx>("up").set(1.0)
                }),

                InstantCommand({
                    hardware.get<MotorEx>("up").set(0.0)
                }),

                gamepad = firstGamepad
            ),

            ButtonData(GamepadKeys.Button.DPAD_DOWN, Activation.HELD,
                InstantCommand({
                    hardware.get<MotorEx>("up").set(1.0)
                }),

                InstantCommand({
                    hardware.get<MotorEx>("up").set(0.0)
                }),

                gamepad = firstGamepad
            ),

            ButtonData(GamepadKeys.Button.DPAD_UP, Activation.PRESS, SequentialCommandGroup(
                InstantCommand({  hardware.get<SimpleServo>("feeder").position = 1.0 }),

                WaitCommand(600),

                InstantCommand({ hardware.get<SimpleServo>("feeder").position = 0.0 }),
            ))
        )

        registerControls(buttons, secondGamepad)
    }

    override fun loop() {
        super.loop()
        drive(firstGamepad)

        val limelight: Limelight3A = hardware["limelight"]
        telemetry.addData("RPM", hardware.get<MotorGroup>("outtake").velocity * 60 / 103.8)

        var minimumDistance = Double.MAX_VALUE

        limelight.latestResult.fiducialResults.associate { tag ->
            val position = listOf(
                tag.targetPoseCameraSpace.position.x,
                tag.targetPoseCameraSpace.position.y,
                tag.targetPoseCameraSpace.position.z
            )

            tag.fiducialId to sqrt(position.sumOf { d -> d * d })
        }.forEach { (tagId, distance) ->
            telemetry.addData("$tagId Distance", distance)

            if (distance < minimumDistance) {
                minimumDistance = distance
            }
        }

        if (minimumDistance == Double.MAX_VALUE) {
            minimumDistance = 0.0
        }

        hardware.get<MotorGroup>("outtake").set(0.119219 * minimumDistance + 0.661507)

        if (limelight.latestResult.isValid) {
            telemetry.addData("Heading", limelight.latestResult.fiducialResults[0].targetPoseCameraSpace.position.x)
        }
    }
}