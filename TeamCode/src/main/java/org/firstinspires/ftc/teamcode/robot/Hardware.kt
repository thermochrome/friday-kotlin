package org.firstinspires.ftc.teamcode.robot

import com.arcrobotics.ftclib.command.Robot
import com.arcrobotics.ftclib.command.Subsystem
import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.sqrt

class Hardware(private val hardwareMap: HardwareMap) {
    private fun motor(name: String) = name to lazy {
        MotorEx(hardwareMap, name).apply {
            setRunMode(Motor.RunMode.RawPower)
        }
    }

    private fun continuousServo(name: String) = name to lazy {
        CRServo(hardwareMap, name)
    }

    private inline fun <reified T> device(name: String, crossinline apply: (T) -> Unit = {}) = name to lazy {
        hardwareMap.get(T::class.java, name).apply(apply)
    }

    val devices = mapOf(
        motor("left_front"),
        motor("right_front"),
        motor("left_back"),
        motor("right_back"),
        motor("intake"),
        "outtake" to lazy { MotorGroup(
            MotorEx(hardwareMap, "outtake_l"),
            MotorEx(hardwareMap, "outtake_r").apply { inverted = true }
        ).apply { setRunMode(Motor.RunMode.RawPower) }},

        motor("up"),

        continuousServo("upper_intake"),

        device<Servo>("feeder"),
        device<GoBildaPinpointDriver>("odometry") { it.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD) },
        device<Limelight3A>("limelight") { it.pipelineSwitch(0) }
    )

    inline operator fun <reified T> get(name: String): T {
        val device = devices[name]?.value ?: throw IllegalArgumentException("Couldn't find $name")
        return device as? T ?: throw IllegalArgumentException("$name is not ${T::class.simpleName}")
    }
}

class Drive(hardware: Hardware): Subsystem {
    val drive = MecanumDrive(
        hardware.get<MotorEx>("left_front"),
        hardware.get<MotorEx>("right_front"),
        hardware.get<MotorEx>("left_back"),
        hardware.get<MotorEx>("right_back")
    )

    @Suppress("unused")
    var power: Double = 1.0
        set(value) { drive.setMaxSpeed(value); field = value }

    fun drive(gamepad: GamepadEx) {
        drive.driveRobotCentric(gamepad.leftX, gamepad.leftY, gamepad.rightX)
    }
}

class Robot(hardwareMap: HardwareMap, gamepad1: Gamepad, gamepad2: Gamepad): Robot() {
    val hardware = Hardware(hardwareMap)
    val drive = Drive(hardware)

    val firstGamepad = GamepadEx(gamepad1); val secondGamepad = GamepadEx(gamepad2)

    private val limelight: Limelight3A = hardware["limelight"]
    private val odometry: GoBildaPinpointDriver = hardware["odometry"]

    init { register(drive) }

    fun start() {
        limelight.start()
        odometry.resetPosAndIMU()
    }

    fun loop(field: Boolean = false) {
        limelight.updateRobotOrientation(odometry.getHeading(AngleUnit.DEGREES))
        odometry.update()

        if (field) {
            drive.drive.driveFieldCentric(firstGamepad.leftX, firstGamepad.leftY, firstGamepad.rightX,
                odometry.getHeading(AngleUnit.DEGREES))
        } else {
            drive.drive(firstGamepad)
        }

        run()
    }

    fun tagDistances(): Map<Int, Double> {
        val result = limelight.latestResult

        return result.fiducialResults.associate { tag ->
            val position = listOf(
                tag.targetPoseCameraSpace.position.x,
                tag.targetPoseCameraSpace.position.y,
                tag.targetPoseCameraSpace.position.z
            )

            tag.fiducialId to sqrt(position.sumOf { d -> d * d })
        }
    }
}