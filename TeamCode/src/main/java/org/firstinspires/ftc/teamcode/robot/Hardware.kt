package org.firstinspires.ftc.teamcode.robot

import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.arcrobotics.ftclib.hardware.motors.*
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

private typealias Applicant<T> = T.() -> Unit

/**
 * A class containing all devices on the robot.
 *
 * @property hardwareMap The hardware map to pull all devices from.
 */
class Hardware(private val hardwareMap: HardwareMap) {
    /**
     * A factory function to create a device.
     *
     * @param T The type of device.
     * @param name The name of the device.
     * @param factory A lambda of the creation of the device's class.
     * @param apply Any configurations to the device.
     * @return A `Pair` of the name and the lazily created device.
     */
    private fun <T> make(name: String, factory: (String) -> T, apply: Applicant<T> = {}) = name to lazy {
        factory(name).apply { apply(this) }
    }

    /**
     * Creates a device from the hardware map.
     *
     * @param T The type of device.
     * @param name The name of the device.
     * @param apply Any configurations to the device.
     */
    private inline fun <reified T> device(name: String, noinline apply: Applicant<T> = {}) = make(name,
        { hardwareMap.get(T::class.java, name) }, apply)

    /**
     * Creates a standard motor from the hardware map.
     *
     * @param name The name of the motor on the hardware map.
     * @param apply Any configurations to the motor.
     */
    private fun motor(name: String, apply: Applicant<MotorEx> = {}) = make(name,
        { MotorEx(hardwareMap, name) }, { setRunMode(Motor.RunMode.RawPower); apply() })

    /**
     * Creates a continuous servo from the hardware map.
     *
     * @param name The name of the continuous servo on the hardware map.
     * @param apply Any configurations to the servo.
     */
    private fun crServo(name: String, apply: Applicant<CRServo> = {}) = make(name,
        { CRServo(hardwareMap, name) }, apply)

    /**
     * Creates a standard servo from the hardware map.
     *
     * @param name The name of the servo.
     * @param apply Any configurations to the servo.
     */
    private fun servo(name: String, apply: Applicant<SimpleServo> = {}) = make(name,
        { SimpleServo(hardwareMap, name, 0.0, 360.0) }, apply)

    val devices: Map<String, Lazy<Any>> = mapOf(
        motor("left_front"),
        motor("right_front"),
        motor("left_back"),
        motor("right_back"),

        motor("intake"),

        make("outtake", { MotorGroup(
            MotorEx(hardwareMap, "outtake_l", Motor.GoBILDA.RPM_1620).apply { inverted = true },
            MotorEx(hardwareMap, "outtake_r", Motor.GoBILDA.RPM_1620).apply { inverted = true })},
            { setRunMode(Motor.RunMode.VelocityControl); setVeloCoefficients(1.0, 0.0, 0.0) }),

        motor("up") { inverted = true },

        crServo("upper_intake") { inverted = true },

        servo("feeder") { setRange(22.0 / 57.2958, 45.0 / 57.2958) },
        crServo("licker"),

        device<GoBildaPinpointDriver>("odometry") { setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD) },
        device<Limelight3A>("limelight") { pipelineSwitch(0) }
    )

    /**
     * Returns the device with the specified name and type.
     *
     * @param T The type of the device.
     * @param name The name of the device.
     * @return The device of the specified type.
     */
    inline operator fun <reified T> get(name: String): T {
        val device = devices[name]?.value ?: throw NoSuchElementException("Couldn't find $name")
        return device as? T ?: throw IllegalArgumentException("$name is not of type ${T::class.simpleName}")
    }
}

@Disabled
open class Setup(): OpMode() {
    protected lateinit var hardware: Hardware
    protected lateinit var drive: MecanumDrive

    protected lateinit var firstGamepad: GamepadEx
    protected lateinit var secondGamepad: GamepadEx

    private lateinit var limelight: Limelight3A
    private lateinit var odometry: GoBildaPinpointDriver

    override fun init() {
        hardware = Hardware(hardwareMap)
        drive = MecanumDrive(
            hardware.get<MotorEx>("left_front"),
            hardware.get<MotorEx>("right_front"),
            hardware.get<MotorEx>("left_back"),
            hardware.get<MotorEx>("right_back"))

        firstGamepad = GamepadEx(gamepad1)
        secondGamepad = GamepadEx(gamepad2)

        limelight = hardware["limelight"]
        odometry = hardware["odometry"]
    }

    override fun start() {
        limelight.start()
        odometry.resetPosAndIMU()
    }

    override fun loop() {
        limelight.updateRobotOrientation(odometry.getHeading(AngleUnit.DEGREES))
        odometry.update()

        telemetry.update()
    }
}