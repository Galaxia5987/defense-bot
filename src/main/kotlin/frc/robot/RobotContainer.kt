package frc.robot

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.volts
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.drive.DriveCommands
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the [Robot] periodic methods (other than the scheduler
 * calls). Instead, the structure of the robot (including subsystems, commands,
 * and trigger mappings) should be declared here.
 */
object RobotContainer {
    private val driverController = CommandXboxController(0)

    val elevator = Elevator()

    val gripperMotor = TalonFX(16)

    private val gripperConfig =
        TalonFXConfiguration().apply {
            MotorOutput =
                MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
        }

    init {
        gripperMotor.configurator.apply(gripperConfig)

        configureButtonBindings()
        configureDefaultCommands()

        if (CURRENT_MODE == Mode.SIM && USE_MAPLE_SIM)
            SimulatedArena.getInstance().resetFieldForAuto()

        enableAutoLogOutputFor(this)
    }

    @AutoLogOutput(key = "MapleSimPose")
    private fun getMapleSimPose(): Pose2d? =
        driveSimulation?.simulatedDriveTrainPose

    private fun configureDefaultCommands() {
        swerveDrive.defaultCommand =
            DriveCommands.joystickDrive(
                swerveDrive,
                { driverController.leftY },
                { driverController.leftX },
                { -driverController.rightX * 0.8 }
            )
    }

    private fun configureButtonBindings() {
        driverController.start().onTrue(resetRobotRotation())

        driverController.povUp().whileTrue(runElevator(3.0.volts))
        driverController.povDown().whileTrue(runElevator((-3.0).volts))

        driverController.y().whileTrue(goTo(Units.Meters.zero(), Units.Meters.zero(), 0.0.degrees))
    }

    fun getAutonomousCommand(): Command = goTo(Units.Meters.zero(), Units.Meters.zero(), 0.0.degrees)
}
