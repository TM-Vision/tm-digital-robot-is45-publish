import asyncio
import gc
import logging
import math  # type: ignore
import queue  # type: ignore
import random  # type: ignore
import socket  # type: ignore
import threading  # type: ignore
import time  # type: ignore
import traceback  # type: ignore
from datetime import datetime, timezone  # type: ignore
from typing import List, Optional  # type: ignore

import carb  # type: ignore
import numpy as np
import omni.kit.commands
import omni.kit.viewport.utility as vp_utils  # noqa
from isaacsim.core.api.world.world import World
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.stage import (
    add_reference_to_stage,
    clear_stage,
    close_stage,
    create_new_stage,
    is_stage_loading,
    update_stage_async,
)
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.surface_gripper._surface_gripper import (  # noqa
    Surface_Gripper,
    Surface_Gripper_Properties,
)
from pxr import Gf, Sdf, Usd, UsdGeom

# isort: off
from tmrobot.digital_robot.models.digital_camera import DigitalCamera  # type: ignore
from tmrobot.digital_robot.models.digital_robot import DigitalRobot  # type: ignore
from tmrobot.digital_robot.models.setting import ExtensionSetting  # type: ignore
from tmrobot.digital_robot.models.setting import RobotSetting  # type: ignore
from tmrobot.digital_robot.services.echo_client import EchoClient  # type: ignore
from tmrobot.digital_robot.services.ethernet_master import EthernetData  # type: ignore
from tmrobot.digital_robot.services.ethernet_master import EthernetMaster  # type: ignore
from tmrobot.digital_robot.services.move_command_server import MoveCommandServer  # type: ignore
from tmrobot.digital_robot.services.virtual_camera_server_secure import VirtualCameraServerSecure  # type: ignore
from tmrobot.digital_robot.ui import constants as const  # type: ignore
from tmrobot.digital_robot.ui.extension_ui import ExtensionUI  # type: ignore

# isort: on

logger = logging.getLogger(__name__)


class TMDigitalRobotExtension(omni.ext.IExt):
    def _initialize(self):
        # fmt: off
        self._extension_setting = ExtensionSetting()
        self._models = {}
        self._virtual_camera_thread: threading.Thread = None
        self._virtual_camera_server: VirtualCameraServerSecure = None
        self._move_command_server: MoveCommandServer = None
        # [prim path] -> (target_position, step_size, target_rotation, rotation_step_size)
        self._move_targets: dict[str, tuple] = {}
        self._dg_robots: dict[str, DigitalRobot] = {}
        self._dg_cameras: dict[str, dict[str, DigitalCamera]] = {}  # [tmflow ip][camera name]
        self._ethernet_masters: dict[str, EthernetMaster] = {}  # [robot name]
        self._ethernet_master_threads: dict[str, threading.Thread] = {}  # [robot name]
        self._motion_queue: queue.Queue = None
        self._latest_motion: dict[str, EthernetData] = {}  # [robot name]
        self._applied_positions: dict[str, np.ndarray] = {}  # [robot name]
        self._smoothing_tau = self._smoothing_time_constant()
        self._last_mirror_time = None
        self._robot_settings: List[RobotSetting] = []
        self._set_queue = queue.Queue()
        self._simulation_count = 0
        self._surface_gripper_state = 0
        self._surface_gripper = None
        self._workpiece_id = 0
        self._world: World = World()
        self._default_workpiece_position = Gf.Vec3d(0, 0.25, 0.5155)
        self._default_workpieces_prim_path = "/World/Accessories/Workpieces"
        # fmt: on

    def on_startup(self, ext_id):
        self._ext_id = ext_id

        # Force-hide the known Hydra warnings about corrupted UV primvars in the
        # robot meshes, regardless of whether extension.toml's
        # log.channels."omni.hydra" setting took effect.
        if const.LOG_LEVEL_OMNI_HYDRA:
            carb.settings.get_settings().set(
                "/log/channels/omni.hydra", const.LOG_LEVEL_OMNI_HYDRA
            )

        self._ext_ui = ExtensionUI(
            self._ext_id,
            self._on_start_service,
            self._on_stop_service,
            self._change_scene_camera_position,
            self._post_load_scene,
        )

        self._initialize()

    def on_shutdown(self):
        if hasattr(self, "_virtual_camera_server"):
            if self._virtual_camera_server is not None:
                asyncio.ensure_future(self._virtual_camera_server.stop())

        if hasattr(self, "_move_command_server"):
            if self._move_command_server is not None:
                asyncio.ensure_future(self._move_command_server.stop())

        if self._world.stage.GetPrimAtPath(Sdf.Path("/World")).IsValid():
            for robot in const.ROBOT_LIST:
                if self._world.scene.object_exists(robot):
                    self._world.scene.remove_object(robot)
                if robot in self._dg_robots:
                    self._dg_robots[robot].destroy_all_cameras()

        if len(self._ethernet_masters) != 0:
            for robot in self._robot_settings:
                self._ethernet_masters[robot.name].stop()
                self._ethernet_master_threads[robot.name].join(timeout=0)

        if self._world.physics_callback_exists("sim_step"):
            self._world.remove_physics_callback("sim_step")

        for handler in logger.handlers[:]:
            logger.removeHandler(handler)
            handler.close()

        logger.handlers.clear()

        if self._world is not None:
            self._world.stop()
            self._world.clear_all_callbacks()

        self._robot_settings = []
        self._ext_ui.clear()

        if is_stage_loading():
            clear_stage()
            close_stage()

        create_new_stage()
        gc.collect()
        return

    def _change_scene_camera_position(self):
        self._console("Change scene camera position")

        # Allow to customize the scene camera position as your need
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{const.SCENE_CAMERA_PATH}.xformOp:translate"),
            value=Gf.Vec3d(7, 4, 3),
            prev=None,
        )

        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{const.SCENE_CAMERA_PATH}.xformOp:orient"),
            value=Gf.Quatd(
                0.38,
                Gf.Vec3d(0.30, 0.53, 0.70),
            ),
            prev=None,
        )

    def _on_start_service(self):
        if not self._ext_ui.validate_form(self._world):
            return

        # _initialize() below drops the reference to any previous move command
        # server - if it's still running (e.g. a reload raced its shutdown), stop
        # it first so the new one isn't left fighting it for the same port.
        previous_move_command_server = getattr(self, "_move_command_server", None)

        self._initialize()
        self._ext_ui.change_action_mode(const.BUTTON_STOP_SERVICE)
        self._ext_ui.update_message("Services started")
        self._ext_ui.collapsed_robot_settings(False)
        self._ext_ui.on_save_scene()

        self._simulation_count = 0
        self._latest_motion = {}
        self._applied_positions = {}
        self._last_mirror_time = None

        if self._world.stage.GetPrimAtPath(
            Sdf.Path(self._default_workpieces_prim_path)
        ).IsValid():
            self._world.stage.RemovePrim(self._default_workpieces_prim_path)

        self._robot_settings = self._get_activated_robots_setting()

        # Check if TMSimulator services are available
        for setting in self._robot_settings:
            self._console(f"Add {setting.name} to the scene")

            # Check if the status of TMSimulator Virtual Camera API is Activated
            echo_client = EchoClient(setting.ip)
            if not echo_client.connectVirtualCameraAPI():
                warning_message = (
                    f"Can't connect to {setting.name} Virtual Camera API at IP: {setting.ip}, "
                    "please check if Virtual Camera API is enabled if you are using TMSimulator. "
                    "You can ignore this message if you are using TMFlow with physical robot."
                )
                logger.warning(warning_message)
                self._ext_ui.update_message(warning_message)

            # Check if the status of TMSimulator Ethernet Slave is Enabled
            if not self._is_service_on(setting.ip, const.PORT_ETHERNET):
                error_message = (
                    f"Can't connect to {setting.name} Ethernet at {setting.ip}:{const.PORT_ETHERNET}, "
                    "please check if the status of TMSimulator Ethernet Slave is Enabled"
                )
                logger.error(error_message)
                self._ext_ui.update_message(error_message)
                self._ext_ui.change_action_mode(const.BUTTON_START_SERVICE)
                self._ext_ui.collapsed_robot_settings(True)
                return

            # Create Digital Robots
            try:
                self._dg_robots[setting.name] = DigitalRobot(setting, self._world.stage)
                self._dg_cameras[setting.ip] = {}

                # Create a Camera list
                camera_list = self._dg_robots[setting.name].get_activated_cameras()
                for camera in camera_list:
                    self._dg_cameras[setting.ip][camera.get_serial_number()] = camera

                self._motion_queue = queue.Queue(maxsize=len(self._robot_settings))

                if not self._world.scene.object_exists(setting.name):
                    self._world.scene.add(self._dg_robots[setting.name].get_robot())

            except Exception as e:
                logger.error(f"Failed to add {setting.name}: {e}")
                logger.error(traceback.format_exc())
                self._ext_ui.change_action_mode(const.BUTTON_START_SERVICE)
                return

            # Delete existing workpieces
            if self._world.stage.GetPrimAtPath(
                Sdf.Path(self._default_workpieces_prim_path)
            ).IsValid():
                self._world.stage.RemovePrim(self._default_workpieces_prim_path)

            # Create new workpiece
            omni.kit.commands.execute(
                "CreatePrimWithDefaultXform",
                prim_type="Xform",
                prim_path=self._default_workpieces_prim_path,
                attributes={},
                select_new_prim=True,
            )

            # === (Surface Gripper Example) Uncomment the code below to control the surface gripper ===
            # The example is only for the first robot Robot01 with model TM12S
            # if self._robot_settings[0].name == const.ROBOT_LIST[0]:
            #     sgp = Surface_Gripper_Properties()
            #     sgp.parentPath = f"/World/{self._robot_settings[0].name}/{self._robot_settings[0].model.lower()}/body/flange_link"  # noqa
            #     sgp.offset.p.x = 0
            #     sgp.offset.p.z = 0.337
            #     sgp.offset.r = [0.7071, 0, 0.7071, 0]
            #     sgp.gripThreshold = 0.005
            #     sgp.forceLimit = 1.0e6
            #     sgp.torqueLimit = 1.0e7
            #     sgp.bendAngle = np.pi / 4
            #     sgp.stiffness = 1.0e8
            #     sgp.damping = 1.0e1
            #     sgp.retryClose = True
            #     self._surface_gripper = Surface_Gripper()
            #     self._surface_gripper.initialize(sgp)

            #     if self._is_prim_exist("/World/Accessories/sugar_box"):
            #         omni.kit.commands.execute(
            #             "ToggleActivePrims",
            #             stage_or_context=omni.usd.get_context().get_stage(),
            #             prim_paths=[Sdf.Path("/World/Accessories/sugar_box")],
            #             active=False,
            #         )
            #     self._spawn_workpiece()

        # Play the world
        async def _play_world_async():
            await self._world.initialize_simulation_context_async()

            self._world.add_physics_callback(
                "sim_step", callback_fn=self._on_simulation_step
            )
            await self._world.reset_async()
            await update_stage_async()
            await self._world.play_async()

        # Create a ethernet master threads for updating robot motion from ethernet slave
        async def _ethernet_master_async():

            robot_models_are_different = []

            for robot in self._robot_settings:
                if robot.activated:
                    self._ethernet_masters[robot.name] = EthernetMaster(
                        robot.name, robot.ip
                    )

                    actual_robot_model = self._ethernet_masters[
                        robot.name
                    ].get_robot_model()

                    if actual_robot_model in const.ROBOT_MODELS:
                        if actual_robot_model != robot.model:
                            robot_models_are_different.append(
                                f"{robot.name}: Virtual Robot model {robot.model} is connect to a "
                                f"TMSimulator/TMflow model {actual_robot_model}, which may cause unexpected behavior"
                            )

                        self._console(
                            f"{robot.name}({robot.model}) is connect to {robot.ip}({actual_robot_model})"
                        )

                    self._ethernet_master_threads[robot.name] = threading.Thread(
                        target=self._ethernet_masters[robot.name].receive_data,
                        args=(self._motion_queue,),
                    )

                    self._ethernet_master_threads[robot.name].start()

            if len(robot_models_are_different) > 0:
                self._ext_ui.update_message("\n".join(robot_models_are_different))

        # Create Virtual Camera gRPC Server
        self._virtual_camera_server = VirtualCameraServerSecure(
            self._set_queue, self._dg_cameras
        )

        # Create Move Command socket server, lets an external client push a
        # "prim_path, x, y, z, rx, ry, rz" request for the physics step callback
        # to drive via _move_to_target. Off unless enabled in .env - it opens an
        # unauthenticated listening port.
        async def _start_move_command_server_async():
            if previous_move_command_server is not None:
                await previous_move_command_server.stop()

            self._move_command_server = MoveCommandServer(
                self._move_targets,
                host=const.MOVE_COMMAND_HOST,
                port=const.PORT_MOVE_COMMAND,
            )
            await self._move_command_server.start()

        asyncio.ensure_future(self._virtual_camera_server.start())
        if const.MOVE_COMMAND_SERVER_ENABLED:
            asyncio.ensure_future(_start_move_command_server_async())
        asyncio.ensure_future(_ethernet_master_async())
        asyncio.ensure_future(_play_world_async())
        omni.kit.commands.execute("SelectNone")

        self._ext_ui.change_action_mode(const.BUTTON_STOP_SERVICE)

    def _on_simulation_step(self, step_size):
        self._simulation_count += 1

        try:
            motion: EthernetData = self._motion_queue.get_nowait()

            # Record the newest sample per robot; the actual joint write happens
            # in _mirror_joint_motion() on every physics step, not only on the
            # steps a packet arrives.
            self._latest_motion[motion.robot_name] = motion

            # === (Surface Gripper Example) Uncomment the code below to control the surface gripper ===
            # if motion.robot_name == const.ROBOT_LIST[0]:
            #     if self._surface_gripper_state != motion.ctrl_do[0]:
            #         self._surface_gripper_state = motion.ctrl_do[0]
            #         if self._surface_gripper_state == 1:
            #             self._surface_gripper.close()
            #             self._console("Surface Gripper suck")
            #             self._ethernet_masters[motion.robot_name].set_end_di(0, 0)
            #         if self._surface_gripper_state == 0:
            #             self._surface_gripper.open()
            #             self._console("Surface Gripper release")
            #             self._spawn_workpiece()
            #             self._ethernet_masters[motion.robot_name].set_end_di(0, 1)

        except queue.Empty:
            # logger.warning("Motion queue is empty")
            pass
        except Exception as e:  # noqa
            # logger.warning(f"{motion.robot_name}: failed to update robot motion: {e}")
            pass

        self._mirror_joint_motion()

        # Drive any pending move/rotate requests from the move command server.
        # _move_to_target only advances a prim by one step per call, so it must
        # keep being called every physics step until it reports the target has
        # been reached. The dict stays empty when the server is disabled.
        for prim_path, (
            target_position,
            move_step_size,
            target_rotation,
            rotation_step_size,
        ) in list(self._move_targets.items()):
            try:
                if self._move_to_target(
                    prim_path,
                    target_position,
                    move_step_size,
                    target_rotation,
                    rotation_step_size,
                ):
                    del self._move_targets[prim_path]
            except Exception as e:
                logger.warning(f"Failed to move {prim_path} to target: {e}")
                del self._move_targets[prim_path]

    def _smoothing_time_constant(self) -> float:
        """Turns the configured per-step factor into a time constant in seconds.

        MOTION_SMOOTHING_ALPHA is quoted as "fraction of the gap closed per step
        at 60 Hz" because that is what it means to whoever tunes it, but the
        filter has to be applied against elapsed time - see _mirror_joint_motion.
        Returns 0.0 when smoothing is switched off (alpha >= 1).
        """
        alpha = const.MOTION_SMOOTHING_ALPHA
        if alpha >= 1.0:
            return 0.0

        return -(1.0 / 60.0) / math.log(1.0 - alpha)

    def _smoothing_alpha_for(self, elapsed: float) -> float:
        """The filter factor that gives the configured time constant over this
        many seconds of real time. Long gaps converge to 1.0, i.e. snap to the
        target, which is what should happen after the app stalls.
        """
        if self._smoothing_tau <= 0.0 or elapsed <= 0.0:
            return 1.0

        return 1.0 - math.exp(-elapsed / self._smoothing_tau)

    def _mirror_joint_motion(self):
        """Moves each robot toward its latest Ethernet Slave target with a
        first-order low-pass filter, once per physics step.

        Running on steps with no fresh sample too keeps irregular packet timing
        from showing up as motion judder, and the ~0.05 deg dither in the
        reported angles is attenuated instead of replayed.
        See docs/troubleshooting/260804_arm_sharking.md.

        The filter runs against real elapsed time rather than a fixed amount per
        step. Ethernet Slave reports in real time, so a fixed per-step factor
        means the arm falls further behind the moment the app cannot hold 60 Hz
        - opening an extra camera viewport was enough to stretch the lag past
        the surface gripper's 5 mm grip threshold, so it stopped picking
        anything up. Deriving the factor from the measured step keeps the lag at
        the configured time constant whatever rate the loop manages.
        """
        now = time.perf_counter()
        elapsed = (
            0.0 if self._last_mirror_time is None else now - self._last_mirror_time
        )
        self._last_mirror_time = now
        alpha = self._smoothing_alpha_for(elapsed)

        for robot_name, latest in self._latest_motion.items():
            dg_robot = self._dg_robots.get(robot_name)
            if dg_robot is None:
                continue

            robot = dg_robot.get_robot()

            # The physics callback is registered before world.reset_async() has
            # finished, so the first steps arrive while the articulation handles
            # are still being set up. Writing then does nothing except log
            # "Articulation needs to be initialized." once per step per call -
            # a warning, not an exception, so the try below cannot swallow it.
            # Skipping the filter update too means the first initialized step
            # snaps to the reported pose instead of easing in from a stale one.
            if not robot.handles_initialized:
                continue

            target = np.asarray(latest.joint_radian, dtype=float)
            applied = self._applied_positions.get(robot_name)
            if applied is None:
                applied = target
            else:
                applied = applied + alpha * (target - applied)
            self._applied_positions[robot_name] = applied

            try:
                joint_positions = applied.tolist()

                if const.MOTION_MIRROR_MODE == "kinematic":
                    # Teleport the joints to the smoothed target. Removes the
                    # last of the jitter, at the cost of ignoring physics.
                    robot.set_joint_positions(joint_positions)
                    robot.set_joint_velocities([0.0] * len(joint_positions))

                # Driven in both modes - in kinematic mode this keeps the drive
                # targets aligned so the drives never fight the written state.
                dg_robot.apply_action(
                    ArticulationAction(joint_positions=joint_positions)
                )
            except Exception:  # noqa
                pass

    def _on_stop_service(self):
        async def _on_stop_service_async():
            self._ext_ui.change_action_mode(const.BUTTON_DISABLE_ALL)
            self._ext_ui.update_message("Services stopping...")
            self._ext_ui.collapsed_robot_settings(True)

            # Release the gripper before the world stops. Stopping mid-grip
            # leaves the D6 joint alive; it is only torn down once the next
            # start drops the old Surface_Gripper, which by then logs
            # "DcDestroyD6Joint: Function called while not simulating" and can
            # leave the fresh gripper unable to pick anything up. Has to happen
            # before stop_async() - destroying the joint afterwards hits the
            # same not-simulating path. No-op while the surface gripper example
            # below is left commented out, since the handle stays None.
            if self._surface_gripper is not None:
                self._surface_gripper.open()
                self._surface_gripper_state = 0
                self._console("Surface Gripper released before stopping")

            await self._world.stop_async()

            if self._world.stage.GetPrimAtPath(
                Sdf.Path(self._default_workpieces_prim_path)
            ).IsValid():
                self._world.stage.RemovePrim(self._default_workpieces_prim_path)

            for robot in self._robot_settings:
                self._ethernet_masters[robot.name].stop()
                self._world.scene.remove_object(robot.name)
                self._ethernet_master_threads[robot.name].join(timeout=0)
                if robot.name in self._dg_robots:
                    self._dg_robots[robot.name].destroy_all_cameras()

            if self._world.physics_callback_exists("sim_step"):
                self._world.remove_physics_callback("sim_step")

            if hasattr(self, "_virtual_camera_server"):
                if self._virtual_camera_server is not None:
                    await self._virtual_camera_server.stop()

            if hasattr(self, "_move_command_server"):
                if self._move_command_server is not None:
                    await self._move_command_server.stop()

            # self._stop_all_async_functions()
            self._ext_ui.change_action_mode(const.BUTTON_START_SERVICE)
            self._console("Services stopped")
            self._ext_ui.update_message("Services stopped")

        asyncio.ensure_future(_on_stop_service_async())

    def _stop_all_async_functions(self):
        async def _stop_all_async_functions_async():
            tasks = [
                task
                for task in asyncio.all_tasks()
                if task is not asyncio.current_task()
                and task.get_coro().__name__ in ["_server_main_loop", "start"]
            ]
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

        asyncio.ensure_future(_stop_all_async_functions_async())

    def _get_activated_robots_setting(self) -> List[RobotSetting]:
        active_robots = []
        for robot_name in const.ROBOT_LIST:
            setting = self._ext_ui._on_load_setting()
            if setting.robots_setting[robot_name].activated:
                active_robots.append(setting.robots_setting[robot_name])
        return active_robots

    def _is_service_on(self, ip, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.settimeout(1)  # Set the timeout value in seconds
                s.connect((ip, port))
                s.close()
                return True
            except socket.error as e:
                s.close()
                logger.error(f"{ip}:{port} is not available. exception: {e}")
                return False

    def _spawn_workpiece(self):
        workpieces_prim = self._world.stage.GetPrimAtPath(
            Sdf.Path(self._default_workpieces_prim_path)
        )

        # Check if the workpiece is already spawned, if yes, return
        spawn_position = self._default_workpiece_position
        for workpiece in workpieces_prim.GetChildren():
            path: str = workpiece.GetPath()
            wp_prim = self._world.stage.GetPrimAtPath(Sdf.Path(path))
            wp_xformable = UsdGeom.Xformable(wp_prim)
            wp_transform = wp_xformable.GetLocalTransformation()
            wp_translation = wp_transform.ExtractTranslation()
            x = round(wp_translation[0], 2)
            y = round(wp_translation[1], 2)
            if spawn_position[0] == x and spawn_position[1] == y:
                return

        self._workpiece_id += 1
        workpiece_prim_path = (
            f"{self._default_workpieces_prim_path}/workpiece_{self._workpiece_id}"
        )

        # Create a workpiece
        absolute_path = f"{const.EXTENSION_ROOT_PATH}/assets/worlds/accessories/workpiece/004_sugar_box/004_sugar_box.usd"  # noqa
        workpiece_prim = add_reference_to_stage(
            usd_path=absolute_path,
            prim_path=workpiece_prim_path,
        ).GetPrim()

        # Place the workpiece on the table
        # fmt: off
        workpiece_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(self._default_workpiece_position))
        workpiece_prim.GetAttribute("xformOp:scale").Set(Gf.Vec3f(0.5, 0.5, 0.5))
        workpiece_prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(0, 0, random.uniform(0, 360)))
        # fmt: on

        omni.kit.commands.execute(
            "SetRigidBody",
            path=Sdf.Path(workpiece_prim_path),
            approximationShape="convexHull",
            kinematic=False,
        )

        self._console(f"{workpiece_prim_path} is spawned")

    # === Common functions ===
    def _is_prim_exist(self, prim_path: str) -> bool:
        prim = self._world.stage.GetPrimAtPath(Sdf.Path(prim_path))
        return prim.IsValid()

    def _get_prim_size(self, prim_path: str) -> Optional[Gf.Vec3d]:
        """Returns the prim's world-space bounding box size in meters, or None
        when the prim doesn't exist. Callers that want it logged should print it
        themselves - _set_prim_size() calls this twice per invocation.
        """
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_]
        )
        bbox_cache.Clear()
        prim = self._world.stage.GetPrimAtPath(Sdf.Path(prim_path))
        if not prim.IsValid():
            self._console(f"Prim not found, skip getting size: {prim_path}")
            return None
        prim_bbox = bbox_cache.ComputeWorldBound(prim)
        prim_range = prim_bbox.ComputeAlignedRange()
        prim_size: Gf.Vec3d = prim_range.GetSize()

        return prim_size

    def _set_prim_size(self, prim_path: str, target_size: tuple) -> None:
        # Reset the prim scale to 1, 1, 1
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{prim_path}.xformOp:scale"),
            value=Gf.Vec3d(1, 1, 1),
            prev=None,
        )

        # Get the original size of the prim
        origin_size = self._get_prim_size(prim_path)
        if origin_size is None:
            self._console(f"Prim not found, skip setting size: {prim_path}")
            return

        if 0 in (origin_size[0], origin_size[1], origin_size[2]):
            self._console(f"Prim has zero-sized bounds, skip setting size: {prim_path}")
            return

        x = target_size[0] / origin_size[0]
        y = target_size[1] / origin_size[1]
        z = target_size[2] / origin_size[2]

        # Set the prim scale to the target size by calculating the ratio
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{prim_path}.xformOp:scale"),
            value=Gf.Vec3d(x, y, z),
            prev=None,
        )

        # Get the changed size of the prim
        changed_size = self._get_prim_size(prim_path)
        x = f"{changed_size[0]:.4f}"
        y = f"{changed_size[1]:.4f}"
        z = f"{changed_size[2]:.4f}"
        self._console(f"Set prim size(Meter): x={x}, y={y}, z={z} {prim_path}")

    def _get_rotation_xform_ops(self, xformable: UsdGeom.Xformable) -> tuple:
        """Returns the (orient, rotateXYZ) ops that are actually listed in the prim's
        xformOpOrder - either may be None. Ops are matched by type rather than by
        name so namespaced ops (e.g. xformOp:orient:unitsResolve) are found too.
        """
        orient_op = None
        euler_op = None

        for op in xformable.GetOrderedXformOps():
            op_type = op.GetOpType()
            if op_type == UsdGeom.XformOp.TypeOrient and orient_op is None:
                orient_op = op
            elif op_type == UsdGeom.XformOp.TypeRotateXYZ and euler_op is None:
                euler_op = op

        return orient_op, euler_op

    def _euler_xyz_to_rotation(self, euler_xyz: tuple) -> Gf.Rotation:
        """Builds the rotation that xformOp:rotateXYZ produces for these degrees.
        USD applies X, then Y, then Z, which in Gf's row-vector convention composes
        as Rx * Ry * Rz - not the reverse.
        """
        rx, ry, rz = euler_xyz

        return (
            Gf.Rotation(Gf.Vec3d.XAxis(), rx)
            * Gf.Rotation(Gf.Vec3d.YAxis(), ry)
            * Gf.Rotation(Gf.Vec3d.ZAxis(), rz)
        )

    def _rotation_to_euler_xyz(self, rotation: Gf.Rotation) -> tuple:
        """Inverse of _euler_xyz_to_rotation. Decompose returns the angles in the
        order its axes are passed, so ask for Z, Y, X and hand them back flipped.
        """
        rz, ry, rx = rotation.Decompose(
            Gf.Vec3d.ZAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.XAxis()
        )

        return rx, ry, rz

    def _read_current_rotation(self, orient_op, euler_op) -> Gf.Rotation:
        """Current rotation of whichever op drives the prim, or identity when neither
        op exists or has a value authored yet.
        """
        if orient_op is not None and orient_op.Get() is not None:
            return Gf.Rotation(Gf.Quatd(orient_op.Get()))

        if euler_op is not None and euler_op.Get() is not None:
            return self._euler_xyz_to_rotation(euler_op.Get())

        # Gf.Rotation() (no-arg default ctor) leaves its internal state
        # uninitialized rather than identity - build identity explicitly.
        return Gf.Rotation(Gf.Vec3d.ZAxis(), 0.0)

    def _quat_for_precision(self, quat: Gf.Quatd, precision):
        """Usd.Attribute.Set() rejects a value whose type doesn't match the authored
        attribute, so match the op's precision instead of always writing quatf.
        """
        if precision == UsdGeom.XformOp.PrecisionHalf:
            return Gf.Quath(quat)

        if precision == UsdGeom.XformOp.PrecisionFloat:
            return Gf.Quatf(quat)

        return Gf.Quatd(quat)

    def _vec3_for_precision(self, vec3: tuple, precision):
        """Same precision matching as _quat_for_precision, for the Euler angle ops."""
        if precision == UsdGeom.XformOp.PrecisionHalf:
            return Gf.Vec3h(*vec3)

        if precision == UsdGeom.XformOp.PrecisionFloat:
            return Gf.Vec3f(*vec3)

        return Gf.Vec3d(*vec3)

    def _step_toward(self, current: float, target: float, step: float) -> float:
        """One clamped step from current toward target - never overshoots.

        Overshooting would flip the direction check on the next call and the prim
        would oscillate around the target forever, so the step is clamped to the
        remaining distance.
        """
        if current < target:
            return min(current + step, target)

        if current > target:
            return max(current - step, target)

        return current

    def _move_to_target(
        self,
        prim_path: str,
        target_position: tuple = None,
        step_size=0.001,
        target_rotation: tuple = None,
        rotation_step_size=1.0,
    ) -> bool:
        """Steps a prim toward a target position (meters) and/or rotation (Euler XYZ
        degrees) by one increment; call every frame until it returns True. Either
        target may be omitted to only move or only rotate.

        Values are written straight to the attributes rather than through the
        ChangeProperty command: this runs every physics step, and every command
        would push an entry onto Kit's unbounded undo stack for a single frame of
        motion - which is both a leak and meaningless to undo.
        """
        prim = get_prim_at_path(prim_path)
        position_done = True
        rotation_done = True

        if target_position is not None:
            translate_attribute = prim.GetPrim().GetAttribute("xformOp:translate")

            if not translate_attribute.IsValid() or translate_attribute.Get() is None:
                self._console(f"No xformOp:translate on {prim_path}, skip moving")
            else:
                # fmt: off
                cx, cy, cz = [round(coord, 4) for coord in translate_attribute.Get()]
                tx, ty, tz = [round(coord, 4) for coord in target_position]
                # fmt: on

                position_done = cx == tx and cy == ty and cz == tz

                if not position_done:
                    cx = self._step_toward(cx, tx, step_size)
                    cy = self._step_toward(cy, ty, step_size)
                    cz = self._step_toward(cz, tz, step_size)

                    translate_attribute.Set(Gf.Vec3d(cx, cy, cz))

        if target_rotation is not None:
            xformable = UsdGeom.Xformable(prim.GetPrim())
            # Resolve the rotation op from xformOpOrder, not from the prim's
            # attributes: an xformOp:orient attribute that is authored but missing
            # from xformOpOrder is ignored by the renderer, and writing to it makes
            # omni.hydra log "cannot find xform op xformOp:orient" every physics
            # step while the prim silently refuses to rotate.
            orient_op, euler_op = self._get_rotation_xform_ops(xformable)

            target_rot = self._euler_xyz_to_rotation(target_rotation)
            current_rot = self._read_current_rotation(orient_op, euler_op)

            diff_rot = target_rot * current_rot.GetInverse()
            diff_angle = abs(diff_rot.GetAngle())

            rotation_done = diff_angle < 1e-3

            if not rotation_done:
                step_angle = min(rotation_step_size, diff_angle)
                step_rot = Gf.Rotation(diff_rot.GetAxis(), step_angle) * current_rot

                if orient_op is not None:
                    orient_op.Set(
                        self._quat_for_precision(
                            step_rot.GetQuat(), orient_op.GetPrecision()
                        )
                    )
                elif euler_op is not None:
                    # The asset drives this prim with Euler angles - keep using its
                    # op instead of adding a competing orient op to the stack.
                    euler_op.Set(
                        self._vec3_for_precision(
                            self._rotation_to_euler_xyz(step_rot),
                            euler_op.GetPrecision(),
                        )
                    )
                else:
                    # No rotation op in xformOpOrder yet - author one properly (float
                    # precision, matching the Gf.Quatf value) so it's actually
                    # applied, instead of setting an orphaned attribute.
                    xformable.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
                        Gf.Quatf(step_rot.GetQuat())
                    )

        return position_done and rotation_done

    def _console(self, message: str):
        current_time = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S")

        print(f"{current_time} [Info] [tmrobot.digital_robot] {message}")
        logger.info(message)

    def _post_load_scene(self):
        # Do your custom actions after loading the scene, for example get the size of the prim
        pass
