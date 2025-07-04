# === (Prismatic Joint Gripper Example)  ===
# === (extension_prismatic_joint_gripper)  ===
# === (.workspace: 20250719_170219_prismatic_joint_gripper)  ===
# === (TMflow: prismatic_joint_gripper_v1) ===
import asyncio
import gc
import logging
import queue  # type: ignore
import random  # type: ignore
import socket  # type: ignore
import threading  # type: ignore
import traceback  # type: ignore
from datetime import datetime, timezone  # type: ignore
from typing import List  # type: ignore

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
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

# isort: off
from tmrobot.digital_robot.models.digital_camera import DigitalCamera  # type: ignore
from tmrobot.digital_robot.models.digital_robot import DigitalRobot  # type: ignore
from tmrobot.digital_robot.models.setting import ExtensionSetting  # type: ignore
from tmrobot.digital_robot.models.setting import RobotSetting  # type: ignore
from tmrobot.digital_robot.services.echo_client import EchoClient  # type: ignore
from tmrobot.digital_robot.services.ethernet_master import EthernetData  # type: ignore
from tmrobot.digital_robot.services.ethernet_master import EthernetMaster  # type: ignore
from tmrobot.digital_robot.services.virtual_camera_server_secure import VirtualCameraServerSecure  # type: ignore
from tmrobot.digital_robot.ui import constants as const  # type: ignore
from tmrobot.digital_robot.ui.extension_ui import ExtensionUI  # type: ignore

# isort: on

logger = logging.getLogger(__name__)
# viewport = vp_utils.get_active_viewport()


class TMDigitalRobotExtension(omni.ext.IExt):
    def _initialize(self):
        # fmt: off
        logger.info(f"ADVANCED_MODE: {const.ADVANCED_MODE}")
        self._extension_setting = ExtensionSetting()
        self._models = {}
        self._virtual_camera_thread: threading.Thread = None
        self._virtual_camera_server: VirtualCameraServerSecure = None
        self._dg_robots: dict[str, DigitalRobot] = {}
        self._dg_cameras: dict[str, dict[str, DigitalCamera]] = {}  # [tmflow ip][camera name]
        self._ethernet_masters: dict[str, EthernetMaster] = {}  # [robot name]
        self._ethernet_master_threads: dict[str, threading.Thread] = {}  # [robot name]
        self._motion_queue: queue.Queue = None
        self._robot_settings: List[RobotSetting] = []
        self._set_queue = queue.Queue()
        self._simulation_count = 0
        self.is_server_right = True
        self._fps_accumulated = 0
        self._surface_gripper_state = 0
        self._surface_gripper = None
        self._parallel_gripper = None
        self._workpiece_id = 0
        self._world: World = World()
        self._default_workpiece_position = Gf.Vec3d(0, 0.25, 0.5155)
        self._default_workpieces_prim_path = "/World/Accessories/Workpieces"
        # fmt: on

    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._ext_ui = ExtensionUI(
            self._ext_id,
            self._on_start_service,
            self._on_stop_service,
            self._change_scene_camera_position,
            self._post_load_scene,
        )

        self._initialize()

    def on_shutdown(self):

        if self._world.stage.GetPrimAtPath(Sdf.Path("/World")).IsValid():
            for robot in const.ROBOT_LIST:
                if self._world.scene.object_exists(robot):
                    self._world.scene.remove_object(robot)

        if len(self._ethernet_masters) != 0:
            for robot in self._robot_settings:
                self._ethernet_masters[robot.name].stop()
                self._ethernet_master_threads[robot.name].join(timeout=0)

        if self._world.physics_callback_exists("sim_step"):
            self._world.remove_physics_callback("sim_step")

        try:

            self._console("Services stopped")
        except Exception as e:
            logger.error(f"Services stopped with error: {e}")

        for handler in logger.handlers[:]:
            logger.removeHandler(handler)
            handler.close()

        logger.handlers.clear()

        if self._world is not None:
            self._world.stop()
            self._world.clear_all_callbacks()
            self._current_tasks = None

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
            "TransformPrimCommand",
            path=Sdf.Path("/World/scene_camera"),
            new_transform_matrix=Gf.Matrix4d(
                0.7469087288392622,
                0.6645784067443482,
                0.021514926745425766,
                0.0,
                -0.22681476346080937,
                0.22423032998539627,
                0.9477741409169473,
                0.0,
                0.6250459294003397,
                -0.7127806818396414,
                0.31821578486348145,
                0.0,
                1.421226871137547,
                -2.377826311651238,
                1.4606384062114774,
                1.0,
            ),
            old_transform_matrix=None,
            time_code=Usd.TimeCode.Default(),
            had_transform_at_key=False,
            usd_context_name="",
        )

    def _on_start_service(self):
        if not self._ext_ui.validate_form(self._world):
            return

        self._initialize()
        self._ext_ui.change_action_mode(const.BUTTON_STOP_SERVICE)
        self._ext_ui.update_message("Services started")
        self._ext_ui.collapsed_robot_settings(False)
        self._ext_ui.on_save_scene()

        self._simulation_count = 0

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

        asyncio.ensure_future(self._virtual_camera_server.start())
        asyncio.ensure_future(_ethernet_master_async())
        asyncio.ensure_future(_play_world_async())
        omni.kit.commands.execute("SelectNone")

        self._ext_ui.change_action_mode(const.BUTTON_STOP_SERVICE)

    def _on_simulation_step(self, step_size):
        self._simulation_count += 1

        try:
            motion: EthernetData = self._motion_queue.get_nowait()

            # motion.joint_radian = motion.joint_radian.extend([10, 10, 10, 10, 10, 10, 10, 10])
            self._dg_robots[motion.robot_name].apply_action(
                ArticulationAction(joint_positions=motion.joint_radian)
            )

            if motion.robot_name == const.ROBOT_LIST[0]:
                if self._surface_gripper_state != motion.ctrl_do[0]:
                    self._surface_gripper_state = motion.ctrl_do[0]

                    if self._surface_gripper_state == 1:

                        omni.kit.commands.execute(
                            "ChangeProperty",
                            prop_path=Sdf.Path(
                                "/World/Robot01/Robotiq_Hand_E_edit/Slider_1.drive:linear:physics:targetPosition"
                            ),
                            value=0,
                            prev=None,
                            usd_context_name=omni.usd.get_context().get_stage(),
                        )

                        omni.kit.commands.execute(
                            "ChangeProperty",
                            prop_path=Sdf.Path(
                                "/World/Robot01/Robotiq_Hand_E_edit/Slider_2.drive:linear:physics:targetPosition"
                            ),
                            value=0,
                            prev=None,
                            usd_context_name=omni.usd.get_context().get_stage(),
                        )

                        self._console("Parallel Gripper is closed")

                    if self._surface_gripper_state == 0:
                        omni.kit.commands.execute(
                            "ChangeProperty",
                            prop_path=Sdf.Path(
                                "/World/Robot01/Robotiq_Hand_E_edit/Slider_1.drive:linear:physics:targetPosition"
                            ),
                            value=0.025,
                            prev=None,
                            usd_context_name=omni.usd.get_context().get_stage(),
                        )

                        omni.kit.commands.execute(
                            "ChangeProperty",
                            prop_path=Sdf.Path(
                                "/World/Robot01/Robotiq_Hand_E_edit/Slider_2.drive:linear:physics:targetPosition"
                            ),
                            value=0.025,
                            prev=None,
                            usd_context_name=omni.usd.get_context().get_stage(),
                        )

                        self._console("Parallel Gripper is opened")

                        omni.kit.commands.execute(
                            "ChangeProperty",
                            prop_path=Sdf.Path(
                                "/World/Accessories/work_piece.xformOp:translate"
                            ),
                            value=Gf.Vec3d(0, 0.25, 0.51551),
                            prev=None,
                        )

                        omni.kit.commands.execute(
                            "ChangeProperty",
                            prop_path=Sdf.Path(
                                "/World/Accessories/work_piece.xformOp:orient"
                            ),
                            value=Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0)),
                            prev=None,
                            usd_context_name=omni.usd.get_context().get_stage(),
                        )

        except queue.Empty:
            logger.warning("Motion queue is empty")
            pass
        except Exception as e:  # noqa
            # logger.warning(f"{motion.robot_name}: failed to update robot motion: {e}")
            # print(json.dumps(motion.__dict__))
            pass

    def _on_stop_service(self):
        async def _on_stop_service_async():
            self._ext_ui.change_action_mode(const.BUTTON_DISABLE_ALL)
            self._ext_ui.update_message("Services stopping...")
            self._ext_ui.collapsed_robot_settings(True)

            await self._world.stop_async()

            if self._world.stage.GetPrimAtPath(
                Sdf.Path(self._default_workpieces_prim_path)
            ).IsValid():
                self._world.stage.RemovePrim(self._default_workpieces_prim_path)

            for robot in self._robot_settings:
                self._ethernet_masters[robot.name].stop()
                self._world.scene.remove_object(robot.name)
                self._ethernet_master_threads[robot.name].join(timeout=0)

            if self._world.physics_callback_exists("sim_step"):
                self._world.remove_physics_callback("sim_step")

            if hasattr(self, "_virtual_camera_server"):
                if self._virtual_camera_server is not None:
                    await self._virtual_camera_server.stop()

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

    def _get_prim_size(self, prim_path: str) -> Gf.Vec3d:
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_]
        )
        bbox_cache.Clear()
        prim = self._world.stage.GetPrimAtPath(Sdf.Path(prim_path))
        prim_bbox = bbox_cache.ComputeWorldBound(prim)
        prim_range = prim_bbox.ComputeAlignedRange()
        prim_size: Gf.Vec3d = prim_range.GetSize()

        self._console(f"Get prim size(Meter): {prim_size} {prim_path}")

        return prim_size

    def _set_prim_size(self, prim_path, target_size) -> None:
        # Reset the prim scale to 1, 1, 1
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{prim_path}.xformOp:scale"),
            value=Gf.Vec3d(1, 1, 1),
            prev=None,
        )

        # Get the original size of the prim
        origin_size = self._get_prim_size(prim_path)
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

    def _move_to_target(
        self, prim_path: str, target_position: tuple, step_size=0.001
    ) -> bool:
        prim = get_prim_at_path(prim_path)
        # fmt: off
        cx, cy, cz = [round(coord, 4) for coord in prim.GetPrim().GetAttribute("xformOp:translate").Get()]
        tx, ty, tz = [round(coord, 4) for coord in target_position]
        # fmt: on

        if cx == tx and cy == ty and cz == tz:
            return True

        if cx < tx:
            cx += step_size
        elif cx > tx:
            cx -= step_size

        if cy < ty:
            cy += step_size
        elif cy > ty:
            cy -= step_size

        if cz < tz:
            cz += step_size
        elif cz > tz:
            cz -= step_size

        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{prim_path}.xformOp:translate"),
            value=Gf.Vec3d(cx, cy, cz),
            prev=None,
        )

        return False

    def _console(self, message):
        current_time = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S")

        print(f"{current_time} [Info] [tmrobot.digital_robot] {message}")
        logger.info(message)

    def _get_weight(self, prim_path: str) -> float:
        prim = self._world.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            self._console(f"Prim not found at path: {prim_path}")
            return 0.0

        # Use the stage and path to get the MassAPI
        mass_api = UsdPhysics.MassAPI.Get(self._world.stage, prim.GetPath())
        if not mass_api:
            self._console(f"MassAPI not available for prim: {prim_path}")
            return 0.0

        # Get the mass attribute
        mass_attr = mass_api.GetMassAttr()
        if mass_attr and mass_attr.HasAuthoredValue():
            mass = mass_attr.Get()
            self._console(f"The mass of the prim is {mass} kg.")
            return mass
        else:
            self._console(
                f"The mass attribute is not authored for this prim: {prim_path}"
            )
            return 0.0

    def _post_load_scene(self):
        # Do your custom actions after loading the scene
        pass
