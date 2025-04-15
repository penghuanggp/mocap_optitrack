# Copyright (c) 2019, Samsung Electronics Inc., Vinnam Kim
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch.actions import LogInfo, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import Shutdown


def generate_launch_description():
    default_config_path = os.path.join(
        get_package_share_directory("mocap_optitrack"), "config", "mocap.yaml"
    )

    declare_config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_path,
        description="Path to the mocap configuration file",
    )

    config_path = LaunchConfiguration("config_path")

    # Check if config file exists
    check_config = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=declare_config_path_arg,
            on_start=[
                LogInfo(msg=f"Checking if config file exists at {config_path}"),
                (
                    EmitEvent(event=Shutdown(reason="Config file not found"))
                    if UnlessCondition(
                        f"'{config_path}' == '' and os.path.isfile('{config_path}')"
                    )
                    else LogInfo(msg="Config file found, proceeding...")
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            declare_config_path_arg,
            check_config,
            Node(
                package="mocap_optitrack",
                executable="mocap_node",
                name="mocap_node",
                parameters=[config_path],
                output="screen",
            ),
        ]
    )


generate_launch_description()
