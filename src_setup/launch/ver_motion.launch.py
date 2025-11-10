import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    cpp_prefix    = get_package_prefix('src_cpp')
    plugin_prefix = get_package_prefix('src_setup')

    set_res_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        f"{cpp_prefix}/share:{os.path.join(get_package_share_directory('src_cpp'),'models')}:{os.path.join(get_package_share_directory('src_cpp'),'worlds')}"
    )
    set_sys_plugins = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        f"{plugin_prefix}/lib"
    )

    ld = LaunchDescription()
    ld.add_action(set_res_path)
    ld.add_action(set_sys_plugins)
    return ld
