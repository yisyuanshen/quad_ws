import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 取得 URDF 檔案路徑
    # 這裡我們為了開發方便，暫時直接指定絕對路徑，或者透過 package share 尋找
    # 為了讓您現在就能跑，我們用 "直接指定路徑" 的方式（開發階段最快）
    urdf_file = os.path.join(
        os.getcwd(), 'src', 'robot_description', 'urdf', 'robot.urdf'
    )

    # 讀取 URDF 內容
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 2. 設定 Robot State Publisher (負責發布 TF 座標變換)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # 3. 設定 Joint State Publisher GUI (負責顯示滑桿讓您玩)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    # 4. 設定 Rviz2 (負責顯示畫面)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node
    ])