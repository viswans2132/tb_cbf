number = 5

uav_model_default = "hb.xacro"
uav_model_special = "hb_white.xacro"  # Only for dcf1

ugv_model_default = "turtlebot3_waffle_pi.urdf.xacro"
ugv_model_special = "turtlebot3_waffle_pi_lcbf.urdf.xacro"  # Only for demo_turtle1

def generate_uav_block(name, is_special=False):
    model = uav_model_special if is_special else uav_model_default
    print(is_special, model)
    return f"""
  <group ns="{name}">
    
    <param name="robot_description" command="$(find xacro)/xacro '$(find ss_workshop)/models/urdf/{model}'"/>
    <param name="tf_prefix" value="{name}" />
    


    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" >
        <param name="tf_prefix" value="{name}" />
    </node>
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
  </group>
"""

def generate_ugv_block(name, is_special=False):
    model = ugv_model_special if is_special else ugv_model_default
    return f"""
  <group ns="{name}">
    <param name="robot_description" command="$(find xacro)/xacro '$(find ss_workshop)/models/urdf/{model}'"/>
    <param name="tf_prefix" value="{name}" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
      <param name="tf_prefix" value="{name}" />
    </node>
  </group>
  <node pkg="tf" type="static_transform_publisher" name="odom_{name}_broadcaster" args="0 0 0 0 0 0 1 odom {name}/odom 1000" />
"""

def generate_launch_file():
    content = """<launch>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/empty.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="false"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
"""

    # Add UAVs
    for i in range(1, number+1):
        name = f"dcf{i}"
        is_special = (i == 4)
        content += generate_uav_block(name, is_special)
        print(i, is_special)

    # Add UGVs
    for i in range(1, number+1):
        name = f"demo_turtle{i}"
        is_special = (i == 4)
        content += generate_ugv_block(name, is_special)
        print(i, is_special)


    # Static transforms
    content += """
  <node pkg="tf" type="static_transform_publisher" name="world_odom_broadcaster" args="0 0 0 0 0 0 1 world odom 1000" />
  <node pkg="tf" type="static_transform_publisher" name="world_map_broadcaster" args="0 0 0 0 0 0 1 world map 1000" />
</launch>
"""

    with open("multi_robot_desc.launch", "w") as f:
        f.write(content)

    print("✅ Launch file 'multi_robot_spawn.launch' created.")

if __name__ == "__main__":
    generate_launch_file()
