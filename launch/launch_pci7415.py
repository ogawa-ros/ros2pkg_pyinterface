from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2pkg_pyinterface',
            node_executable='pci7415',
            parameters=[
                {"rsw_id" : "0" },
                {"do_conf" :"[1, 1, 1, 1]" },

                {"use_axis" :"xyu" },

            #    {"x_pulse_conf" :"{'PULSE': 1, 'OUT': 0, 'DIR': 0, 'WAIT': 0, 'DUTY': 0}" },
                {"x_mode" :"jog"} ,
                {"x_clock" :"59"} ,
                {"x_acc_mode" :"acc_normal" },
                {"x_low_speed" :"10" },
                {"x_speed" :"5000" },
                {"x_acc" :"50" },
                {"x_dec" : "50"},
                {"x_step" :"1" },

                #{"y_pulse_conf" :"{'PULSE': 1, 'OUT': 0, 'DIR': 0, 'WAIT': 0, 'DUTY': 0}"},
                {"y_mode" :"jog" },
                {"y_clock" :"59"} ,
                {"y_acc_mode":"acc_normal" },
                {"y_low_speed" :"10" },
                {"y_speed" :"50000" },
                {"y_acc" :"50"} ,
                {"y_dec" :"50" },
                {"y_step" :"1" },

                #{"u_pulse_conf" :"{'PULSE': 1, 'OUT': 1, 'DIR': 1, 'WAIT': 0, 'DUTY': 0}" },
                {"u_mode" :"ptp"},
                {"u_clock" :"299"},
                {"u_acc_mode" :"acc_normal" },
                {"u_low_speed" :"10" },
                {"u_speed" :"200"},
                {"u_acc" :"50"} ,
                {"u_dec" :"50"},
                {"u_step" :"5000" }
            ]
        )],
       )
