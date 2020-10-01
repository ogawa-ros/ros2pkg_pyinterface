node_name = 'pci7415_driver'

import sys
import time
import queue
import rclpy
from functools import partial

import pyinterface
print(pyinterface.__version__)
import std_msgs.msg

class pci7415_driver(object):


    def __init__(self, rsw_id, params):
        #（__init__における handler の組み込み場所）
        self.node = rclpy.create_node(node_name)

        self.node.declare_parameter('~node_name')
        self.node.declare_parameter('~rsw_id')
        self.node.declare_parameter('~use_axis')

        self.name = self.node.get_parameter('~node_name').get_parameter_value().string_value
        self.rsw_id = self.node.get_parameter('~rsw_id').get_parameter_value().string_value
        self._use_axis = self.node.get_parameter('~use_axis').get_parameter_value().string_value

        self.params = {}
        for ax in self._use_axis:
            params[ax] = {}
            #params[ax]['mode'] = rospy.get_param('~{ax}_mode'.format(**locals()), default_mode)
            #params[ax]['pulse_conf'] = [eval(rospy.get_param('~{ax}_pulse_conf'.format(**locals()), default_pulse_conf))] #evalとは？？？
            self.node.declare_parameter('~{ax}_mode')
            self.node.declare_parameter('~{ax}_pulse_conf')

            self.params[ax]['mode'] = self.node.get_parameter('~{ax}_mode').get_parameter_value().string_value
            self.params[ax]['pulse_conf'] = self.node.get_parameter('~{ax}_pulse_conf').get_parameter_value().string_value


            self.mp = {}
                # mp['clock'] = rospy.get_param('~{ax}_clock'.format(**locals()), default_clock)
                # mp['acc_mode'] = rospy.get_param('~{ax}_acc_mode'.format(**locals()), default_acc_mode)
                # mp['low_speed'] = rospy.get_param('~{ax}_low_speed'.format(**locals()), default_low_speed)
                # mp['speed'] = rospy.get_param('~{ax}_speed'.format(**locals()), default_speed)
                # mp['acc'] = rospy.get_param('~{ax}_acc'.format(**locals()), default_acc)
                # mp['dec'] = rospy.get_param('~{ax}_dec'.format(**locals()), default_dec)
                # mp['step'] = rospy.get_param('~{ax}_step'.format(**locals()), default_step)

            self.node.declare_parameter('~{ax}_clock')
            self.node.declare_parameter('~{ax}_acc_mode')
            self.node.declare_parameter('~{ax}_low_speed')
            self.node.declare_parameter('~{ax}_speed')
            self.node.declare_parameter('~{ax}_acc')
            self.node.declare_parameter('~{ax}_dec')
            self.node.declare_parameter('~{ax}_step')


            self.mp['clock'] = self.node.get_parameter('~{ax}_clock').get_parameter_value().string_value #mpにはselfはいらない？？　paramsに格納するから
            self.mp['acc_mode'] = self.node.get_parameter('~{ax}_acc_mode').get_parameter_value().string_value
            self.mp['low_speed'] = self.node.get_parameter('~{ax}_low_speed').get_parameter_value().string_value
            self.mp['speed'] = self.node.get_parameter('~{ax}_speed').get_parameter_value().string_value
            self.mp['acc'] = self.node.get_parameter('~{ax}_acc').get_parameter_value().string_value
            self.mp['dec'] = self.node.get_parameter('~{ax}_dec').get_parameter_value().string_value
            self.mp['step'] = self.node.get_parameter('~{ax}_step').get_parameter_value().string_value
            self.params[ax]['motion'] = mp
            continue

    #元driver
        self.func_queue = queue.Queue()
        self.pub = {}
        self.use_axis = ''.join([ax for ax in self.params])

        self.mode = [self.params[ax]['mode'] for ax in self.use_axis]
        self.motion = {ax: self.params[ax]['motion'] for ax in self.use_axis}


    #元handler
        #use axis
        self.current_speed = {ax: 0 for ax in self.use_axis}
        self.current_step = {ax: 0 for ax in self.use_axis}
        self.current_moving = {ax: 0 for ax in self.use_axis}
        self.last_direction = {ax: 0 for ax in self.use_axis}
        self.do_status = [0, 0, 0, 0]

        self.move_mode = {ax: p['mode'] for ax, p in self.params.items()}
        self.default_speed = {ax: p['motion']['speed'] for ax, p in self.params.items()}
        self.low_speed = {ax: p['motion']['low_speed'] for ax, p in self.params.items()}

        self.mot = pyinterface.open(7415, self.rsw_id)
        [self.mot.set_pulse_out(ax, 'method', self.params[ax]['pulse_conf']) for ax in self.use_axis]
        self.mot.set_motion(self.use_axis, self.mode, self.motion)


    #元handler 命令値を受け取る


        base = '/pyinterface/pci7415/rsw{self.rsw_id}'.format(**locals())

        for ax in self.use_axis:
            b = '{base}/{ax}/'.format(**locals())
            # rospy.Subscriber(b+'step_cmd', std_msgs.msg.Int64, self.set_step, callback_args=ax)
            self.node.create_subscription(std_msgs.msg.Int64, b+'step_cmd', partial(self.set_step, ax))

            # rospy.Subscriber(b+'speed_cmd', std_msgs.msg.Float64, self.set_speed, callback_args=ax)
            self.node.create_subscription(std_msgs.msg.Int64, b+'speed_cmd', partial(self.set_speed, ax))

        for do_num in range(1,5):
            self.node.create_subscription(std_msgs.msg.Int64, '{}/output_do{}_cmd'.format(base, do_num), partial(self.set_do, do_num))
            #rospy.Subscriber('{}/output_do{}_cmd'.format(base, do_num), std_msgs.msg.Int64, self.set_do, callback_args=do_num)


        # loop start 元driver
        # self.th = threading.Thread(target=self.loop)
        # self.th.setDaemon(True)
        # self.th.start()
        self.node.create_timer(1e-5,self.loop)

        return




    #　元driver
    def loop(self):
        # while not rospy.is_shutdown():
            #t0 = time.time()
        speed = self.mot.read_speed(self.use_axis)
        step = self.mot.read_counter(self.use_axis, cnt_mode='counter')
        moving = self.mot.driver.get_main_status(self.use_axis)
            #t1 = time.time()
         # speed, step はそのまま辞書に入れる　　for文はいらない

        self.current_speed = {ax: _ for ax, _ in zip(self.use_axis, speed)}
        self.current_step = {ax: _ for ax, _ in zip(self.use_axis, step)}
        self.current_moving = {ax: _ for ax, _ in zip(self.use_axis, moving)}

            # 所要時間を測る
            #t2 = time.time()
            #self.pub_dt1.publish(t1-t0)
            #self.pub_dt2.publish(t2-t1)
            #self.pub_qsize.publish(self.func_queue.qsize())
        if not self.func_queue.empty():
            f = self.func_queue.get()
            f['func'](f['data'], f['axis'])
        else:
            pass

        #time.sleep(1e-5)
            # 要検討

    def set_speed(self, speed, ax):
        if abs(speed) < self.low_speed[ax]:
            #pub stop
            self.func_queue.put({'func': self.stop, 'data': 1, 'axis': ax}) # ->req data は１のこと？？？？？???????
            while self.current_moving[ax] != 0:
                time.sleep(10e-5)
            self.last_direction[ax] = 0
            return

        if self.move_mode[ax] == 'jog':
            if (self.last_direction[ax] * speed > 0) & (self.current_moving[ax] != 0):
                #pub change_speed
                self.func_queue.put({'func': self.change_speed, 'data': abs(speed), 'axis': ax}) #req dataはabs(speed.data)???????
                pass

            else:

                if speed > 0:
                    step = +1
                    pass

                else:
                    step = -1
                    pass

                speed_step = [abs(speed), step]
                self.last_direction[ax] = step

                #pub start

                self.func_queue.put({'func': self.start, 'data': speed_step, 'axis': ax})　# speed_step_arrayはどこに代入？？？？？？？？
                time.sleep(0.01)
                pass
            pass

        else:
            pass


        return

    def set_step(self, step, ax):
        if self.move_mode[ax] == 'ptp':
            if self.current_moving[ax] != 0:
                #pub change_step
                self.func_queue.put({'func': self.change_speed, 'data': step.data, 'axis': ax})

            else:
                speed_step = [abs(self.default_speed[ax]), step.data]
                speed_step_array = std_msgs.msg.Float64MultiArray()
                speed_step_array.data = speed_step
                self.func_queue.put({'func': self.start, 'data': speed_step_array, 'axis': ax})
                pass
        else: pass
        return

　　#元handler
    def set_do(self, do, do_num):
        self.do_status[do_num-1] = do.data
        self.func_queue.put({'func': self.output_do, 'data': self.do_status, 'axis': 0})
        return

    #function
    def output_do(self, data, axis):
        self.mot.output_do(data)
        pass

    def start(self, data, axis):
        self.mot.stop_motion(axis=axis, stop_mode='immediate_stop')
        self.motion[axis]['speed'] = data[0]
        self.motion[axis]['step'] = int(data[1])
        axis_mode = [self.mode[self.use_axis.find(axis)]]
        while self.current_moving[axis] != 0:
            time.sleep(10e-5)
            continue
        self.mot.set_motion(axis=axis, mode=axis_mode, motion=self.motion)
        self.mot.start_motion(axis=axis, start_mode='const', move_mode=self.params[axis]['mode'])
        pass

    def stop(self, data, axis):
        self.mot.stop_motion(axis=axis, stop_mode='immediate_stop')
        pass

    def change_speed(self, data, axis):
        self.mot.change_speed(axis=axis, mode='accdec_change', speed=[data])
        #self.params[axis]['motion'][axis]['speed'] = abs(req.data)
        pass

    def change_step(self, data, axis):
        self.mot.change_step(axis=axis, step=[data])
        pass





# default_name = 'pci7415'
# default_rsw_id = '0'
# default_do_conf = "[0, 0, 0, 0]"
# default_use_axis = 'xyzu'
# default_pulse_conf = "{'PULSE': '0', 'OUT': '0', 'DIR': '0', 'WAIT': '0', 'DUTY': '0'}"
# default_mode = 'ptp'
# default_clock = 299
# default_acc_mode = 'acc_normal'
# default_low_speed = 200
# default_speed = 10000
# default_acc = 1000
# default_dec = 1000
# default_step = 0

def main(args=None):
    rclpy.init(args = args)
    pci7415=pci7415_driver()
    rclpy.spin(pci7415.node)

    pci7415.node.destory_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    #name = rospy.get_param('~node_name'.format(**locals()), default_name)
    #rospy.init_node(name)
    #rsw_id = rospy.get_param('~rsw_id', default_rsw_id)
    #use_axis = rospy.get_param('~use_axis', default_use_axis) # ex. 'xyzu', 'xy', or 'yu'
