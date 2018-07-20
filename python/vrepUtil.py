#coding:utf-8
import vrep

import functools
import subprocess as sp
import warnings
from inspect import getargspec
import os
import numpy as np
import psutil
import os
import socket
from contextlib import closing
import atexit


list_of_instances = []



def getVirtualCamInternalMatrix(angle,resolution,flatten=True):
    '''获取相机的内参和投影矩阵
    @angle为视场角的弧度
    @flatten:为true时转换为1维list
    '''
    res =resolution
    cx = cy = res[0] * 0.5  
    #方法1:间接求
    # f = 0.008  # 相机焦距:单位mm(虚拟焦距)
    # dx = dy = tan(angle / 2) * f / (cx * 0.5)  # 单个像素大小
    # fx = f / dx #焦距的像素长度
    # fy = f / dy
    #方法2:直接求
    fx=fy=cx/np.tan(angle / 2) 
    Tx = Ty = 1
    K = np.array([[fx, 0, cx],
                [0, fy, cy],
                [0, 0,   1]],dtype=np.float64)  # 相机内参
    P = np.array([[fx, 0, cx, 0],
                [0, fy, cy, 0],
                [0, 0,   1, 0]],dtype=np.float64) # 投影矩阵
    if not flatten:
        return K
    return map(lambda x:x.flatten().tolist(),(K,P))


def getVirtualCamAdditionalMatrix(clientID, sensorHandle):
    "获得深度相机的近景/远景距离/视场角/分辨率"
    err, nearClip = vrep.simxGetObjectFloatParameter(
        clientID, sensorHandle, vrep.sim_visionfloatparam_near_clipping, vrep.simx_opmode_oneshot_wait)
    err, farClip = vrep.simxGetObjectFloatParameter(
        clientID, sensorHandle, vrep.sim_visionfloatparam_far_clipping, vrep.simx_opmode_oneshot_wait)
    err, angle = vrep.simxGetObjectFloatParameter(
        clientID, sensorHandle, vrep.sim_visionfloatparam_perspective_angle, vrep.simx_opmode_oneshot_wait)
    err, res_x = vrep.simxGetObjectIntParameter(
        clientID, sensorHandle, vrep.sim_visionintparam_resolution_x, vrep.simx_opmode_oneshot_wait)
    err, res_y = vrep.simxGetObjectIntParameter(
        clientID, sensorHandle, vrep.sim_visionintparam_resolution_y, vrep.simx_opmode_oneshot_wait)
    return  nearClip,farClip,angle,(res_x,res_y)



def cleanup():  # kill all spawned subprocesses on exit
    for i in list_of_instances:
        i.end()


atexit.register(cleanup)


def deprecated(msg=''):
    def dep(func):
        '''This is a decorator which can be used to mark functions
        as deprecated. It will result in a warning being emitted
        when the function is used.'''

        @functools.wraps(func)
        def new_func(*args, **kwargs):
            warnings.warn_explicit(
                "Call to deprecated function {}. {}".format(
                    func.__name__, msg),
                category=DeprecationWarning,
                filename=func.func_code.co_filename,
                lineno=func.func_code.co_firstlineno + 1)
            return func(*args, **kwargs)

        return new_func

    return deprecated


# the class holding a subprocess instance.
class instance():
    def __init__(self, args, suppress_output=True):
        self.args = args
        self.suppress_output = suppress_output
        list_of_instances.append(self)

    def start(self):
        print('(instance) starting...')
        try:
            if self.suppress_output:
                stdout = open(os.devnull, 'w')
            else:
                stdout = sp.STDOUT
            self.inst = sp.Popen(self.args, stdout=stdout, stderr=sp.STDOUT)
        except EnvironmentError:
            print('(instance) Error: cannot find executable at', self.args[0])
            raise

        return self

    def isAlive(self):
        return True if self.inst.poll() is None else False

    def end(self):
        print('(instance) terminating...')
        if self.isAlive():
            pid = self.inst.pid
            parent = psutil.Process(pid)
            for _ in parent.children(recursive=True):
                _.kill()
            retcode = parent.kill()
        else:
            retcode = self.inst.returncode
        print('(instance) retcode:', retcode)
        return self


# class holding a v-rep simulation environment.
import types, random
import numpy as np

blocking = vrep.simx_opmode_blocking
oneshot = vrep.simx_opmode_oneshot


class vrepper():
    def __init__(self,
                 port_num=None,
                 dir_vrep='',
                 headless=False,
                 suppress_output=True):
        if port_num is None:
            port_num = self.find_free_port_to_use()

        self.port_num = port_num

        if dir_vrep == '':
            print('(vrepper) trying to find V-REP executable in your PATH')
            import distutils.spawn as dsp
            path_vrep = dsp.find_executable('vrep.sh')  # fix for linux
            if path_vrep == None:
                path_vrep = dsp.find_executable('vrep')
        else:
            path_vrep = dir_vrep + 'vrep'
        print('(vrepper) path to your V-REP executable is:', path_vrep)
        if path_vrep is None:
            raise Exception(
                "Sorry I couldn't find V-Rep binary. "
                "Please make sure it's in the PATH environmental variable")

        # start V-REP in a sub process
        # vrep.exe -gREMOTEAPISERVERSERVICE_PORT_DEBUG_PREENABLESYNC
        # where PORT -> 19997, DEBUG -> FALSE, PREENABLESYNC -> TRUE
        # by default the server will start at 19997,
        # use the -g argument if you want to start the server on a different port.
        args = [
            path_vrep,
            '-gREMOTEAPISERVERSERVICE_' + str(self.port_num) + '_FALSE_TRUE'
        ]

        if headless:
            args.append('-h')

        # instance created but not started.
        self.instance = instance(args, suppress_output)

        self.cid = -1
        # clientID of the instance when connected to server,
        # to differentiate between instances in the driver

        self.started = False

        # is the simulation currently running (as far as we know)
        self.sim_running = False

        # assign every API function call from vrep to self
        vrep_methods = [
            a for a in dir(vrep) if not a.startswith('__')
            and isinstance(getattr(vrep, a), types.FunctionType)
        ]

        def assign_from_vrep_to_self(name):
            wrapee = getattr(vrep, name)
            arg0 = getargspec(wrapee)[0][0]
            if arg0 == 'clientID':

                def func(*args, **kwargs):
                    return wrapee(self.cid, *args, **kwargs)
            else:

                def func(*args, **kwargs):
                    return wrapee(*args, **kwargs)

            setattr(self, name, func)

        for name in vrep_methods:
            assign_from_vrep_to_self(name)

    def find_free_port_to_use(
            self
    ):  # https://stackoverflow.com/questions/1365265/on-localhost-how-do-i-pick-a-free-port-number
        with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:
            s.bind(('', 0))
            return s.getsockname()[1]

    # start everything
    def start(self):
        if self.started == True:
            raise RuntimeError('you should not call start() more than once')

        print('(vrepper) starting an instance of V-REP...')
        self.instance.start()

        # try to connect to V-REP instance via socket
        retries = 0
        while True:
            print('(vrepper) trying to connect to server on port',
                  self.port_num, 'retry:', retries)
            # vrep.simxFinish(-1) # just in case, close all opened connections
            self.cid = self.simxStart(
                '127.0.0.1',
                self.port_num,
                waitUntilConnected=True,
                doNotReconnectOnceDisconnected=True,
                timeOutInMs=1000,
                commThreadCycleInMs=0)  # Connect to V-REP

            if self.cid != -1:
                print('(vrepper) Connected to remote API server!')
                break
            else:
                retries += 1
                if retries > 15:
                    self.end()
                    raise RuntimeError(
                        '(vrepper) Unable to connect to V-REP after 15 retries.'
                    )

        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        objs, = check_ret(self.simxGetObjects(vrep.sim_handle_all, blocking))

        print('(vrepper) Number of objects in the scene: ', len(objs))

        # Now send some data to V-REP in a non-blocking fashion:
        self.simxAddStatusbarMessage('(vrepper)Hello V-REP!', oneshot)

        # setup a useless signal
        self.simxSetIntegerSignal('asdf', 1, blocking)

        print(
            '(vrepper) V-REP instance started, remote API connection created. Everything seems to be ready.'
        )

        self.started = True
        return self

    # kill everything, clean up
    def end(self):
        print('(vrepper) shutting things down...')
        # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        # vrep.simxGetPingTime(clientID)

        # Now close the connection to V-REP:
        if self.sim_running:
            self.stop_simulation()
        self.simxFinish()
        self.instance.end()
        print('(vrepper) everything shut down.')
        return self

    def load_scene(self, fullpathname):
        print('(vrepper) loading scene from', fullpathname)
        try:
            check_ret(
                self.simxLoadScene(
                    fullpathname,
                    0,  # assume file is at server side
                    blocking))
        except:
            print('(vrepper) scene loading failure')
            raise
        print('(vrepper) scene successfully loaded')

    def start_blocking_simulation(self):
        self.start_simulation(True)

    def start_nonblocking_simulation(self):
        self.start_simulation(False)

    def start_simulation(self, is_sync):
        # IMPORTANT
        # you should poll the server state to make sure
        # the simulation completely stops before starting a new one
        while True:
            # poll the useless signal (to receive a message from server)
            check_ret(self.simxGetIntegerSignal('asdf', blocking))

            # check server state (within the received message)
            e = self.simxGetInMessageInfo(vrep.simx_headeroffset_server_state)

            # check bit0
            not_stopped = e[1] & 1

            if not not_stopped:
                break

        # enter sync mode
        check_ret(self.simxSynchronous(is_sync))
        check_ret(self.simxStartSimulation(blocking))
        self.sim_running = True

    def make_simulation_synchronous(self, sync):
        if not self.sim_running:
            print(
                '(vrepper) simulation doesn\'t seem to be running. starting up'
            )
            self.start_simulation(sync)
        else:
            check_ret(self.simxSynchronous(sync))

    def stop_simulation(self):
        check_ret(self.simxStopSimulation(oneshot), ignore_one=True)
        self.sim_running = False

    @deprecated('Please use method "stop_simulation" instead.')
    def stop_blocking_simulation(self):
        self.stop_simulation()

    def step_blocking_simulation(self):
        check_ret(self.simxSynchronousTrigger())

    def get_object_handle(self, name):
        handle, = check_ret(self.simxGetObjectHandle(name, blocking))
        return handle

    def get_object_by_handle(self, handle, is_joint=True):
        """
        Get the vrep object for a given handle

        :param int handle: handle code
        :param bool is_joint: True if the object is a joint that can be moved
        :returns: vrepobject
        """
        return vrepobject(self, handle, is_joint)

    def get_object_by_name(self, name, is_joint=True):
        """
        Get the vrep object for a given name

        :param str name: name of the object
        :param bool is_joint: True if the object is a joint that can be moved
        :returns: vrepobject
        """
        return self.get_object_by_handle(
            self.get_object_handle(name), is_joint)

    @staticmethod
    def create_params(ints=[], floats=[], strings=[], bytes=''):
        if bytes == '':
            bytes_in = bytearray()
        else:
            bytes_in = bytes
        return (ints, floats, strings, bytes_in)

    def call_script_function(self,
                             function_name,
                             params,
                             script_name="remoteApiCommandServer"):
        """
        Calls a function in a script that is mounted as child in the scene

        :param str script_name: the name of the script that contains the function
        :param str function_name: the name of the function to call
        :param tuple params: the parameters to call the function with (must be 4 parameters: list of integers, list of floats, list of string, and bytearray

        :returns: tuple (res_ints, res_floats, res_strs, res_bytes)
            WHERE
            list res_ints is a list of integer results
            list res_floats is a list of floating point results
            list res_strs is a list of string results
            bytearray res_bytes is a bytearray containing the resulting bytes
        """
        assert type(params) is tuple
        assert len(params) == 4

        return check_ret(
            self.simxCallScriptFunction(
                script_name,
                vrep.sim_scripttype_childscript,
                function_name,
                params[0],  # integers
                params[1],  # floats
                params[2],  # strings
                params[3],  # bytes
                blocking))

    def get_global_variable(self, name, is_first_time):
        if is_first_time:
            return vrep.simxGetFloatSignal(self.cid, name,
                                           vrep.simx_opmode_streaming)
        else:
            return vrep.simxGetFloatSignal(self.cid, name,
                                           vrep.simx_opmode_buffer)

    def _convert_byte_image_to_color(self, res, img):
        reds = np.zeros(res[0] * res[1], dtype=np.uint8)
        greens = np.zeros(res[0] * res[1], dtype=np.uint8)
        blues = np.zeros(res[0] * res[1], dtype=np.uint8)
        for i in range(0, len(img), 3):
            reds[int(i / 3)] = img[i] & 255
            greens[int(i / 3)] = img[i + 1] & 255
            blues[int(i / 3)] = img[i + 2] & 255

        img_out = np.zeros((res[0], res[1], 3), dtype=np.uint8)
        img_out[:, :, 0] = np.array(reds).reshape(res)
        img_out[:, :, 1] = np.array(greens).reshape(res)
        img_out[:, :, 2] = np.array(blues).reshape(res)

        return img_out

    def get_image(self, object_id):
        res, img = check_ret(
            self.simxGetVisionSensorImage(object_id, 0, blocking))
        return self._convert_byte_image_to_color(res, img)

    def _convert_depth_to_image(self, res, depth):
        reshaped_scaled = 255 - np.array(depth).reshape(
            res) * 255  # because is in range [0,1] and inverted
        rounded = np.around(reshaped_scaled, 0).astype(np.uint8)
        return rounded

    def _convert_depth_to_rgb(self, res, depth):
        rounded = self._convert_depth_to_image(res, depth)
        img = np.zeros((res[0], res[1], 3), dtype=np.uint8)
        img[:, :, 0] = rounded
        img[:, :, 1] = rounded
        img[:, :, 2] = rounded
        return img

    def get_depth_image(self, object_id):
        res, depth = check_ret(
            self.simxGetVisionSensorDepthBuffer(object_id, blocking))
        return self._convert_depth_to_image(res, depth)

    def get_depth_image_as_rgb(self, object_id):
        res, depth = check_ret(
            self.simxGetVisionSensorDepthBuffer(object_id, blocking))
        return self._convert_depth_to_rgb(res, depth)

    def get_image_and_depth(self, object_id):
        img = self.get_image(object_id)
        depth = self.get_depth_image(object_id)

        out = np.zeros((img.shape[0], img.shape[1], 4), dtype=np.uint8)
        out[:, :, :3] = img
        out[:, :, 3] = depth

        return out

    def get_collision_handle(self, name_of_collision_obj):
        """ In order to use this you first have to open the scene in V-Rep, then
            click on "calculation module properties" on the left side (the button
            that looks like "f(x)"), then click "add new collision object", chose the
            two things between which you want to check for collision (one of them can be a collection
            which you can create in yet another window), and finally double click on the new
            collision object in order to rename it to something more catchy than "Collision".
            You can find more information here:
            http://www.coppeliarobotics.com/helpFiles/en/collisionDetection.htm
            Also don't forget to save the scene after adding the collision object.

        :param name_of_collision_obj: the "#" is added automatically at the end
        :return: collision_handle (this is an integer that you need for check_collision)
        """
        return check_ret(
            self.simxGetCollisionHandle(name_of_collision_obj + "#",
                                        blocking))[0]

    def check_collision(self, collision_handle):
        """ At any point in time call this function to get a boolean value if the
            collision object is currently detecting a collision. True for collision.

        :param collision_handle: integer, the handle that you obtaind from
                                    "get_collision_handle(name_of_collision_obj)"
        :return: boolean
        """
        return check_ret(self.simxReadCollision(collision_handle, blocking))[0]

    def get_collision_object(self, name_of_collision_obj):
        """ this is effectively the same as "get_collision_handle" but instead of an
            integer (the handle) it instead returns an object that has a ".is_colliding()"
            function, which is super marginally more convenient.

        :param name_of_collision_obj: string, name of the collision object in V-Rep
        :return: Collision object that you can check with ".is_colliding()->bool"
        """

        handle = check_ret(
            self.simxGetCollisionHandle(name_of_collision_obj + "#",
                                        blocking))[0]
        col = Collision(env=self, handle=handle)
        return col


class Collision(object):
    def __init__(self, env, handle):
        self.handle = handle
        self.env = env

    def is_colliding(self):
        return check_ret(self.env.simxReadCollision(self.handle, blocking))[0]


# check return tuple, raise error if retcode is not OK,
# return remaining data otherwise
def check_ret(ret_tuple, ignore_one=False):
    istuple = isinstance(ret_tuple, tuple)
    if not istuple:
        ret = ret_tuple
    else:
        ret = ret_tuple[0]

    if (not ignore_one and ret != vrep.simx_return_ok) or (ignore_one and ret > 1):
        raise RuntimeError('retcode(' + str(ret) +
                           ') not OK, API call failed. Check the paramters!')

    return ret_tuple[1:] if istuple else None


class vrepobject():
    def __init__(self, env, handle, is_joint=True):
        self.env = env
        self.handle = handle
        self.is_joint = is_joint

    def get_orientation(self, relative_to=None):
        eulerAngles, = check_ret(
            self.env.simxGetObjectOrientation(
                self.handle, -1
                if relative_to is None else relative_to.handle, blocking))
        return eulerAngles

    def get_position(self, relative_to=None):
        position, = check_ret(
            self.env.simxGetObjectPosition(
                self.handle, -1
                if relative_to is None else relative_to.handle, blocking))
        return position

    def get_velocity(self):
        return check_ret(
            self.env.simxGetObjectVelocity(
                self.handle,
                # -1 if relative_to is None else relative_to.handle,
                blocking))
        # linearVel, angularVel

    def set_velocity(self, v):
        self._check_joint()
        return check_ret(
            self.env.simxSetJointTargetVelocity(self.handle, v, blocking))

    def set_force(self, f):
        self._check_joint()
        return check_ret(self.env.simxSetJointForce(self.handle, f, blocking))

    def set_position_target(self, angle):
        """
        Set desired position of a servo

        :param int angle: target servo angle in degrees
        :return: None if successful, otherwise raises exception
        """
        self._check_joint()
        return check_ret(
            self.env.simxSetJointTargetPosition(self.handle, -np.deg2rad(angle),
                                                blocking))

    def set_position(self, x, y, z):
        """
        Set object to specific position (should never be done with joints)
        :param pos:  tuple or list with 3 coordinates
        :return: None
        """
        pos = (x, y, z)
        return check_ret(
            self.env.simxSetObjectPosition(self.handle, -1, pos, blocking))

    def get_joint_angle(self):
        self._check_joint()
        angle = check_ret(self.env.simxGetJointPosition(self.handle, blocking))
        return -np.rad2deg(angle[0])

    def get_joint_force(self):
        self._check_joint()
        force = check_ret(self.env.simxGetJointForce(self.handle, blocking))
        return force

    def get_joint_velocity(self):
        self._check_joint()
        vel = check_ret(
            self.env.simxGetObjectFloatParameter(
                self.handle, vrep.sim_jointfloatparam_velocity, blocking))
        return vel

    def read_force_sensor(self):
        state, forceVector, torqueVector = check_ret(
            self.env.simxReadForceSensor(self.handle, blocking))

        if state & 1 == 1:
            return None  # sensor data not ready
        else:
            return forceVector, torqueVector

    def get_vision_image(self):
        resolution, image = check_ret(
            self.env.simxGetVisionSensorImage(
                self.handle,
                0,  # options=0 -> RGB
                blocking,
            ))
        dim, im = resolution, image
        nim = np.array(im, dtype='uint8')
        nim = np.reshape(nim, (dim[1], dim[0], 3))
        nim = np.flip(nim, 0)  # LR flip
        nim = np.flip(nim, 2)  # RGB -> BGR
        return nim

    def _check_joint(self):
        if not self.is_joint:
            raise Exception(
                "Trying to call a joint function on a non-joint object.")

    def get_global_variable(self, name, is_first_time):
        if is_first_time:
            return vrep.simxGetFloatSignal(self.cid, name,
                                           vrep.simx_opmode_streaming)
        else:
            return vrep.simxGetFloatSignal(self.cid, name,
                                           vrep.simx_opmode_buffer)
