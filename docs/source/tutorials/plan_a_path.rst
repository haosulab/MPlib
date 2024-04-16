.. _plan_a_path:

Plan a Path
==================

.. highlight:: python

In this tutorial, we will talk about how to plan paths for the agent. As shown in the demo, the robot needs to move the three boxes a bit forward. The full script can be found here :download:`demo.py <../../../mplib/examples/demo.py>`. You will also need :download:`demo_setup.py <../../../mplib/examples/demo_setup.py>` and grab the panda URDF.

.. figure:: assets/RRT.gif
   :width: 320px
   :align: center

   plan with RRTConnect

.. note::
   This tutorial only talks about the basic usages, and the robot only avoids self-collisions (i.e., collisions between the robot links) in this demo. Please refer to :ref:`collision_avoidance` to include the environment model and other advanced usages. 

Plan with sampling-based algorithms
--------------------------------------

``mplib`` supports state-of-the-art sampling-based motion planning algorithms by leveraging `OMPL <https://github.com/ompl/ompl>`_. You can call ``planner.plan_pose()`` to plan a path for moving the ``move_group`` link to a target pose: 

.. literalinclude:: ../../../mplib/examples/demo_setup.py
   :dedent: 0
   :start-after: # plan_pose ankor
   :end-before: # plan_pose ankor end

Specifically, ``planner.plan_pose()`` takes two required arguments as input. The first one is the target pose of the ``move_group`` link. It's a 7-dim list, where the first three elements describe the position part, and the remaining four elements describe the quaternion (wxyz) for the rotation part. **Note that the pose is relative to the world frame**. Normally, the base link of the robot is the world frame unless you have called ``set_base_pose(new_pose)`` in on the planner. You can also temporarily plan w.r.t. the robot base by passing in ``wrt_world=False``.

The second argument is the current joint positions of all the active joints (not just all the active joints in the movegroup). The ``planner.plan_pose()`` function first solves the inverse kinematics to get the joint positions for the target pose. It then calls the RRTConnect algorithm to find a path in the joint space. Finally, it simplifies the path and parameterizes the path to generate time, velocity, and acceleration information.

``planner.plan_pose()`` returns a dict which includes:   

- ``status``: a string indicates the status:

  - ``Success``: planned a path successfully.
  - ``IK Failed``: failed to solve the inverse kinematics. This may happen when the target pose is not reachable.
  - ``RRT Failed``: failed to find a valid path in the joint space. This may happen when there is no valid path or the task is too complicated.
- ``position``: a NumPy array of shape :math:`(n \times m)` describes the joint positions of the waypoints. :math:`n` is the number of waypoints in the path, and each row describes a waypoint. :math:`m` is the number of active joints that affect the pose of the ``move_group`` link. For example, for our panda robot arm, each row includes the positions for the first seven joints. 
- ``duration``: a scalar indicates the duration of the output path. ``mplib`` returns the optimal duration considering the velocity and acceleration constraints. 
- ``time``: a NumPy array of shape :math:`(n)` describes the time step of each waypoint. The first element is equal to 0, and the last one is equal to the ``duration``. Argument ``time_step`` determines the interval of the elements.
- ``velocity``: a NumPy array of shape :math:`(n \times m)` describes the joint velocities of the waypoints. 
- ``acceleration``: a NumPy array of shape :math:`(n \times m)` describing the joint accelerations of the waypoints. 


``planner.plan_pose()`` also takes other optional arguments with default values:

.. automethod:: mplib.Planner.plan_pose
   :no-index:

Follow a path
--------------------------------------
``plan_pose()`` outputs a time-parameterized path, and we need to drive the robot to follow the path. In this demo, we use ``sapien`` to simulate and drive the robot.

.. literalinclude:: ../../../mplib/examples/demo_setup.py
   :dedent: 0
   :start-after: # follow path ankor
   :end-before: # follow path ankor end

.. note::
    If you find your robot doesn't move as expected, please **double-check** your controller, especially the controller's parameters. In many cases, the planner finds a good path while the controller fails to follow the path.


Plan with screw motion
--------------------------------------
Besides using the sampling-based algorithms, we also provide another simple way (trick) to plan a path. For some tasks, we can directly move the ``move_group`` link towards the target pose. It's internally achieved by first calculating the relative transformation from its current pose to the target pose, then calculating the relative transformation's exponential coordinates, and finally calculating the joint velocities with the Jacobian matrix.

Compared to the sampling-based algorithms, planning with screw motion has the following pros:

- faster: since it doesn't need to sample lots of states in the joint space, planning with screw motion can save lots of planning time.
- `straighter` path: there is no guarantee for sampling-based algorithms to generate `straight` paths even it's a simple lifting task since it connects states in the joint space. In contrast, the returned path by the exponential coordinates and the Jacobian matrix can sometimes be more reasonable. See the above figures for comparison. 


You can call ``planner.plan_screw()`` to plan a path with screw motion. Similar to ``planner.plan_pose()``, it also takes two required arguments: target pose and current joint positions, and returns a dict containing the same set of elements. 

.. literalinclude:: ../../../mplib/planner.py
   :dedent: 0
   :start-after: # plan_screw ankor
   :end-before: # plan_screw ankor end

However, planning with screw motion only succeeds when there is no collision during the planning since it can not detour or replan. We thus recommend use ``planner.plan_screw()`` for some simple tasks or combined with ``planner.plan_pose()``. As shown in the code, we first try ``planner.plan_screw()``, if it fails (e.g., collision during the planning), we then turn to the sampling-based algorithms. Other arguments are the same with ``planner.plan_pose()``.


Move the boxes
--------------------------------------

In this example, we create some boxes inside the simulation like so:

.. literalinclude:: ../../../mplib/examples/demo.py
   :dedent: 0
   :start-after: # boxes ankor
   :end-before: # boxes ankor end

We then find the target poses needed to reach the boxes.

.. literalinclude:: ../../../mplib/examples/demo.py
   :dedent: 0
   :start-after: # target poses ankor
   :end-before: # target poses ankor end

Then, we plan and execute the motion:

.. literalinclude:: ../../../mplib/examples/demo.py
   :dedent: 0
   :start-after: # execute motion ankor
   :end-before: # execute motion ankor end

.. figure:: assets/screw.gif
   :width: 320px
   :align: center

   plan with screw motion
