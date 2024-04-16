.. detect_collision

Detecting Collision
===================

In this tutorial, we will see how to use the planner to detect collisions without planning a path. There are two APIs that are wrappers around some `fcl` library functions to provide a more convenient interface. In particular, we have `check_for_self_collision` and `check_for_env_collision`. As the name suggests, the former checks for robot self-collision, while the latter checks for collision between the robot and its environment.

Setting Up the Planner
----------------------

We will use the convenient function `setup_planner` provided by `mplib.examples.demo_setup.DemoSetup` to load the robot and create a planner. We will also make a function to print out the collisions detected.

.. literalinclude:: ../../../mplib/examples/detect_collision.py
    :language: python
    :start-after: # print_collision ankor
    :end-before: # print_collision ankor end

We will also create a floor as one of the collision objects to demonstrate the `check_for_env_collision` API.

.. literalinclude:: ../../../mplib/examples/detect_collision.py
    :language: python
    :start-after: # floor ankor
    :end-before: # floor ankor end

Note that we call floor an object because it is not an articulated object. The function to add an object to the planning world is `add_object`. This can also be used to update the pose of the object or change it our entirely.

Collision Time!
---------------

We will now test several configurations to see how the planner detects collisions. First, we will set the robot to a self-collision-free qpos and check for self-collision. This should return no collision. Note that the full joint configuration is not provided here. Instead, on the movegroup related joints are set. The rest of the joints are set to the current joint angle.

.. literalinclude:: ../../../mplib/examples/detect_collision.py
    :language: python
    :start-after: print("\n----- self-collision-free qpos -----")
    :end-before: print("\n----- self-collision qpos -----")

Next, we will put the robot into a self-collision qpos and check for self-collision. This should return a collision.

.. literalinclude:: ../../../mplib/examples/detect_collision.py
    :language: python
    :start-after: print("\n----- self-collision qpos -----")
    :end-before: print("\n----- env-collision-free qpos -----")

Then, we do the same thing with environment collision as we put the robot into a pose that collides with the floor. Additionally, we also try to plan a path to this qpos. This will cause the planner to timeout.

.. literalinclude:: ../../../mplib/examples/detect_collision.py
    :language: python
    :start-after: print("\n----- env-collision qpos -----")
    :end-before: print("\n----- no more env-collision after removing the floor -----")

Finally, we remove the floor and check for environment collision again. This should return no collision.

.. literalinclude:: ../../../mplib/examples/detect_collision.py
    :language: python
    :start-after: print("\n----- no more env-collision after removing the floor -----")
    :end-before: # end ankor
