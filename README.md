# ik_solver_6dof

Inverse kinematic solver for 6-DOF manipulator with configuration like ABB's IRB120 or Han's Elfin robot...

## Usage

```bash
catkin build --make-args tests -- ik_solver_6dof
source devel/setup.bash
roslaunch ik_solver_6dof ik_solver_test.launch
```
