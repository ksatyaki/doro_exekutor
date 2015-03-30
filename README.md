###DORO EXEKUTOR

The exekutor components for use on Domestic Robot platform used in Robot-Era:

|S.No.| Class name | Format of Parameter Tuple | Summary|
|----:|:-----------|:--------------------------|:-------|
|1| DockExekutor | $object_name | Robot docks to object to be able to manipulate it. The tuple $object_name.pos.geo should exist in the CAM.|  
|2| MoveHandExekutor | cartesian/joints, absolute/relative, value1, value2, ... ,value6 | Moves arm to the desired pose. Takes both cartesian-space and joints-space goals. For cartesian space, relative means "from the current pose" and the six values are (x, y, z, r, p, y) relative to the jaco's base frame.|
|3 | MoveitHandExekutor | p.x, p.y, p.z, q.x, q.y, q.z, q.w | Uses Moveit! to plan a motion to the pose specified by position __p__ and quaternion __q__. This pose should be in the 'base_link' frame of the robot.|
|4 | PickUpExekutor | $object_name | Does the following in order: Look($object_name), Acquire($object_name), uses the grasp pose generation to generate a grasp pose and uses MoveitHand to pick up the object.|