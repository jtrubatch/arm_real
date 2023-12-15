## Needed Packages For Testing 
ur_mtc
real_moveit_config

## Launch Files
setup_real.launch.py
test_ng.launch.py

## General Notes
mtc_test_ng.cpp is the most current. It is an altered version of my main program that does not grasp the real cup, 
but does attach the cup object and goes through all the same motions. 

There is a primary failure mode where the initial grasp is planned with the gripper upside down. This did not occur in the sim, 
I need to enable a constraint, but I have just been restarting if this occurs. 

I was adjusting the retract from coffee machine before stopping. There may be a failure to travel the full distance. 

In general if there is a failure to plan, restarting the mtc_test_ng node will eventually get past it. 

## Octomap 
There are a few parameters passed in setup_real.launch.py. The bulk are in sensors_3d.yaml. 
I was able to get the clearing/updating to occur more reliably by increasing the point_subsample value to 2.  
The padding values are still set extremely high. If you drop them to around padding_offset: 0.05 and padding_scale: 1.1 
you should see more of the issues I have been describing. 

Even with the update rate increased it seems like there is something that is blocking/slowing the update. I have a delay in my program but in practice
it seems to take up to a minute or two to fully update. To allow this to happen, leave the setup node running and just wait, or restart the test node. 