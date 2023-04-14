-------------------------------------------
            !!!
This file was used to document some useful and repetitive steps during the process 
of learning how to create, source, and build the custom-created package.
 It has no other purpose for the project. 
 It may come in handy when you want to use this code to build it on your own machine. 
 Notice that all commands presented here were used for Windows 10 platform.
-------------------------------------------

cd <PATH_TO_THE_ROOT> (the root is ros2_ws folder)

call C:\dev\ros2_foxy\local_setup.bat
set ROS_LOCALHOST_ONLY=0
set ROS_DOMAIN_ID=0
call install/setup.bat
ros2 run demo_nodes_cpp talker


--------------------------------------------------
colcon build --merge-install --packages-select itlsimulator

--------------------------------------------------


- > in other terminal navigate to ros2_ws and source the setup file:
call install/setup.bat

- > now run the talker
ros2 run itlsimulator talker

---------------------------------------------------
cd PATH_TO_THE_ROOT
call install/setup.bat
set ROS_LOCALHOST_ONLY=0
set ROS_DOMAIN_ID=0


---------------------------------------------------
in the  root  <PATH_TO_THE_ROOT>   

- > in other terminal navigate to ros2_ws and source the setup file:
call install/setup.bat
ros2 run itlsimulator listener
---------------------------------------------------
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
----------------------------------------------------
entry_points={
        'console_scripts': [
            'talker = itlsimulator.pubGuiNode:main',
            'listener = itlsimulator.subGuiNode:main',
            'stalker = itlsimulator.simplepublisher:main',
            'slistener = itlsimulator.simplesubscriber:main',
            'itlsubgui = itlsimulator.itl_subscriber:main',
            'itlpubgui = itlsimulator.itl_publisher:main',
            'getterUi = itlsimulator.itlGetterUi:main',
            'setterUi = itlsimulator.itlSetterUi:main',
        ],
    },
----------------------------------------------------
cd PATH_TO_THE_ROOT
call C:\dev\ros2_foxy\local_setup.bat
set ROS_LOCALHOST_ONLY=0
set ROS_DOMAIN_ID=0
call install/setup.bat
ros2 run demo_nodes_cpp talker
----------------------------------------------------
----------------------------------------------------