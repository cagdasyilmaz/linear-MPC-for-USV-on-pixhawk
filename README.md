# linear-MPC-for-USV-on-pixhawk

During the thesis study, this program run on the mRo Pixhawk Flight Controller (Pixhawk 1). So, you should make your changes for your (Flight) Controller hardware.

usv_mpc_control file is located in the Pixhawk Firmware's src/examples directory.

In the directory of cmake/configs nuttx_px4fmu-v2_default.cmake file is opened and the pogram name that we defined above is added as a line "examples/usv_mpc_control" 
