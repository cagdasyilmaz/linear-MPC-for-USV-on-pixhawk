# linear-MPC-for-USV-on-pixhawk

During the thesis study, this program run on the mRo Pixhawk Flight Controller (Pixhawk 1). So, you should make your changes for your (Flight) Controller hardware.

_usv_mpc_control_ file is located in the Pixhawk Firmware's _src/examples_ directory.

In the directory of _cmake/configs_ _nuttx_px4fmu-v2_default.cmake_ file is opened and the pogram name that we defined above is added as a line "examples/usv_mpc_control". A sample nuttx_px4fmu-v2_default.cmake file can be found above.

_usv_mpc_control_ program uses special uorb messages _vehicle_reference_states_ in addition the pre-defined uorb-messages. In order to employ this message, you should create the _vehicle_reference_states_ file in the _msg_ directory and you have to add a line in _CMakeList.txt_ file which in also in the same directory. A sample _vehicle_reference_states_ and _CMakeList.txt_ files can be found above in the msg file.



