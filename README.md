MultiVehicleTool
==========

Tool to control Anki vehicles using AnkiDrive SDK.

Hardware
========
You need a working Bluetooth 4.0 Dongle (BTLE capable).
hciconfig should give an output like this:

	hci0:   Type: BR/EDR  Bus: USB
        BD Address: 5C:F3:70:65:4C:EB  ACL MTU: 1021:8  SCO MTU: 64:1
        UP RUNNING
        RX bytes:159679 acl:5344 sco:0 events:1104 errors:0
        TX bytes:12871 acl:505 sco:0 commands:329 errors:0

If the device is marked as down you need to set it up using ```sudo hciconfig hci0 up```

Dependencies
============

### glib-2 and readline
Install via your favorite package manager

	sudo apt-get install libglib2.0-dev libreadline-dev libboost-dev libboost-filesystem-dev libboost-thread-dev

Building
========
This build relies on a current Bluez version which is included as a submodule. So you need to fetch this using --recursive.

	git clone --recursive gitlab@novgit05.novero.com:rstrac/multiVehicleTool.git
	cd multiVehicleTool
	mkdir build
	cd build
	cmake ..
	make

Capabilities
============
To set BLE connection parameters the user needs some special privileges which are called `capabilities` on linux.
If you run the program as root or via sudo you can just start the program.

Else you need to set the capabilities of the binary by issuing the following command:

	sudo setcap cap_net_raw,cap_net_admin+eip multiVehicleTool

The cmake configuration is trying to do this automatically. This why it will ask you to provide your password if you are not root.
You can surpress this step by passing `-D NOSETCAP=YES` to the cmake command. (useful for automated builds)

### Caveats
Your filesystem needs to support capabilities. Know working is `ext4`. Definitely not working are most "remote filesystems" like `smbfs` or `hgfs` (VMWare Shared Folder).
Additionally your filesystem needs to be mounted allowing `suid`. So if you see a `nosuid` flag when you issue a mount command this will not work.

You can check if multiVehicleTool is running with the correct capabilities by issuing the following command after starting it:

	getpcaps $(pidof multiVehicleTool)
	#Capabilities for `104234': = cap_net_admin,cap_net_raw+ep

Running
=======
If you have set capabilities just start the program like this:

	./multiVehicleTool

After starting the tool you have to scan for vehicles using the 

	scan

command. If your HCI device is working correctly you should see a list of available vehicles, it should look like this:

	[    Broadcast (0)]> scan
	1475229008.232: Scanning for bluetooth low-energy devices using hci0
	1475229008.262: Discovered FC:49:C7:17:BE:62 GROUNDSHOCK0 [v2774] (unknown d75c)

You can select the vehicle either by its name 

	select-vehicle GROUNDSHOCK0

or by its macaddress

	select FC:49:C7:BE:62

Once the vehicle is selected, you can then connect to it

	connect 

and upon successful connection start sending commands to the vehicle. Some commands to try

	set-speed 500
	set-speed 0
	change-lane 100 1000 6
	uturn

If you have a vehicle collection file in the same directory (see as an example inputs/vehiclePoolDefaults.json) you can skip the scan and connect directly to the vehicle via its name

	connect VEHICLE_NAME
	

The program has the ability to run a preprogrammed sequence of commands. As an example for how this works see the file inputs/get_started.drive. You can either pipe this file directly into multiVehicleTool by running

	./multiVehicleTool < inputs/get_started.drive

or by calling

	execute inputs/get_started.drive

directly inside multiVehicleTool.

For more options regarding supported commands call 'help' inside multiVehicleTool.
