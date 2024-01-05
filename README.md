# LIDAR07-100W

Driver and sample code for Single Point Lidar

# Quick Start:

1. Install Python
2. Install pyserial

# Run sample code

In ubuntu, make sure the user has read/write permission to the serial port. Otherwise, grant permission:

sudo chmod a+rw <serial port>

For example:

sudo chmod a+rw /dev/ttyUSB0

To run the sample code:

python continuous_measure.py
