# Get rdk

sshpass -p adminisadmin ssh sunrise@192.168.0.107

# Usage on the RDK100:

cd /home/sunrise/ratgun_hardware

## always start in root ##

# AprilTag mode:

sudo ./run_stack.py --april_tags

# Red-point mode:

sudo ./run_stack.py --red_point


# Core stack only:

sudo ./run_stack.py


# From the workspace directory, run:

sudo ./run_stack.py --stop

# If you are already logged in as root:

./run_stack.py --stop




