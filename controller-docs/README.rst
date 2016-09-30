=========================
CompGeom Autonomous Robot
=========================
CompGeom Autonomous Robot specialized in GPS denied environment.

To use (with caution) and see a basic example, simply do::

    >>> import cgbot
    >>> print(cgbot.demo())


Installation
------------
To install the package `cgbot`, download and extract the zip and enter into the root directory where you will find `setup.py`::

    $ python setup.py install

Quickstart
----------
After successful installation, run `cg-log` to start logging. Use `-o` flag for console only output. See below for more 

Start compass service for direction. Either connect 9dof sensor and start instance of compass hardware::
    
    >>> from cgbot.compass import CompassHW
    >>> compass = CompassHW()
    >>> compass.start()

or connect to Android phone via bluetooth and start streaming using compass software::
    
    >>> from cgbot.compass import CompassSW
    >>> compass = CompassSW()
    >>> compass.start()

Both of these will keep Redis updated with appropriate sensor data. Once done, run the python 2 compatible instance of the robot in `robot_py2.main` module::
    
    >>> python2 -m robot_py2.main

Now that all systems are ready, import and use `commands` API for navigation (see below).

If you want to control it with XBOX controller, simply run the command `cg-ctrl` and press and hold the guide button on the controller. Once the program connects to the controller successfully, you can maneuver the robot with that.

Enjoy!


Command Line Entry Points
-------------------------
Installing the package will also install all the entry points as well as set the following executables in your `PATH`. Most of these accepts flags to operate in different ways::

    $ cg-log - start log process
        -j: Json output
        -c: Print log on console 
        -o: Print log only on console, not in logfile

    $ cg-init - Undefined

    $ cg-ctrl - start default controller, Xbox
    
    $ cg-run - Undefined. It will be assigned to start the robot
    
    $ cg-kill - Kills all processes responsible for running the robot

    $ cg-stop - Stop movement
        -a: Acceleration
        -w: Delay in seconds before execution
    
    $ cg-forward - Move in forward direction
        -a: Acceleration
        -s: Speed
        -d: Movement duration in seconds, 0 for unlimited time
    
    $ cg-reverse - Move in reverse direction
        -a: Acceleration
        -s: Speed
        -d: Movement duration in seconds, 0 for unlimited time
    
    $ cg-move - Moves the robot with specified parameters
        -al: Acceleration of left wheel
        -sl: Speed of left wheel
        -ar: Acceleration of right wheel
        -sr: Speed of right wheel
        -d: Movement duration in seconds, 0 for unlimited time

    $ cg-turn - Turns the robot with specified intensity. Intensity value ranges from -1 to 1.
        -i: Intensity value, -1.0 turns to the left at full intensity, +1.0 turns to the right
    
    $ cg-turn-left - Undefined. Turns the robot to the left at a specified angle
        -a: Angle in degrees/ radians
    
    $ cg-turn-right - Undefined. Turns the robot to the right at a specified angle
        -a: Angle in degrees/ radians
    
    $ cg-switch - Switches orientation of head
        -n: no of times
    
    $ cg-faster - Increases the speed of movement until it hits the limit
        -s: Speed increment amount
    
    $ cg-slower - Decreases the speed of movement until it stops
        -s: Speed decrement amount
    
    $ cg-follow - Work in progress. Follows a specified path
        -w: Waits w seconds before it starts following
    
    $ cg-goto - Go to a specified GPS coordinate
        -x: Latitude
        -y: Longitude
        -w: Wait time before starting
    
    $ cg-bt - Invokes Bluetooth receiver process in the robot to communicate with the cell phone. It requires flags to operate
        --start: Starts the receiver
        --stop: Stops the receiver
        --restart: Restarts the receiver
        --status: Shows the status of the receiver

    $ cg-cam-move - Pans/Tilts mounted 3d camera
        -p: target pan angle
        -t: target tilt angle

    $ cg-cam-capture - Captures video clips using 3d camera
        -d: duration of clip in seconds, default 5s
        -g: enable grayscale mode (alpha mode now)


Python API for Navigation
-------------------------
If you want to use the python api to send command to the robot, simply do::

    >>> import cgbot
    >>> from cgbot.commands import cmd

`cmd` is an instance of the `Commands` class, using which you can send instructions to the robot. The commands also support parameters in the same order as mentioned above for the command line entries::

    stop()
    forward()
    reverse()
    turn()
    turn_left()
    turn_right()
    switch_head()
    speed_up()
    speed_down()
    follow()
    go_to_point()

You can make the following calls to interact with the robot::

    >>> stop(accel, delay)
    # Instructs to stop after `delay` time

    >>> move(acc_l, spd_l, acc_r, spd_r, duration)
    # Instructs to move with individual wheel control for `duration` time. A value of zero means perpetual motion

    >>> forward(accel, speed, duration)
    # Instructs to move forward for `duration` time. A value of zero means perpetual motion

    >>> reverse(accel, speed, duration)
    # Instructs to move forward for `duration` time. A value of zero means perpetual motion

    >>> turn(amount)
    # Instructs to turn left/ right with `amount` intensity (synonymous to torque)
    #    0 means no turn
    #    +1.0 means turn at full capacity to the right
    #    -1.0 means turn at full capacity to the left

    >>> turn_left(angle)
    # Instructs to turn a predefined `angle` to the left

    >>> turn_right(angle)
    # Instructs to turn a predefined `angle` to the right

    >>> switch_head(times)
    # Instructs to switch the head `times` times

    >>> speed_up(amount)
    # Instructs to speed up a certain `amount`

    >>> speed_down(amount)
    # Instructs to speed down a certain `amount`

    >>> follow(delay)
    # Instructs to follow traced path after `delay`

    >>> goto_point(lat, lon, delay)
    # Instructs to move to gps coordinate (`lat`, `lon`) after `delay` seconds delay


Sensors
-------
While all the sensors are connected and fully operational, they update Redis with their readings associated with their corresponding keys. Values are 3 element tuples concatenated by colon (`:`). The three elements in the tuple represent different entities for different sensors.:

.. csv-table:: Sensor IDs
    :header: "ID", "Sensor", "Sample Values", "Description"
    :widths: 20, 30, 20, 140

    "CG_ACCELEROMETER", "Accelerometer", "(x, y, z)", "Acceleration force in m/s2 on all three physical axes (x, y, and z), including the force of gravity"
    "CG_MAGNETOMETER", "Magnetometer", "(x, y, z)", "Ambient geomagnetic field for all three physical axes (x, y, z) in μT."    
    "CG_ORIENRTATION", "Compass", "(x, y, z)", "Degrees of rotation around all three physical axes (x, y, z)"
    "CG_GYROSCOPE", "Gyroscope", "(x, y, z)", "Rate of rotation in rad/s around each of the three physical axes (x, y, and z)"
    "CG_LIGHTMETER", "Lightmeter", "(x, y, z)", "Measures the ambient light level x (illumination) in lux; y and z are always 0"    
    "CG_BAROMETER", "Barometer", "(x, y, z)", "Ambient air pressure x in hPa or mbar, y and z are always 0"    
    "CG_GEOLOCATION", "GPS", "(x, y, z)", "Latitude x, longitude y and confidence in these gps reading z (range of confidence 0-15)"    
    "CG_PROXIMITYSENSOR", "Proximity Sensor", "(x, y, z)", "Proximity of an object x relative to the view screen of a device. x is either 0 or 1; y and z are always 0"    
    "CG_GRVTATION", "Gravitational Acceleration", "(x, y, z)", "Acceleration due to gravity in m/s2 on all three physical axes (x, y, z)"    
    "CG_LINEAR_ACCELERATION", "Linear Acceleration", "(x, y, z)", "Acceleration force in m/s2 on all three physical axes (x, y, and z), excluding the force of gravity."    
    "CG_ROTATIONAL_VECTOR", "Rotational Velocity", "(x, y, z)", "Orientation by providing the three elements of the device’s rotation vector."    
    "CG_MICROPHONE", "Sound Amplitude", "(x, y, z)", "Amplitude in decibel given by x; y and z are always 0"


In python import redis instance and access the sensor keys to get their last updated values:

.. code:: python

    >>> from cgbot.redisdb import rdb
    >>> gps_data = rdb.get("CG_GEOLOCATION")
    >>> print(gps_data)
    30.396362:-84.210266:8.0

Here, using GPS key we obtained last recorded data from GPS, which in the example is `30.396362:-84.210266:8.0`. Splitting this string on colon (`:`) gives us:: 

    latitude = 30.40
    longitude = -84.21
    confidence = 8

When sensors are logged into a database, we use `SqliteDict`. The key value pair format in the database presents a challenge. Since we are aggregating multiple sensors, we can use the sensor identifier as key. But that alone is insufficient as it would get overwritten over time. So, the key is formed by appending the sensor identifier with time-stamp (accurate up to 0.1 microseconds). Value is the 3-element tuple (see table above) obtained from the sensors.::
    
    Key: <identifier>:<time-stamp>
    value: <Value 1>:<Value 2>:<Value 3>


Sensors API
-----------
+---------------+------------+--------------+
| Sensor        | Attribute  | Description  |
+===============+============+==============+
| Accelerometer |     ts     |  Timestamp   |
+               +------------+--------------+
| (Gravity      |     x      |   m/s2       |
+               +------------+--------------+
| Included)     |     y      |   m/s2       |
+               +------------+--------------+
|               |     z      |   m/s2       |
+---------------+------------+--------------+
| Barometer     |     ts     |  Timestamp   |
+               +------------+--------------+
|               |     hg     |   mbar       |
+---------------+------------+--------------+
| Encoder       |     ts     |  Timestamp   |
+               +------------+--------------+
|               |    left    |   Unitless   |
+               +------------+--------------+
|               |    right   |   Unitless   |
+---------------+------------+--------------+
| Lightmeter    |     ts     |  Timestamp   |
+               +------------+--------------+
|               |    lux     |     lux      |
+---------------+------------+--------------+
|               |     ts     |  Timestamp   |
+               +------------+--------------+
| Magnetometer  |     x      |     uT       |
+               +------------+--------------+
|               |     y      |     uT       |
+               +------------+--------------+
|               |     z      |     uT       |
+---------------+------------+--------------+
|               |     ts     |  Timestamp   |
+               +------------+--------------+
| Orientation   |     yaw    |    Degree    |
+               +------------+--------------+
|               |    pitch   |    Degree    |
+               +------------+--------------+
|               |    roll    |    Degree    |
+---------------+------------+--------------+

Usage of the Sensors API:

.. code:: python

    >>> from cgbot.sensors import Orientation
    >>> compass = Orientation()
    >>> compass.yaw, compass.pitch, compass.roll
    (0.12345, -0.32451, 0.65434)



Camera Movement
---------------
There is a pythonic way to access the camera pan tilt system. Simply import the camera module and send the pan and tilt angles (integer) to the camera::

    >>> from cgbot.camera import cam
    >>> cam.connect()  # Get a handle on the pan-tilt system
    >>> pan = 90; tilt = 120  # Sample pan, tilt values
    >>> cam.send(*cam.normalize_angles(pan, tilt))  # send
    >>> cam.disconnect()  # Once done, release the handle

In our current settings, panning is limited to [0 - 180] degree range and tilt is limited to [30 - 150] degree range. If any value is entered which is beyond those limits, the edge values will be assumed.

The sketch file for Arduino that drives the pan-tilt system can be found in the `arduino` directory:

.. code:: cpp

    #include <Servo.h>

    Servo base, top;

    const int MSG_START = 124;
    const int HORZ_ANGLE = 125;
    const int VERT_ANGLE = 126;
    const int MSG_END = 127;

    const int BASE_PIN = 4;
    const int TOP_PIN = 7;
    const int BAUDRATE = 9600;

    const int b_min = 0;
    const int b_max = 180;
    const int t_min = 30;
    const int t_max = 150;

    const int wait = 25;

    int pos_b = 90;
    int pos_t = 115;


    int rotate_base(int angle){
        /*
        ** Check if target angle exceeds limit. Increase or decrease angle
        ** gradually and wait briefly until target is reached
        */
        if (angle < b_min){
            angle = b_min;
        }
        if (angle > b_max){
            angle = b_max;
        }
        while(angle != pos_b){
            if(pos_b > angle){
                pos_b -= 1;
            } else{
                pos_b += 1;
            }
            base.write(pos_b);
            delay(wait);
        }

        return pos_b;
    }


    int rotate_top(int angle){
        /*
        ** Check if target angle exceeds limit. Increase or decrease angle
        ** gradually and wait briefly until target is reached
        */
        if (angle < t_min){
            angle = t_min;
        }
        if (angle > t_max){
            angle = t_max;
        }

        while(angle != pos_t){
            if(pos_t > angle){
                pos_t -= 1;
            } else{
                pos_t += 1;
            }
            top.write(pos_t);
            delay(wait);
        }
        return pos_t;
    }


    int decode_angle(int angle, int min_op, int max_op){
        /*
        ** Converts encoded angles ranging from 0-120 to intended angle
        ** Intended angle range for base (pan) is 0-180, top (tilt) is 30-150
        */
        const int min_ip = 0;
        const int max_ip = 120;
        // angle = min_op + (angle - min_ip) * (max_op - min_op)/(max_ip - min_ip);
        angle = min_op + (angle * (max_op - min_op)/120);
        return angle;
    }


    void setup() {
        base.attach(BASE_PIN);
        top.attach(TOP_PIN);

        Serial.begin(BAUDRATE);

        top.write(pos_t);
        base.write(pos_b);
    }


    void loop() {

        if(Serial.available() > 0){
            
            int flag = int(Serial.read());
            
            int angle_h = pos_b;
            int angle_v = pos_t;

            if (flag == HORZ_ANGLE){
                while(Serial.available() == 0){
                    continue;
                }
                angle_h = int(Serial.read());
                rotate_base(decode_angle(angle_h, b_min, b_max));
            } else if (flag == VERT_ANGLE){
                while(Serial.available() == 0){
                    continue;
                }
                angle_v = int(Serial.read());
                rotate_top(decode_angle(angle_v, t_min, t_max));
            }
        }
    }


Run Test
--------
We use nosetests with coverage here with the help of a `setup.cfg` file::

    [nosetests]
    verbosity=1
    detailed-errors=1
    with-coverage=1
    cover-package=cgbot
    debug=nose.loader
    pdb=1
    pdb-failures=1

`nose` automatically finds and runs the testcases. Simply run::
    
    python setup.py test

or::

    python setup.py nosetests

Add `-s` flag to not suppress print statement output::

    python setup.py nosetests -s


System Dependency
-----------------
:Essential:
    build-essential
    linux-headers-generic
    dkms
    libtool
    pkg-config
    libbz2-dev
    libz-dev
    liblzma-dev
    libreadline-dev
    autoconf
    libncursesw5-dev
    libssl-dev


:32 Bit Libraries:
    lib32z1
    lib32ncurses5
    lib32stdc++6


:C C++ Libraries:
    libc6
    libc6-dev
    libgcc1
    libstdc++6


:Database Dependencies:
    libgdbm-dev
    libsqlite3-dev
    libdb-dev


:Compiler:
    cython
    cmake


:Linear Algebra / Fortran / Fourier:
    liblapack3gf
    libatlas-base-dev
    libblas3gf
    libgfortran3
    gfortran
    libfftw3-dev
    libfftw3-doc


:X System:
    tk-dev


:QT:
    qt4-dev-tools
    qt4-designer
    libqtgui4
    libqtcore4
    libqt4-xml
    libqt4-test
    libqt4-script
    libqt4-network
    libqt4-dbus


:Graphics Libraries:
    libgle3
    libpng-dev
    libjpeg8-dev
    libfreetype6-dev


:Version Control:
    git
    mercurial


:XBOX Controller:
    jstest
    joystick
    xboxdrv


:Bluetooth:
    bluez
    bluez-cups
    bluez-dbg
    bluez-btsco
    bluez-tools
    bluewho
    indicator-bluetooth
    libbluetooth-dev
    libbluetooth3


Warning
-------
As of now, we are in the process of migrating from Python 2 to 3. All the code developed by us are fully compatible. But since many of the libraries used here may not conform fully to python 3 standards, there might be small hiccups here and there, most notably the motor controller driver.
