"""
This module wraps the pyrplidar library to provide functions to connect to and disconnect from a RPLidar device,
as well as process incoming scan data. It builds a wrapper around the PyRPlidar library, and implements enough functionality to connect to the device, start a scan, and process the incoming data. More advanced functionality can be added as needed. Refer to the PyRPlidar library to see what other functionality is available.

Refer to the alex_example_lidar.py file for an example of how to use these functions.


Functions:
    lidarConnect(port: str = PORT, baudrate: int = BAUDRATE, wait: int = 2) -> PyRPlidar:
        Connects to the RPLidar device, resets it, and sets the motor PWM.
    lidarDisconnect(lidar: PyRPlidar):
        Stops the RPLidar device, sets the motor PWM to 0, and disconnects it.
    process_scan(incomingScanTuple: tuple, currentRound: dict = {"r": 0, "buff": [], "doScan": False}) -> tuple:
        Processes incoming scan data from the RPLidar device, buffering scans and returning
        complete scan data for 1 full rotation.
Constants:
    PORT (str): The default port to connect to the RPLidar device.
    BAUDRATE (int): The default baud rate for the RPLidar device.

"""

# import matplotlib
# matplotlib.use('TKagg')

import time
import numpy as np

import sys
sys.path.append("/home/prof/code/pyrplidar")

from pyrplidar import PyRPlidar, PyRPlidarMeasurement



# RPLidar A1M8
# PORT = "COM4"           # Windows
PORT = "/dev/ttyUSB0"   # Linux
BAUDRATE = 115200       # The default baud rate for the RPLidar A1M8
_LIDAR_OBJECT = None


def lidarConnect(port=PORT, baudrate=BAUDRATE, wait=2):
    """
    Establishes a connection to the LiDAR device, resets it, and sets the motor PWM. We connect to the LiDAR device twice to ensure that the lidar is properly reset and the motor is set to the correct PWM. The lidar occasionally fails to start correctly if previously connected and left running without being reset.

    Args:
        port (str): The port to which the LiDAR device is connected. Default is PORT.
        baudrate (int): The baud rate for the connection. Default is BAUDRATE.
        wait (int): The time in seconds to wait after resetting the LiDAR device. Default is 2 seconds.

    Returns:
        PyRPlidar: An instance of the connected and configured LiDAR device.
    """
    global _LIDAR_OBJECT
    if _LIDAR_OBJECT is not None:
        return _LIDAR_OBJECT
    
    lidar = PyRPlidar()

    # We reset the lidar and reconnect to ensure proper function.
    # This is based on empirical testing and may not be necessary in all cases.
    # The lidar occasionally fails to start correctly if previously connected and left running without being reset.
    lidar.connect(port=port, baudrate=baudrate, timeout=10)
    lidar.reset()
    if wait:
        time.sleep(wait)
    lidar.disconnect()

    # Connect to the lidar for real this time
    # This is the actual connection we will use
    lidar.connect(port=port, baudrate=baudrate, timeout=10)
    lidar.set_motor_pwm(500)
    _LIDAR_OBJECT = lidar
    return _LIDAR_OBJECT

def lidarDisconnect(lidar:PyRPlidar = _LIDAR_OBJECT):
    """
    Disconnects the given lidar device.

    This function stops the lidar, sets its motor PWM to 0, and then disconnects it.

    Args:
        lidar (PyRPlidar): The lidar device to be disconnected.
    """
    global _LIDAR_OBJECT
    lidar.stop()
    lidar.set_motor_pwm(0)
    lidar.disconnect()
    _LIDAR_OBJECT = None

def lidarStatus(lidar:PyRPlidar = _LIDAR_OBJECT, verbose = True):
    """
    Gets the status of the given lidar device. This function retrieves the health, info, scan modes, and typical scan mode of the lidar device.

    Args:
        lidar (PyRPlidar): The lidar device for which to get the status.
        doPrint (bool, optional): Whether to print the status information. Defaults to True.
    
    Returns:
        dict: A dictionary containing the health, info, scan modes, and typical scan mode of the lidar device.
    """
    if lidar is None:
        return None
    health = lidar.get_health()
    info = lidar.get_info()
    scan_modes = lidar.get_scan_modes()
    typical_scan_mode = lidar.get_scan_mode_typical()
    
    if verbose:
        print("Health: ", health)
        print("Info: ", info)
        print("Scan Modes:")
        for i, mode in enumerate(scan_modes):
            print(f"Mode {i}: {mode}")
        print("Typical Scan Mode: ", typical_scan_mode)


    return {
        "health": health,
        "info": info,
        "scan_modes": scan_modes,
        "typical_scan_mode": typical_scan_mode
    }

def startScan(lidar:PyRPlidar = _LIDAR_OBJECT, mode=2):
    """
    Starts a scan on the given lidar device with the specified mode and returns a generator that yields the scan data.

    A generator is a special type of iterator that generates values on the fly. In this case, the generator yields the scan data as it is received from the lidar device. In english, a generator produces a sequence of values, one at a time, on demand. In this instance, each time the generator "runs", it produces a single scan data point from the lidar device.

    Be sure to consume the generator data quickly, as the lidar device will continue buffer the data until it is read. If the data is not read quickly enough, the buffer may overflow and data may be lost.

    Args:
        lidar (PyRPlidar): The lidar device on which to start the scan.
        mode (int, optional): The scan mode to use. Defaults to 2.

    Returns:
        generator: A generator that yields the scan data.
    """
    return lidar.start_scan_express(mode)

def stopScan(lidar:PyRPlidar = _LIDAR_OBJECT):
    """
    Stops the scan on the given lidar device.

    Args:
        lidar (PyRPlidar): The lidar device on which to stop the scan.
    """
    lidar.stop()
    # flush the serial buffer to ensure no data is left over
    # This is important to prevent data from being read incorrectly
    time.sleep(0.05)
    lidar.lidar_serial._serial.reset_input_buffer()

def setMotorPWM(lidar:PyRPlidar = _LIDAR_OBJECT, pwm=500):
    """
    Sets the motor PWM of the given lidar device.

    Args:
        lidar (PyRPlidar): The lidar device on which to set the motor PWM.
        pwm (int, optional): The PWM value to set. Defaults to 500.
    """
    # We call the underlying set_motor_pwm function from the PyRPlidar library
    # This will send the appropriate command to the lidar device to set the motor PWM
    lidar.set_motor_pwm(pwm)


def performSingleScan(lidar:PyRPlidar = _LIDAR_OBJECT , mode=2):
    """
    Initiates a single scan on the given lidar device and processes the scan data.

    This function connects to the given lidar device, starts an express scan with the specified mode, and processes the incoming scan data. The scan continues until a result is obtained and returns the angles and distances for a full rotation.

    Args:
        lidar (PyRPlidar): The lidar device on which to start the scan.
        mode (int, optional): The scan mode to use. Defaults to 2.
    """
    scan_generator = startScan(lidar, mode)
    scan_state = {"r":0, "buff":[], "doScan":False}
    for count, scan in enumerate(scan_generator()):
        # Start conusming the scan data
        scan_state, results = process_scan((count, scan), scan_state)
        if results:
            # We have a full scan, return the results, stop the scan
            stopScan(lidar)
            return results
        else:
            # We do not have a full scan yet, continue
            pass

def process_scan(incomingScanTuple, scanState = {"r":0, "buff":[], "doScan":False}):
    """
    Processes incoming scan data and manages scan rounds.
    This function processes incoming scan data from the RPLidar device, buffering scans and returning complete scan data for 1 full rotation.
    
    This function should be called for each incoming scan data tuple from the RPLidar device. It buffers the incoming scans until a full rotation is completed, then returns the angles and distances for the full rotation.

    State is maintained in the scanState dictionary, which keeps track of the current scan round (i.e., how many full rotations have been completed), the buffer of scans (i.e., the scan data points that have been received in the current rotation), and whether a scan is currently in progress (to keep track of the start of a new rotation).

    If a full rotation is not completed, the function returns the current state (encoded in the scanState dictionary) and None.

    If a full rotation is completed, the function returns the current state (encoded in the scanState dictionary) and a tuple of angles and distances, which is a tuple of two lists containing the angles and distances of the full rotation.

    Args:
        incomingScanTuple (tuple): A tuple containing the count of scans and a PyRPlidarMeasurement object.
        scanState (dict, optional): A dictionary to keep track of the current scan round, buffer, and scan status.
            Defaults to {"r": 0, "buff": [], "doScan": False}.
    Returns:
        tuple: A tuple containing the updated scanState dictionary and a tuple of angles and distances if a full scan is completed, otherwise None.
    """
    count = incomingScanTuple[0]
    scan:PyRPlidarMeasurement = incomingScanTuple[1]

    if scan.start_flag and not scanState["doScan"]:
        # We ignore all data until we see the start flag
        # Encountering the start_flag means we are at the start of a new scan
        scanState["doScan"] = True

    elif scan.start_flag and scanState["doScan"]:
        # We have a full scan, process the buffer
        buff:list = scanState["buff"]
        scanState["r"] += 1
        ang = tuple([x.angle for x in scanState["buff"]])
        dist = tuple([x.distance for x in scanState["buff"]])
        quality = tuple([x.quality for x in scanState["buff"]])
        # Clear the buffer to make way for the next scan
        buff.clear()
        # add current scan to the buffer as the first scan of the next round
        scanState["buff"].append(scan)
        # return the current state and the full scan data of the previous round
        return (scanState, (ang, dist, quality))

    if scanState["doScan"]:
        # add the scan to the buffer if we are in a scan
        scanState["buff"].append(scan)
        
    return (scanState, None)


def resampleLidarScan(distance, angles, 
                        target_measurements_per_scan = 360, offset_degrees = 0, 
                        merge_strategy=np.mean, 
                        fill_value=0):
    """
    Resample a LIDAR scan to a specified number of measurements per scan. This function is useful for downsampling the LIDAR data to a more manageable size for display or processing. This function can also apply an offset to the angles, allowing for the rotation of the LIDAR data to reorient the scans to a different reference frame. 

    For example, the lidar might produce 1080 measurements per scan. However, for our purposes, that is too much data. So we group (or Bin) the measurements by angle (i.e., 0-1 degrees, 1-2 degrees) and take the average of the distances in each group. This reduces the number of measurements to 360, which is more manageable. 
    
    Args:
        distance (list): A list of distance measurements from the LIDAR scan.
        angles (list): A list of angles corresponding to the distance measurements.
        target_measurements_per_scan (int, optional): The number of measurements to resample the scan to. Defaults to 360.
        offset_degrees (int, optional): The offset to add to the angles. Defaults to 0.
        merge_strategy (function, optional): The strategy to merge the distances within each angular bin. Defaults to np.mean. AKA the average of the distances within each bin.
        fill_value (int, optional): The value to use for empty bins. Defaults to 0.

    Returns:
        tuple: A tuple containing the resampled distances and angles.
    """

    # Handle adding the offset to the angles
    # create an array of angles using the numpy library
    angles = np.array(angles) 
    # Add the offset to the angles and wrap around to 0-360
    # For numpy arrays, if you add a scalar (i.e., a single number) to an array, it adds the scalar to each element of the array. This can also be applied to subtraction, multiplication, division, and modulo operations.
    # Example: If angles = [10, 20, 30] and offset_degrees = 5, then angles + offset_degrees = [15, 25, 35]
    # The modulo operation (%) is used to wrap the angles around to the range 0-360. For example, if the angles are [355, 5, 15] and the offset is 10, the result should be [5, 15, 25] (not [365, 15, 25]).
    angles = (angles + offset_degrees) % 360 

    # Calculate the target degree per measurement
    # This is targeted change in the angle for each distance measurement in the final result
    target_degree_per_measurement = 360 / target_measurements_per_scan 

    # Create angular bins. Each bin corresponds to a range of angles (i.e., 0-10 degrees, 10-20 degrees, etc.)
    # The bin limits are later used to group the distances that are within the same angular range
    # np.arange(start, stop, step) creates an array of values from start to stop with a specified step size
    bin_limits = np.array([x for x in np.arange(0,360,target_degree_per_measurement)]+[360]) # Create the bin limits [0 ... 360]

    # For each angle, determine the index of the bin to which it belongs
    indices = np.digitize(angles, bin_limits) 

    # Group the distances by the angular bins
    temp = [[] for _ in range(0, len(bin_limits)-1)]
    # For each distance, check the index of the bin to which it belongs and add it to the corresponding bin 
    for i, idx in enumerate(indices):
        temp[idx-1].append(distance[i])

    # Merge the distances within each bin using the specified merge strategy
    # In this case, we use the mean of the distances within each bin
    # Here list comprehension is used to apply the merge_strategy function to each bin
    # If the bin is empty (i.e., no distances), the fill_value is used
    new_distance = [(merge_strategy(y) if y!=[] else fill_value) for y in temp]
    
    # Return the resampled distances and angles
    # We use the left limit of each bin as the angle for the resampled data
    # For example, if the bin limits are 0-10, 10-20, 20-30 the angles will be [0, 10, 20]
    return list(new_distance), list(bin_limits[:-1])



            
