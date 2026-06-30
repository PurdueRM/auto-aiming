import subprocess
import threading
import psutil
import time

num_of_success = 0
total_restarts = 0

def run(command, success_string, failure_string, kill_strings, num_checks):
    global num_of_success
    num_of_success = 0
    command_thread = threading.Thread(target=run_command, args=(command, success_string, failure_string, kill_strings, num_checks))
    command_thread.start()
    #command_thread.join()  # Wait for the completion of the command

def run_command(command, success_strings, failure_strings, kill_strings, num_checks):
    global num_of_success
    global total_restarts
    success = 0
    tf_failure_count = 0  # Counter for tf failures
    
    for kill_string in kill_strings:
        kill_processes(kill_string)

    print("Running command:", command)

    while True:
        start_time = time.time()  # Start time of the command execution
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        for line in process.stdout:
            print("        " + line.strip())

            if not success:
                if all(success_string in line for success_string in success_strings):
                    num_of_success += 1
                    print("  [✔] SUCCESS string found (", num_of_success, "/", num_checks, ")")
                    
                elif any(failure_string in line for failure_string in failure_strings):
                    print("  [x] FAILURE string found")

                    # see if it's a process has died and reset eth
                    # "Message Filter dropping message" in line or 
                    if "Init lds lidar fail!" in line or "frame ID \"map\" passed to canTransform argument" in line:
                        print("  [x] LiDAR Fail detected. Resetting ETH devices...")
                        try:
                            subprocess.run("echo purdueRM2023 | sudo -S systemctl restart NetworkManager", shell=True, check=True)
                        except exception as e:
                            print("  [x] Failed to reset ETH devices:", e)
                            
                    process.terminate()
                    for kill_string in kill_strings:
                        kill_processes(kill_string)
                    total_restarts += 1
                    check_total_restarts()
                    break  # Break the inner loop and retry

                # See if tf error with lidar and restart NetworkManager
                if "Timed out waiting for transform from base_link to map to become available" in line:
                    tf_failure_count += 1
                    print(f"  [!] Detected tf error ({tf_failure_count})")
                    if tf_failure_count >= 3:
                        print("  [!] tf error repeated 3+ times. Restarting NetworkManager...")
                        try:
                            subprocess.run("echo purdueRM2023 | sudo -S systemctl restart NetworkManager", shell=True, check=True)
                        except Exception as e:
                            print("  [x] Failed to restart NetworkManager:", e)
                        tf_failure_count = 0  # Reset counter after restart


                if num_of_success != 0 and any(success_string not in line for success_string in success_strings):
                    print("  [x] SUCCESS string not found, resetting... (", round(time.time() - start_time, 1), "/ 15 until reset)")
                    num_of_success = 0

                if num_of_success == num_checks:
                    print("  [✔] Navigation startup completed.")
                    success = 1
                    #return


                # Check if the command has been running for more than 10 seconds
                if time.time() - start_time > 17:
                    print("     Command execution timed out (more than 17 seconds). Retrying...")
                    for kill_string in kill_strings:
                        kill_processes(kill_string)
                    total_restarts += 1

                    # if about to restart for a 4th time (3 attempts), reset the system
                    check_total_restarts()
                   
                    break
        time.sleep(1)

def check_total_restarts():
    global total_restarts
    total_restarts = 0

def kill_processes(kill_string):
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        if kill_string in ' '.join(proc.info['cmdline']):
            # try to kill the process
            try:
                print("  [x] Killing process:", proc.info['name'])
                proc.terminate()
            except Exception as e:
                print("  [x] Failed to kill process")

def main():
    if (subprocess.run("lsusb | grep MindVision | grep 'Bus 002'", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE).returncode):
        print("Camera on low speed bus\n") 
    else:
        print("Camera on high speed bus\n")    

    #  "Message Filter dropping message: frame 'odom'", 
    run("ros2 launch prm_autobot_2023 autobot_launch.py", ["[pose_scheduler_sm]: Publishing"], ["Init lds lidar fail!", "Bad Odom read", "process has died", "Please set the initial pose"], ["ros2", "mv2pnp.py", "livox", "MVCameraNode", "OpenCVArmorDete", "PNPSolverNode", "subscriber.py", "autobot_launch.py", "waypoint_follow", "recoveries_serv", "bt_navigator", "controller_serv", "planner_server", "lifecycle_manag", "amcl", "map_server", "rviz2", "ScanLimitNode", "rplidar"], 1)


    print("Navigation startup completed.")
if __name__ == "__main__":
    main()