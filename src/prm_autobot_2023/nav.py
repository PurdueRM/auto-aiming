import subprocess
import threading
import psutil
import time

num_of_success = 0
total_restarts = 0


def run(command, success_string, failure_string, kill_strings, num_checks):
    global num_of_success
    num_of_success = 0
    command_thread = threading.Thread(
        target=run_command,
        args=(command, success_string, failure_string, kill_strings, num_checks),
        daemon=True,
    )
    command_thread.start()


def run_command(command, success_strings, failure_strings, kill_strings, num_checks):
    global num_of_success
    global total_restarts

    tf_failure_count = 0

    while True:
        success = False
        num_of_success = 0
        start_time = time.monotonic()
        timeout_deadline = start_time + 17.0

        for kill_string in kill_strings:
            kill_processes(kill_string)

        print("Running command:", command)
        process = subprocess.Popen(
            command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
        )

        try:
            for line in process.stdout:
                cleaned = line.strip()
                if not cleaned:
                    continue
                print("        " + cleaned)

                if any(failure_string in cleaned for failure_string in failure_strings):
                    print("  [x] FAILURE string found")

                    if "Init lds lidar fail!" in cleaned or "frame ID \"map\" passed to canTransform argument" in cleaned:
                        print("  [x] LiDAR Fail detected. Resetting ETH devices...")
                        try:
                            subprocess.run("echo purdueRM2023 | sudo -S systemctl restart NetworkManager", shell=True, check=True)
                        except Exception as e:
                            print("  [x] Failed to reset ETH devices:", e)

                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                    for kill_string in kill_strings:
                        kill_processes(kill_string)
                    total_restarts += 1
                    check_total_restarts()
                    break

                if "Timed out waiting for transform from base_link to map to become available" in cleaned:
                    tf_failure_count += 1
                    print(f"  [!] Detected tf error ({tf_failure_count})")
                    if tf_failure_count >= 3:
                        print("  [!] tf error repeated 3+ times. Restarting NetworkManager...")
                        try:
                            subprocess.run("echo purdueRM2023 | sudo -S systemctl restart NetworkManager", shell=True, check=True)
                        except Exception as e:
                            print("  [x] Failed to restart NetworkManager:", e)
                        tf_failure_count = 0

                if all(success_string in cleaned for success_string in success_strings):
                    num_of_success += 1
                    print("  [✔] SUCCESS string found (", num_of_success, "/", num_checks, ")")
                    if num_of_success >= num_checks:
                        print("  [✔] Navigation startup completed.")
                        success = True
                        break

                if time.monotonic() >= timeout_deadline:
                    print("     Command execution timed out (more than 17 seconds). Retrying...")
                    for kill_string in kill_strings:
                        kill_processes(kill_string)
                    total_restarts += 1
                    check_total_restarts()
                    break

            if success:
                return

            if process.poll() is not None and process.returncode != 0:
                print(f"  [x] Command exited with return code {process.returncode}. Restarting...")
                total_restarts += 1
                check_total_restarts()

        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
            if process is not None and process.stdout is not None:
                process.stdout.close()

        time.sleep(1)


def check_total_restarts():
    global total_restarts
    if total_restarts >= 3:
        print("  [!] Restart threshold reached; resetting restart counter.")
        total_restarts = 0


def kill_processes(kill_string):
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        cmdline = proc.info.get('cmdline') or []
        cmdline_str = ' '.join(str(part) for part in cmdline)
        if not kill_string or kill_string not in cmdline_str:
            continue
        try:
            print("  [x] Killing process:", proc.info.get('name'))
            proc.terminate()
            proc.wait(timeout=2)
        except (psutil.NoSuchProcess, psutil.TimeoutExpired):
            try:
                proc.kill()
            except Exception:
                pass
        except Exception:
            print("  [x] Failed to kill process")


def main():
    if (subprocess.run("lsusb | grep MindVision | grep 'Bus 002'", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE).returncode):
        print("Camera on low speed bus\n")
    else:
        print("Camera on high speed bus\n")

    run(
        "ros2 launch prm_autobot_2023 autobot_launch.py",
        ["[pose_scheduler_sm]: Publishing"],
        ["Init lds lidar fail!", "Bad Odom read", "process has died", "Please set the initial pose"],
        ["ros2", "mv2pnp.py", "livox", "MVCameraNode", "OpenCVArmorDete", "PNPSolverNode", "subscriber.py", "autobot_launch.py", "waypoint_follow", "recoveries_serv", "bt_navigator", "controller_serv", "planner_server", "lifecycle_manag", "amcl", "map_server", "rviz2", "ScanLimitNode", "rplidar"],
        1,
    )

    print("Navigation startup completed.")


if __name__ == "__main__":
    main()