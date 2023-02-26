import argparse
import subprocess
import os

def check_dashboard_running():
    stream = os.popen('ps aux')
    for line in stream:
        if line.strip().endswith("dashboard.py"):
            return True
    return False

def start_dashboard():
    if not check_dashboard_running():
        process = subprocess.Popen(['streamlit', 'run', 'dashboard/dashboard.py'], shell=False)
        while True:
            output = process.stdout.readline()
            if output == "" and process.poll() is not None:
                break
            if output:
                print(output.strip())
        rc = process.poll()
        return rc
    return True

def run_test_case(command):
    print(command)
    proc = subprocess.Popen(command, shell=False, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    while True:
        print("HERE")
        line = proc.stdout.readline()
        if not line and proc.poll() is not None:
            break
        print(line.strip())
    """
    while True:
        output = process.stdout.readline()
        if output == "" and process.poll() is not None:
            break
        if output:
            pass
            #print(output.strip())
    rc = process.poll()
    return rc
    """
    return True

def generate_test_filename(test_number):
    stream = os.popen('pwd')
    filename = stream.readlines()[0].strip() + f'/test_params/test_case_{test_number}.yaml'
    return filename

def generate_command(args):
    test_filename = generate_test_filename(args.test_number)
    if args.is_jetson:
        return ['rosrun', 'sdp', 'runTestCase.py', '-y', test_filename]
    else:
        if args.is_simulation:
            return ['python3', 'runTestCase.py', '-y', test_filename, '-s']
        else:
            return ['python3', 'runTestCase.py', '-y', test_filename]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--test_number', type=int)
    parser.add_argument('-s', '--is_simulation', action='store_true')
    parser.add_argument('-j', '--is_jetson', action='store_true')
    parser.add_argument('-d', '--dashboard', action='store_true')

    args = parser.parse_args()

    if args.dashboard:
        start_dashboard()
    else:
        is_dashboard_running = check_dashboard_running()
        if not is_dashboard_running:
            print("DASHBOARD NOT RUNNING")
        command = generate_command(args)
        run_test_case(command)
