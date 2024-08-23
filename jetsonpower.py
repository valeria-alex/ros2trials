import time
import subprocess

def log_tegrastats(logfile='power_log.txt', duration=60, interval=1):
    with open(logfile, 'w') as f:
        process = subprocess.Popen(['sudo', 'tegrastats', '--interval', str(interval * 1000)], stdout=subprocess.PIPE)
        start_time = time.time()

        try:
            while time.time() - start_time < duration:
                output = process.stdout.readline().decode('utf-8')
                f.write(output)
                print(output, end='')
        except KeyboardInterrupt:
            pass
        finally:
            process.terminate()

def parse_tegrastats_log(logfile='power_log.txt'):
    with open(logfile, 'r') as f:
        for line in f:
            if "POM_5V_IN" in line:
                power_consumption = line.split("POM_5V_IN")[1].split()[0]
                print(f"Power Consumption: {power_consumption} mW")

if __name__ == "__main__":
    log_tegrastats()
    parse_tegrastats_log()
