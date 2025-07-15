#!/usr/bin/env python3

import time
import psutil
import csv

LOG_FILE = "raspi_resource_log.csv"
INTERFACE = 'wlan0'              # 모니터링할 네트워크 인터페이스 이름
PROCESS_KEYWORD = "oakd_container"  # 모니터링할 프로세스명 또는 커맨드 키워드 일부

class RaspiMonitor:
    def __init__(self, interval_sec=1):
        self.interval = interval_sec
        self._stop = False
        self.target_process = self.find_process_by_keyword(PROCESS_KEYWORD)

        # 네트워크 초기값 설정
        counters = psutil.net_io_counters(pernic=True)
        if INTERFACE not in counters:
            raise RuntimeError(f"[ERROR] 인터페이스 '{INTERFACE}'를 찾을 수 없습니다.")
        self.prev_net = counters[INTERFACE]

        # CSV 초기화
        self.csv_file = open(LOG_FILE, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "CPU%", "PID_CPU%", "Mem%", "TX_Mb", "RX_Mb"])

        # psutil 초기화
        psutil.cpu_percent()
        self.target_process.cpu_percent()
        time.sleep(1.0)   # ← 첫 측정에서 0이 안 나오도록 1초 대기

        self.cpu_list = []
        self.pid_cpu_list = []
        self.mem_list = []
        self.tx_list = []
        self.rx_list = []

    def find_process_by_keyword(self, keyword):
        """프로세스명 또는 커맨드 라인에 키워드가 포함된 첫 번째 프로세스를 반환"""
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmd = ' '.join(proc.info['cmdline'])
                if keyword in cmd:
                    print(f"▶ 타겟 프로세스 찾음: PID={proc.pid}, CMD='{cmd}'")
                    return psutil.Process(proc.pid)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        raise RuntimeError(f"[ERROR] '{keyword}'를 포함한 프로세스를 찾을 수 없습니다.")

    def start(self):
        print(f"모니터링 시작. 'Ctrl+C'로 종료하세요.")
        try:
            while not self._stop:
                self.log()
                time.sleep(self.interval)
        except KeyboardInterrupt:
            print("중단 요청됨. 종료합니다.")
        finally:
            self.write_average()
            self.csv_file.close()

    def log(self):
        cpu = psutil.cpu_percent()
        pid_cpu = self.target_process.cpu_percent()
        mem = psutil.virtual_memory().percent

        counters = psutil.net_io_counters(pernic=True)
        net_now = counters[INTERFACE]

        # MB -> Mb 변경: bytes * 8 / (1024 * 1024)
        tx = (net_now.bytes_sent - self.prev_net.bytes_sent) * 8 / (1024 * 1024)
        rx = (net_now.bytes_recv - self.prev_net.bytes_recv) * 8 / (1024 * 1024)
        self.prev_net = net_now

        timestamp = time.strftime("%H:%M:%S")
        self.csv_writer.writerow([timestamp, cpu, pid_cpu, mem, f"{tx:.2f}", f"{rx:.2f}"])
        print(f"[{timestamp}] CPU: {cpu}% | PID_CPU: {pid_cpu:.2f}% | MEM: {mem}% | TX: {tx:.2f}Mb | RX: {rx:.2f}Mb")

        self.cpu_list.append(cpu)
        self.pid_cpu_list.append(pid_cpu)
        self.mem_list.append(mem)
        self.tx_list.append(tx)
        self.rx_list.append(rx)

    def write_average(self):
        if self.cpu_list:
            avg_cpu = sum(self.cpu_list) / len(self.cpu_list)
            avg_pid_cpu = sum(self.pid_cpu_list) / len(self.pid_cpu_list)
            avg_mem = sum(self.mem_list) / len(self.mem_list)
            avg_tx = sum(self.tx_list) / len(self.tx_list)
            avg_rx = sum(self.rx_list) / len(self.rx_list)

            self.csv_writer.writerow([])
            self.csv_writer.writerow([
                "AVERAGE",
                f"{avg_cpu:.2f}",
                f"{avg_pid_cpu:.2f}",
                f"{avg_mem:.2f}",
                f"{avg_tx:.2f}",
                f"{avg_rx:.2f}"
            ])
            print(f"\n종료 - 평균값 기록됨: CPU {avg_cpu:.2f}%, PID_CPU {avg_pid_cpu:.2f}%, MEM {avg_mem:.2f}%, TX {avg_tx:.2f}Mb, RX {avg_rx:.2f}Mb")

if __name__ == '__main__':
    monitor = RaspiMonitor(interval_sec=1)
    monitor.start()
