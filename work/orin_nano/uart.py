import queue
import threading

try:
    import serial
except ImportError:
    serial = None


# STM32와 약속된 송신 명령
# 회의 결과에 따라 VENT_OFF는 사용하지 않는다.
VALID_COMMANDS = {
    "DROWSY_WARN",
    "DROWSY_OK",
    "VENT_ON",
    "WIN_CLOSE",
    "WIN_OPEN",
    "SIDE_WARN",
}


class UARTCommunication:
    """Jetson과 STM32 사이의 UART 송수신을 관리한다."""

    def __init__(
        self,
        port="/dev/ttyTHS1",
        baud_rate=115200,
        timeout=0.1,
    ):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout

        self.serial_port = None
        self.running = False
        self.reader_thread = None

        self.write_lock = threading.Lock()
        self.message_queue = queue.Queue()

    @property
    def is_open(self):
        """UART 포트가 열려 있는지 반환한다."""
        return (
            self.serial_port is not None
            and self.serial_port.is_open
        )

    def start(self):
        """UART 포트를 열고 수신 스레드를 시작한다."""
        if serial is None:
            self._put_message(
                "ERROR",
                "pyserial이 설치되어 있지 않습니다. "
                "python -m pip install pyserial을 실행하세요.",
            )
            return False

        if self.is_open:
            return True

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                write_timeout=0.5,
            )
        except (OSError, serial.SerialException) as error:
            self.serial_port = None
            self._put_message(
                "ERROR",
                f"UART 연결 실패 ({self.port}): {error}",
            )
            return False

        self.running = True
        self.reader_thread = threading.Thread(
            target=self._read_loop,
            daemon=True,
        )
        self.reader_thread.start()

        self._put_message(
            "INFO",
            f"STM32 UART 연결됨: {self.port}, "
            f"{self.baud_rate} bps, 8-N-1",
        )
        return True

    def send_command(self, command):
        """명령 뒤에 LF(\n)를 붙여 ASCII 패킷으로 전송한다."""
        if command not in VALID_COMMANDS:
            self._put_message(
                "ERROR",
                f"정의되지 않은 UART 명령: {command}",
            )
            return False

        if not self.is_open:
            self._put_message(
                "ERROR",
                f"UART 미연결로 전송 실패: {command}",
            )
            return False

        packet = f"{command}\n".encode("ascii")

        try:
            with self.write_lock:
                self.serial_port.write(packet)

            self._put_message("TX", command)
            return True
        except (OSError, serial.SerialException) as error:
            self._put_message(
                "ERROR",
                f"UART 전송 실패 ({command}): {error}",
            )
            return False

    def _read_loop(self):
        """STM32가 보내는 개행 단위 응답을 백그라운드에서 읽는다."""
        while self.running and self.is_open:
            try:
                received = self.serial_port.readline()

                if not received:
                    continue

                message = received.decode(
                    "ascii",
                    errors="replace",
                ).strip("\r\n")

                if message:
                    self._put_message("RX", message)

            except (OSError, serial.SerialException) as error:
                if self.running:
                    self._put_message(
                        "ERROR",
                        f"UART 수신 오류: {error}",
                    )
                break

    def _put_message(self, message_type, content):
        """GUI에서 출력할 UART 로그를 큐에 저장한다."""
        self.message_queue.put((message_type, content))

    def get_messages(self):
        """현재까지 쌓인 UART 로그를 모두 반환한다."""
        messages = []

        while True:
            try:
                messages.append(self.message_queue.get_nowait())
            except queue.Empty:
                break

        return messages

    def close(self):
        """수신 스레드와 UART 포트를 안전하게 종료한다."""
        self.running = False

        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)
            self.reader_thread = None

        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except (OSError, serial.SerialException):
                pass
            finally:
                self.serial_port = None

