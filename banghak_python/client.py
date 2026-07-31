import socket
import pickle
import cv2
import time
from picamera2 import Picamera2

SERVER_IP = '192.168.0.121'  # PC(서버) IP 주소
SERVER_PORT = 5051

def send_frame_get_detections(frame, sock):
    """
    한 프레임을 서버로 보내고 객체 인식 결과를 받아옴
    """
    success, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    data = encoded.tobytes()

    sock.sendall(len(data).to_bytes(4, 'big'))
    sock.sendall(data)

    raw_len = sock.recv(4)
    result_len = int.from_bytes(raw_len, 'big')
    result = b''
    while len(result) < result_len:
        result += sock.recv(4096)

    detections = pickle.loads(result)
    return detections


def main():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
    time.sleep(2)

    print(f"서버({SERVER_IP}:{SERVER_PORT})에 연결 시도 중...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((SERVER_IP, SERVER_PORT))
    print("서버 연결됨!")

    try:
        while True:
            frame = picam2.capture_array()
            #frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            detections = send_frame_get_detections(frame, sock)

            # 인식된 물체마다 박스 + 이름 + 확신도를 화면에 그림
            # 확신도(conf)가 0.4 이상인 것만 표시 (낮은 건 오탐일 가능성 높아서 필터링)
            for d in detections:
                if d['conf'] < 0.4:
                    continue
                x1, y1, x2, y2 = [int(v) for v in d['box']]
                label = f"{d['label']} {d['conf']:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 실시간 화면 표시 (모니터 연결된 상태에서만 작동함)
            cv2.imshow('Object Detection (Offloaded)', frame)

            # 'q' 키 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("종료합니다...")
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        sock.close()


if __name__ == '__main__':
    main()