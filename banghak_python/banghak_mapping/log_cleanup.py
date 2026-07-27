#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
log_cleanup.py
================
역할: logs 폴더에 쌓이는 파일들 중, 지정한 개수(KEEP_LAST_N)만 남기고
      오래된 파일부터 자동으로 삭제해주는 작은 유틸리티입니다.

사용법 (다른 스크립트 맨 위, 파일 생성하기 전에 딱 한 줄만 추가):

    from log_cleanup import cleanup_old_logs

    cleanup_old_logs("logs", "lidar_*.csv", keep_last=5)
    # -> logs 폴더 안에서 "lidar_*.csv" 패턴에 맞는 파일 중
    #    가장 최근 5개만 남기고 나머지는 삭제합니다.

    cleanup_old_logs("logs", "slam_map_2*.png", keep_last=3)
    # -> slam_map_latest.png 는 패턴에 안 걸리게(2로 시작하는 타임스탬프 파일만) 해서
    #    실시간 미리보기용 파일은 안 지워지게 만든 예시입니다.

동작 방식:
    1. 폴더 안에서 pattern(와일드카드)에 맞는 파일들을 전부 찾음
    2. "수정된 시각(mtime)" 기준으로 오래된 순 -> 최신 순으로 정렬
    3. keep_last개를 넘는 만큼, 오래된 파일부터 하나씩 삭제

주의:
    - 삭제는 되돌릴 수 없습니다(휴지통 안 거침). 정말 중요한 로그는
      logs 폴더가 아닌 다른 곳에 따로 백업해두세요.
    - keep_last=0 으로 주면 패턴에 맞는 파일을 전부 삭제합니다.
      (="테스트 끝나면 무조건 다 지우기" 원하시면 이렇게 쓰시면 됩니다)
"""

import glob
import os


def cleanup_old_logs(log_dir, pattern, keep_last=5, verbose=True):
    """
    log_dir 폴더 안에서 pattern에 맞는 파일들 중, 최신 keep_last개만 남기고
    나머지(오래된 것부터)를 삭제합니다.

    log_dir  : 로그가 들어있는 폴더 경로 (예: "logs")
    pattern  : glob 와일드카드 패턴 (예: "lidar_*.csv", "slam_map_2*.png")
    keep_last: 남겨둘 최근 파일 개수 (기본 5개)
    verbose  : True면 어떤 파일을 지웠는지 화면에 출력
    """
    # log_dir 자체가 없으면(아직 한 번도 기록 안 한 상태) 아무 것도 안 하고 종료
    if not os.path.isdir(log_dir):
        return

    # pattern에 맞는 파일 전체 경로 목록
    search_path = os.path.join(log_dir, pattern)
    files = glob.glob(search_path)

    if len(files) <= keep_last:
        # 아직 개수가 기준 이하면 지울 게 없음
        return

    # 수정 시각(mtime) 기준 오름차순 정렬 -> 리스트 맨 앞이 가장 오래된 파일
    files.sort(key=os.path.getmtime)

    # keep_last개를 뺀 나머지(앞쪽, 오래된 것들)를 삭제 대상으로 추림
    files_to_delete = files[: len(files) - keep_last]

    for path in files_to_delete:
        try:
            os.remove(path)
            if verbose:
                print(f"[정리] 오래된 로그 삭제: {path}")
        except OSError as e:
            if verbose:
                print(f"[정리 실패] {path} : {e}")


# ============================================================
# 단독 실행 테스트
#   python3 log_cleanup.py 로 직접 실행하면
#   logs 폴더의 라이다 csv 로그를 최근 5개만 남기고 정리합니다.
# ============================================================
if __name__ == "__main__":
    cleanup_old_logs("logs", "lidar_*.csv", keep_last=5)
    cleanup_old_logs("logs", "slam_map_2*.png", keep_last=3)
    print("정리 완료.")
