"""Small process lock used to keep test ROS publishers single-authority."""
import fcntl
import os
from pathlib import Path
from typing import TextIO


class DuplicateProcessError(RuntimeError):
    """Raised when the same authoritative node already exists in this domain."""


def acquire_process_lock(role: str) -> TextIO:
    """Acquire a non-blocking lock that is released automatically on process exit."""
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    path = Path('/tmp') / f'mmission_{role}_domain_{domain}.lock'
    stream = path.open('a+', encoding='utf-8')
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        stream.close()
        raise DuplicateProcessError(
            f'{role} is already running in ROS_DOMAIN_ID={domain}') from exc
    stream.seek(0)
    stream.truncate()
    stream.write(str(os.getpid()))
    stream.flush()
    return stream
