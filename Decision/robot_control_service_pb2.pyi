from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class Empty(_message.Message):
    __slots__ = []
    def __init__(self) -> None: ...

class speed_message(_message.Message):
    __slots__ = ["speed"]
    SPEED_FIELD_NUMBER: _ClassVar[int]
    speed: int
    def __init__(self, speed: _Optional[int] = ...) -> None: ...

class turn_message(_message.Message):
    __slots__ = ["degree"]
    DEGREE_FIELD_NUMBER: _ClassVar[int]
    degree: int
    def __init__(self, degree: _Optional[int] = ...) -> None: ...
