from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Empty(_message.Message):
    __slots__ = []
    def __init__(self) -> None: ...

class image_chunk(_message.Message):
    __slots__ = ["image"]
    IMAGE_FIELD_NUMBER: _ClassVar[int]
    image: bytes
    def __init__(self, image: _Optional[bytes] = ...) -> None: ...

class slam_kpts(_message.Message):
    __slots__ = []
    def __init__(self) -> None: ...

class slam_payload(_message.Message):
    __slots__ = ["img", "kpts", "tripoints3d"]
    IMG_FIELD_NUMBER: _ClassVar[int]
    KPTS_FIELD_NUMBER: _ClassVar[int]
    TRIPOINTS3D_FIELD_NUMBER: _ClassVar[int]
    img: image_chunk
    kpts: slam_kpts
    tripoints3d: slam_points
    def __init__(self, img: _Optional[_Union[image_chunk, _Mapping]] = ..., tripoints3d: _Optional[_Union[slam_points, _Mapping]] = ..., kpts: _Optional[_Union[slam_kpts, _Mapping]] = ...) -> None: ...

class slam_points(_message.Message):
    __slots__ = ["x", "y", "z"]
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ...) -> None: ...
