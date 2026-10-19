"""BLE WriteValueで分割受信した改行区切りフレームを再構成する。"""

from __future__ import annotations


class AssemblerOverflowError(ValueError):
    """受信フレームが蓄積上限を超え、未完成データを破棄したことを示す。"""


class ChunkAssembler:
    """区切り文字で分割された入力を、完全なフレーム列へ再構成する。

    Args:
        max_buffer_bytes: 未完成データの蓄積上限 [bytes]。超過時はバッファを捨てて
            :class:`AssemblerOverflowError` を送出する。
        delimiter: フレーム終端として扱う非空のバイト列。既定値は改行1 byte。
    """

    def __init__(
        self,
        max_buffer_bytes: int = 8192,
        delimiter: bytes = b"\n",
    ) -> None:
        if max_buffer_bytes <= 0:
            raise ValueError("max_buffer_bytes must be positive")
        if not delimiter:
            raise ValueError("delimiter must not be empty")
        self._max_buffer_bytes = max_buffer_bytes
        self._delimiter = delimiter
        self._buf = bytearray()

    @property
    def pending_bytes(self) -> int:
        """次の区切り文字を待つ未完成データのサイズ [bytes]。"""
        return len(self._buf)

    def feed(self, data: bytes) -> list[bytes]:
        """チャンクを追加し、区切り文字を除いた完全フレームを返す。

        空入力と空フレームは無視する。末尾の未完成データは次回まで保持する。
        """
        if not data:
            return []
        self._buf += data
        if len(self._buf) > self._max_buffer_bytes:
            self._buf.clear()
            raise AssemblerOverflowError(
                f"buffer exceeded {self._max_buffer_bytes} bytes; cleared"
            )
        parts = bytes(self._buf).split(self._delimiter)
        self._buf = bytearray(parts.pop())
        return [p for p in parts if p]
