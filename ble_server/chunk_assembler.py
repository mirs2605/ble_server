"""
BLE Chunk Reassembly (ROS-free)
===============================
アプリ側はJSON末尾に ``\\n`` を付けてMTU単位に分割送信する
(BlePayloadChunker.split + WriteValue複数回)。
GATTのWriteValueはチャンク単位で届くため、``\\n`` 区切りで
フレーム復元してからconverterに渡す必要がある。

ROSへの依存はなし。ホストのpytestで検証できる。
"""

from __future__ import annotations


class AssemblerOverflowError(ValueError):
    """バッファ上限超過。バッファはクリア済み。再送待ちに戻る。"""


class ChunkAssembler:
    """``\\n`` 区切りフレームの再構成バッファ。

    Args:
        max_buffer_bytes: 蓄積上限。超過時はバッファを捨てて
            :class:`AssemblerOverflowError` を送出する。
            アプリの ``maxPayloadBytes`` (4096) より大きく取ること。
        delimiter: フレーム区切り。アプリ送信形式に合わせて ``\\n``。
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
        """未完成フレームの蓄積バイト数。"""
        return len(self._buf)

    def feed(self, data: bytes) -> list[bytes]:
        """チャンクを追加し、完成したフレーム列を返す(区切り文字は除く)。

        空フレーム(連続 ``\\n`` 等)は無視する。
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
        self._buf = bytearray(parts.pop())  # 末尾=未完成分を保持
        return [p for p in parts if p]
