"""Unit tests for ChunkAssembler (ROS-free, host-runnable)."""

import pytest

from ble_server.chunk_assembler import AssemblerOverflowError, ChunkAssembler


def test_single_complete_frame():
    asm = ChunkAssembler()
    assert asm.feed(b'{"a": 1}\n') == [b'{"a": 1}']
    assert asm.pending_bytes == 0


def test_split_across_feeds():
    asm = ChunkAssembler()
    assert asm.feed(b'{"a": ') == []
    assert asm.pending_bytes == len(b'{"a": ')
    assert asm.feed(b'1}\n') == [b'{"a": 1}']
    assert asm.pending_bytes == 0


def test_multiple_frames_in_one_feed():
    asm = ChunkAssembler()
    assert asm.feed(b'{}\n{}\n') == [b'{}', b'{}']


def test_partial_tail_kept():
    asm = ChunkAssembler()
    assert asm.feed(b'{}\n{"b":') == [b'{}']
    assert asm.feed(b' 2}\n') == [b'{"b": 2}']


def test_empty_frames_ignored():
    asm = ChunkAssembler()
    assert asm.feed(b'\n\n{}\n') == [b'{}']


def test_empty_feed_noop():
    asm = ChunkAssembler()
    assert asm.feed(b'') == []
    assert asm.pending_bytes == 0


def test_overflow_clears_and_raises():
    asm = ChunkAssembler(max_buffer_bytes=8)
    with pytest.raises(AssemblerOverflowError):
        asm.feed(b'123456789')
    assert asm.pending_bytes == 0
    # クリア後は再利用できる
    assert asm.feed(b'{}\n') == [b'{}']


def test_invalid_args():
    with pytest.raises(ValueError):
        ChunkAssembler(max_buffer_bytes=0)
    with pytest.raises(ValueError):
        ChunkAssembler(delimiter=b'')
