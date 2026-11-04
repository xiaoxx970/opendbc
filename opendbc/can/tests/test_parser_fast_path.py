import glob
import os
import random

from opendbc.can.dbc import DBC
from opendbc.can.parser import CANParser, MessageState
from opendbc.can.packer import CANPacker
from opendbc.can.tests import TEST_DBC

DBC_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'dbc')


def byte_wise(state, dat):
  raw = []
  for sig in state.signals:
    tmp = sig.get_raw_value(dat)
    if sig.is_signed:
      tmp -= ((tmp >> (sig.size - 1)) & 0x1) * (1 << sig.size)
    raw.append(tmp)
  return raw


def test_whole_frame_reader_matches_byte_wise_reader():
  rng = random.Random(0)
  paths = sorted(glob.glob(os.path.join(DBC_PATH, '*.dbc')))
  assert len(paths) > 20
  for path in paths:
    dbc = DBC(os.path.basename(path)[:-4])
    for msg in dbc.addr_to_msg.values():
      state = MessageState(address=msg.address, name=msg.name, size=msg.size, signals=list(msg.sigs.values()))
      for length in (msg.size, max(msg.size - 1, 1)):  # a short frame takes the byte-wise path
        dat = bytes(rng.getrandbits(8) for _ in range(length))
        assert state.raw_values(dat) == byte_wise(state, dat), (path, msg.name)


def test_vl_all_empty_when_not_received():
  packer = CANPacker(TEST_DBC)
  parser = CANParser(TEST_DBC, [("CAN_FD_MESSAGE", 0)], 0)
  msg = packer.make_can_msg("CAN_FD_MESSAGE", 0, {"SIGNED": -3})
  parser.update([(1, [msg, msg])])
  assert parser.vl_all["CAN_FD_MESSAGE"]["SIGNED"] == [-3., -3.]
  parser.update([(2, [])])
  assert parser.vl_all["CAN_FD_MESSAGE"]["SIGNED"] == []
  assert parser.vl["CAN_FD_MESSAGE"]["SIGNED"] == -3.


def test_unregistered_message_read_registers_it():
  parser = CANParser(TEST_DBC, [], 0)
  assert parser.vl["CAN_FD_MESSAGE"]["SIGNED"] == 0.
  assert "CAN_FD_MESSAGE" in parser.vl
