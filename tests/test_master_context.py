"""Offline regression tests: never instantiate a real PySOEM master."""
import ast
import importlib.util
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
fake_pysoem = types.ModuleType("pysoem")
fake_pysoem.INIT_STATE = 1
fake_pysoem.SAFEOP_STATE = 4
fake_pysoem.OP_STATE = 8
spec = importlib.util.spec_from_file_location("_test_actuator_utils", ROOT / "scripts/utils.py")
u = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {"pysoem": fake_pysoem}):
    spec.loader.exec_module(u)

HARDWARE = {
    "send_cmd", "trig", "command", "read_status", "wait_until", "enable",
    "halt", "reset", "find_index", "move_abs",
    "set_param", "apply_default_settings", "pdo_settle", "prepare_actuator",
    "dpos", "check_bpf_actuator_ready",
    "halt_bpf", "move_bpf",
}


class FakeMaster:
    def __init__(self, failure=None, position=123):
        self.failure = failure
        self.events = []
        self.state = 0
        self.slaves = [
            types.SimpleNamespace(output=bytes(20), input=struct.pack("<i", position) + bytes(3))
            for _ in range(6)
        ]

    def open(self, ifname):
        self.events.append(("open", ifname))
        if self.failure == "open":
            raise RuntimeError("open failed")

    def config_init(self):
        self.events.append(("config_init",))
        return 0 if self.failure == "no_slaves" else 6

    def config_map(self):
        self.events.append(("config_map",))
        if self.failure == "map":
            raise RuntimeError("map failed")

    def state_check(self, state, timeout):
        self.events.append(("state_check", state, timeout))
        self.state = 0 if self.failure == ("state", state) else state

    def write_state(self):
        self.events.append(("write_state", self.state))
        if self.state == 1 and self.failure == "init_cleanup":
            raise RuntimeError("INIT failed")

    def close(self):
        self.events.append(("close",))
        if self.failure == "close":
            raise RuntimeError("close failed")

    def send_processdata(self):
        self.events.append(("send", self.slaves[0].output))

    def receive_processdata(self):
        self.events.append(("receive",))
        return 1


class MasterTests(unittest.TestCase):
    def factory(self, master):
        return patch.object(fake_pysoem, "Master", return_value=master, create=True)

    def assert_closed_once(self, master):
        self.assertEqual(master.events.count(("close",)), 1)

    def test_normal_lifetime_and_initialization_sequence(self):
        master = FakeMaster()
        with self.factory(master):
            with u.ethercat_master("eth0") as actual:
                self.assertIs(actual, master)
                self.assertEqual(master.state, 8)
                self.assertNotIn(("close",), master.events)
        self.assertEqual(master.events, [
            ("open", "eth0"), ("config_init",), ("config_map",),
            ("state_check", 4, u.ETHERCAT_STATE_CHECK_TIMEOUT_US),
            ("write_state", 8), ("state_check", 8, u.ETHERCAT_STATE_CHECK_TIMEOUT_US),
            ("write_state", 1), ("close",),
        ])

    def test_initialization_failures_close(self):
        for failure in ("open", "no_slaves", "map", ("state", 4), ("state", 8)):
            with self.subTest(failure=failure):
                master = FakeMaster(failure)
                with self.factory(master), self.assertRaises(RuntimeError):
                    with u.ethercat_master("eth0"):
                        self.fail("must not yield on initialization failure")
                self.assert_closed_once(master)
                if failure == "open":
                    self.assertNotIn(("write_state", 1), master.events)

    def test_body_exception_and_interrupt_preserved(self):
        for error in (RuntimeError("motion failed"), KeyboardInterrupt(), SystemExit(2)):
            with self.subTest(error=type(error)):
                master = FakeMaster()
                with self.factory(master), self.assertRaises(type(error)) as caught:
                    with u.ethercat_master("eth0"):
                        raise error
                self.assertIs(caught.exception, error)
                self.assert_closed_once(master)

    def test_cleanup_failure_does_not_mask_body_error(self):
        for failure in ("init_cleanup", "close"):
            with self.subTest(failure=failure):
                master = FakeMaster(failure)
                original = ValueError("primary failure")
                with self.factory(master), self.assertRaises(ValueError) as caught:
                    with u.ethercat_master("eth0"):
                        raise original
                self.assertIs(caught.exception, original)
                self.assertTrue(original.__notes__)
                self.assert_closed_once(master)

    def test_cleanup_failure_on_normal_exit_is_reported(self):
        for failure in ("init_cleanup", "close"):
            with self.subTest(failure=failure):
                master = FakeMaster(failure)
                with self.factory(master), self.assertRaises(RuntimeError):
                    with u.ethercat_master("eth0"):
                        pass
                self.assert_closed_once(master)

    def test_nested_masters_are_independent(self):
        outer, inner = FakeMaster(position=111), FakeMaster(position=222)
        with patch.object(fake_pysoem, "Master", side_effect=[outer, inner], create=True), patch.object(u.time, "sleep"):
            with u.ethercat_master("outer") as first:
                with u.ethercat_master("inner") as second:
                    self.assertEqual(u.read_status(first, 0)["pos"], 111)
                    self.assertEqual(u.read_status(second, 0)["pos"], 222)
                self.assert_closed_once(inner)
                self.assertNotIn(("close",), outer.events)
            self.assert_closed_once(outer)

    def test_command_payload_and_exchange_count_unchanged(self):
        master = FakeMaster()
        with patch.object(u.time, "sleep") as sleep:
            u.command(master, 0, b"DPOS", 800, 500000, 65000, 65000)
        packets = [event[1] for event in master.events if event[0] == "send"]
        self.assertEqual(packets, [
            struct.pack("<4siiHHB", b"DPOS", 800, 500000, 65000, 65000, execute).ljust(20, b"\x00")
            for execute in (0, 1)
        ])
        self.assertEqual(master.events.count(("receive",)), 2)
        self.assertEqual(sleep.call_count, 2)

    def test_hardware_call_graph_passes_master(self):
        tree = ast.parse((ROOT / "scripts/utils.py").read_text())
        self.assertFalse(any(isinstance(n, ast.Global) and "master" in n.names for n in ast.walk(tree)))
        for node in tree.body:
            if isinstance(node, ast.FunctionDef) and node.name in HARDWARE:
                self.assertEqual(node.args.args[0].arg, "master", node.name)
                self.assertIsNotNone(node.args.args[0].annotation)
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in HARDWARE:
                self.assertIsInstance(node.args[0], ast.Name)
                self.assertEqual(node.args[0].id, "master", node.func.id)

    def test_default_settings_keep_config_values_and_send_order(self):
        master = FakeMaster()
        for hpf in u.CONFIG["hpfs"].values():
            slave_id = hpf["slave_id"]
            with self.subTest(slave=slave_id), patch.object(u, "set_param") as send, patch.object(u.time, "sleep"):
                u.apply_default_settings(master, slave_id)
                expected = [
                    ("FREQ", hpf["FREQ"]), ("FRQ2", hpf["FRQ2"]),
                    *[(name, u.CONFIG["controller_defaults"][name]) for name in u.CONTROLLER_PARAMETER_ORDER],
                ]
                self.assertEqual(
                    [call.args for call in send.call_args_list],
                    [(master, slave_id, name, value) for name, value in expected],
                )

    def test_dpos_preserves_conversion_and_forwards_master(self):
        master = FakeMaster()
        with patch.object(u, "move_abs") as move, patch.object(u, "read_status", return_value={"pos": 800}) as read:
            u.dpos(master, 3, 1.0)
        move.assert_called_once_with(master, 3, round(1.0 / u.RESOLUTION_MM), u.DEFAULT_VEL, u.ACCEL, u.DECEL)
        read.assert_called_once_with(master, 3)


if __name__ == "__main__":
    unittest.main()
