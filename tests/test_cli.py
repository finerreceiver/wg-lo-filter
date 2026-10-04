"""Typer and safety regression tests; all hardware calls are mocked."""
import importlib.util
from pathlib import Path
import sys
import unittest
from contextlib import ExitStack
from unittest.mock import patch

from typer.testing import CliRunner
import typer.rich_utils  # Load Rich classes before CliRunner's isolated imports.
from test_master_context import ROOT, FakeMaster, fake_pysoem, u

spec = importlib.util.spec_from_file_location("lo_filter_cli", ROOT / "scripts/lo-filter.py")
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)
runner = CliRunner()


def ready_status(pos=0):
    status = {name: False for name in u.STATUS_BITS}
    status.update(pos=pos, enabled=True, encoder_valid=True, position_reached=True,
                  ecat_ack=True, status_raw="0x80541")
    return status


class CliTests(unittest.TestCase):
    def invoke(self, args, patches=None):
        master = FakeMaster()
        with ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules, {"utils": u}))
            stack.enter_context(patch.object(fake_pysoem, "Master", return_value=master, create=True))
            stack.enter_context(patch.object(u.time, "sleep"))
            stack.enter_context(patch.object(u, "read_status", return_value=ready_status()))
            mocks = {name: stack.enter_context(patch.object(u, name, **kwargs))
                     for name, kwargs in (patches or {}).items()}
            result = runner.invoke(cli.app, args)
            u.configure_logging("CRITICAL")  # Do not retain CliRunner's closed stderr.
        return result, master, mocks

    def test_help_and_each_subcommand_help_without_hardware(self):
        for command in ([], ["prepare"], ["set"], ["read-status"], ["halt"], ["reset"]):
            with self.subTest(command=command):
                result, master, _ = self.invoke([*command, "--help"])
                self.assertEqual(result.exit_code, 0, result.output)
                self.assertEqual(master.events, [])

    def test_required_set_options_and_no_hpf_selector(self):
        for args in (["set"], ["set", "--rx", "6+7", "--lo", "230"],
                     ["set", "--rx", "6+7", "--lo", "230", "--bw", "4", "--hpf-id", "5"]):
            result, master, _ = self.invoke(args)
            self.assertNotEqual(result.exit_code, 0)  # Typer usage error, not a custom control exit code.
            self.assertEqual(master.events, [])

    def test_exclusive_target_selector_before_open(self):
        for command in ("prepare", "read-status", "halt", "reset"):
            for options in ([], ["--rx", "6+7", "--hpf-id", "5"], ["--hpf-id", "99"], ["--rx", "bad"]):
                result, master, _ = self.invoke([command, *options])
                self.assertEqual(master.events, [])
                self.assertIn("ERROR", result.output)

    def test_invalid_numeric_values_before_open(self):
        for lo, bw in (("nan", "4"), ("inf", "4"), ("0", "4"), ("230", "-1"), ("230", "nan")):
            result, master, _ = self.invoke(["set", "--rx", "6+7", "--lo", lo, "--bw", bw])
            self.assertEqual(master.events, [])
            self.assertIn("ERROR", result.output)

    def test_rx_multiplier_and_bpf_mapping(self):
        for rx, lo, bpf, multiplier in (("4+5", "160", 1, 2), ("6+7", "230", 2, 3)):
            result, master, mocks = self.invoke(
                ["set", "--rx", rx, "--lo", lo, "--bw", "4"],
                {"move_bpf": {"return_value": {}}},
            )
            self.assertEqual(result.exit_code, 0, result.output)
            mocks["move_bpf"].assert_called_once_with(master, bpf, float(lo) / multiplier, 4.0)
            self.assertEqual(master.events.count(("close",)), 1)

    def test_target_mapping_for_all_non_set_commands(self):
        function = {"prepare": "prepare_selected", "read-status": "log_selected_status",
                    "halt": "halt_selected", "reset": "reset_selected"}
        for command, name in function.items():
            for opts, bpf, ids in ((["--rx", "6+7"], 2, [3, 4, 5]),
                                   (["--hpf-id", "5"], 2, [4])):
                with self.subTest(command=command, options=opts):
                    result, master, mocks = self.invoke([command, *opts], {name: {}})
                    expected = (master, bpf, ids) if command == "prepare" else (master, ids)
                    mocks[name].assert_called_once_with(*expected)
                    self.assertEqual(master.events.count(("close",)), 1)

    def test_read_status_does_not_enable_reset_or_prepare(self):
        result, master, mocks = self.invoke(["read-status", "--hpf-id", "5"], {
            "enable": {}, "reset": {}, "prepare_actuator": {},
        })
        self.assertIn("position", result.output)
        for mock in mocks.values():
            mock.assert_not_called()

    def test_log_level_case_insensitive_and_invalid_level_before_open(self):
        result, _, _ = self.invoke(["read-status", "--hpf-id", "5", "--log-level", "warning"])
        self.assertNotIn("position", result.output)
        result, master, _ = self.invoke(["read-status", "--hpf-id", "5", "--log-level", "bogus"])
        self.assertNotEqual(result.exit_code, 0)
        self.assertEqual(master.events, [])


class MotionTests(unittest.TestCase):
    def test_preparation_failure_or_interrupt_halts_whole_bpf(self):
        for error in (TimeoutError("index timeout"), RuntimeError("drive error"), KeyboardInterrupt()):
            with self.subTest(error=type(error)), patch.object(u, "prepare_actuator"), patch.object(u, "find_index", side_effect=error), patch.object(u, "halt") as halt, patch.object(u, "read_status", return_value=ready_status()), patch.object(u.time, "sleep"):
                with self.assertRaises(type(error)) as caught:
                    u.prepare_selected(FakeMaster(), 2, [4])
                self.assertIs(caught.exception, error)
                self.assertEqual([c.args[1] for c in halt.call_args_list], [3, 4, 5])

    def test_set_motion_failure_or_interrupt_halts_whole_bpf(self):
        for error in (TimeoutError("move timeout"), KeyboardInterrupt()):
            master = FakeMaster()
            with patch.object(u, "read_status", return_value=ready_status()), patch.object(u, "dpos", side_effect=error), patch.object(u, "halt") as halt, patch.object(u.time, "sleep"):
                with self.assertRaises(type(error)):
                    u.move_bpf(master, 2, 230 / 3, 4)
                self.assertEqual([c.args for c in halt.call_args_list], [(master, 3), (master, 4), (master, 5)])

    def test_unprepared_set_has_no_motion_no_prepare_no_halt(self):
        for field in ("enabled", "encoder_valid"):
            status = ready_status()
            status[field] = False
            with patch.object(u, "read_status", return_value=status), patch.object(u, "dpos") as move, patch.object(u, "prepare_actuator") as prepare, patch.object(u, "halt") as halt:
                with self.assertRaisesRegex(RuntimeError, "prepare"):
                    u.move_bpf(FakeMaster(), 2, 230 / 3, 4)
                move.assert_not_called()
                prepare.assert_not_called()
                halt.assert_not_called()

    def test_existing_soft_limit_still_cancels_without_halt(self):
        with patch.object(u, "read_status", return_value=ready_status()), patch.object(u, "dpos") as move, patch.object(u, "halt") as halt:
            self.assertIsNone(u.move_bpf(FakeMaster(), 2, 1000, 4))
            move.assert_not_called()
            halt.assert_not_called()

    def test_no_new_busy_or_frequency_readback_check(self):
        status = ready_status()
        status["motor_on"] = True
        with patch.object(u, "read_status", return_value=status):
            self.assertIs(u.check_bpf_actuator_ready(FakeMaster(), "HPF #5", 4), status)

    def test_halt_failure_does_not_prevent_other_halts_or_hide_primary_error(self):
        original = TimeoutError("original")
        with patch.object(u, "halt", side_effect=[RuntimeError("HALT failed"), None, None]) as halt, patch.object(u, "read_status", side_effect=RuntimeError("read failed")), patch.object(u.time, "sleep"):
            with self.assertRaises(TimeoutError) as caught:
                with u.halt_on_motion_error(FakeMaster(), 2):
                    raise original
            self.assertIs(caught.exception, original)
            self.assertEqual(halt.call_count, 3)

    def test_halt_precedes_close_on_cli_timeout(self):
        events = []
        master = FakeMaster()
        old_close = master.close
        def close():
            events.append("close")
            old_close()
        master.close = close
        with patch.dict(sys.modules, {"utils": u}), patch.object(fake_pysoem, "Master", return_value=master, create=True), patch.object(u.time, "sleep"), patch.object(u, "read_status", return_value=ready_status()), patch.object(u, "dpos", side_effect=TimeoutError("test timeout")), patch.object(u, "halt", side_effect=lambda *a: events.append("halt")):
            result = runner.invoke(cli.app, ["set", "--rx", "6+7", "--lo", "230", "--bw", "4"])
            u.configure_logging("CRITICAL")
        self.assertEqual(events, ["halt", "halt", "halt", "close"])
        self.assertIn("TimeoutError", result.output)
        self.assertNotIn("setting completed", result.output)


if __name__ == "__main__":
    unittest.main()
