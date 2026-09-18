"""配置与 .env 解析测试：纯逻辑，不需要硬件与网络。"""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from unittest import mock

MODULE_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

import vlm_config


class ParseEnvTextTest(unittest.TestCase):
    """验证 KEY=VALUE 文本解析的各种写法。"""

    def test_basic_pairs(self) -> None:
        values = vlm_config.parse_env_text("A=1\nB=two\n")

        self.assertEqual(values, {"A": "1", "B": "two"})

    def test_comments_and_blank_lines_ignored(self) -> None:
        text = "# 注释\n\n   \nA=1\n  # 缩进注释\n"

        self.assertEqual(vlm_config.parse_env_text(text), {"A": "1"})

    def test_quotes_are_stripped(self) -> None:
        values = vlm_config.parse_env_text('A="有 空格"\nB=\'单引号\'\n')

        self.assertEqual(values, {"A": "有 空格", "B": "单引号"})

    def test_export_prefix_supported(self) -> None:
        self.assertEqual(vlm_config.parse_env_text("export A=1\n"), {"A": "1"})

    def test_value_may_contain_equals_sign(self) -> None:
        self.assertEqual(vlm_config.parse_env_text("URL=https://x/v1?q=1\n"), {"URL": "https://x/v1?q=1"})

    def test_line_without_equals_is_skipped(self) -> None:
        self.assertEqual(vlm_config.parse_env_text("A=1\n没有等号\n"), {"A": "1"})


class LoadEnvTest(unittest.TestCase):
    """验证多文件合并与不存在文件的容错。"""

    def test_missing_files_are_skipped(self) -> None:
        # 不存在的路径只是跳过，不应抛异常（仿真模式就没有 .env）。
        self.assertEqual(vlm_config.load_env((Path("/nonexistent/.env"),)), {})

    def test_later_file_overrides_earlier(self) -> None:
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            first = Path(directory) / "a.env"
            second = Path(directory) / "b.env"
            first.write_text("A=1\nB=1\n", encoding="utf-8")
            second.write_text("B=2\n", encoding="utf-8")

            merged = vlm_config.load_env((first, second))

        self.assertEqual(merged, {"A": "1", "B": "2"})


class ReadSettingTest(unittest.TestCase):
    """验证"环境变量优先，其次 .env"的读取顺序。"""

    def test_env_file_used_when_no_process_env(self) -> None:
        os.environ.pop("VLM_NAV_TEST_KEY", None)

        value = vlm_config.read_setting("VLM_NAV_TEST_KEY", "default", {"VLM_NAV_TEST_KEY": "from-file"})

        self.assertEqual(value, "from-file")

    def test_process_env_takes_priority(self) -> None:
        os.environ["VLM_NAV_TEST_KEY"] = "from-env"
        try:
            value = vlm_config.read_setting("VLM_NAV_TEST_KEY", "default", {"VLM_NAV_TEST_KEY": "from-file"})
        finally:
            del os.environ["VLM_NAV_TEST_KEY"]

        self.assertEqual(value, "from-env")

    def test_empty_value_falls_back_to_default(self) -> None:
        self.assertEqual(vlm_config.read_setting("VLM_NAV_MISSING", "default", {}), "default")

    def test_load_vlm_settings_uses_defaults(self) -> None:
        # 环境变量优先级最高，因此测试时必须先清掉进程里可能存在的真实凭据，
        # 否则会读到开发机 shell 里导出的 key，测试就不再是"空环境"。
        with mock.patch.dict(os.environ, {}, clear=False):
            for name in ("DEEPSEEK_API_KEY", "VLM_API_KEY", "DEEPSEEK_BASE_URL", "DEEPSEEK_MODEL"):
                os.environ.pop(name, None)
            # 空环境时 base_url / model 应回落到内置默认值，api_key 为空。
            settings = vlm_config.load_vlm_settings({})

        self.assertIsNone(settings["api_key"])
        self.assertEqual(settings["base_url"], vlm_config.VlmConfig.base_url)
        self.assertEqual(settings["model"], vlm_config.VlmConfig.model)

    def test_load_vlm_settings_reads_overrides(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=False):
            for name in ("DEEPSEEK_API_KEY", "VLM_API_KEY", "DEEPSEEK_BASE_URL", "DEEPSEEK_MODEL"):
                os.environ.pop(name, None)
            settings = vlm_config.load_vlm_settings(
                {"DEEPSEEK_API_KEY": "k", "DEEPSEEK_BASE_URL": "https://x/v1", "DEEPSEEK_MODEL": "m"}
            )

        self.assertEqual(settings, {"api_key": "k", "base_url": "https://x/v1", "model": "m"})


class ModuleConstantsTest(unittest.TestCase):
    """确认路径常量指向真实目录，避免 sys.path 注入到错误位置。"""

    def test_paths_exist(self) -> None:
        self.assertTrue(vlm_config.MODULE_ROOT.is_dir())
        self.assertTrue(vlm_config.CONTROL_ROOT.is_dir())
        self.assertTrue(vlm_config.MANUAL_CONTROL_ROOT.is_dir())


if __name__ == "__main__":
    unittest.main()
