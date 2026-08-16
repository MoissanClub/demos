import io
import unittest
from contextlib import redirect_stdout
from unittest.mock import mock_open, patch

from handshake.speaker import SpeakerRunner, load_demo_config


class GreetingConfigTests(unittest.TestCase):
    def test_loads_and_trims_greeting(self):
        data = '{"greeting_phrase": "  nice to meet you  ", "speaker_id": 2}'
        with patch("builtins.open", mock_open(read_data=data)):
            self.assertEqual(
                load_demo_config("config.json"),
                ("nice to meet you", "你好", 2),
            )

    def test_loads_invitation_phrase(self):
        data = '{"greeting_phrase": "welcome", "invitation_phrase": "  你好  "}'
        with patch("builtins.open", mock_open(read_data=data)):
            self.assertEqual(load_demo_config("config.json"), ("welcome", "你好", 0))

    def test_rejects_empty_greeting(self):
        data = '{"greeting_phrase": "  "}'
        with patch("builtins.open", mock_open(read_data=data)):
            with self.assertRaisesRegex(ValueError, "greeting_phrase"):
                load_demo_config("config.json")

    def test_rejects_boolean_speaker_id(self):
        data = '{"greeting_phrase": "hello", "speaker_id": true}'
        with patch("builtins.open", mock_open(read_data=data)):
            with self.assertRaisesRegex(ValueError, "speaker_id"):
                load_demo_config("config.json")


class SpeakerRunnerTests(unittest.TestCase):
    def test_dry_run_reports_greeting_without_sdk(self):
        runner = SpeakerRunner("nice to meet you", 0, True, None)
        output = io.StringIO()

        with redirect_stdout(output):
            runner.init(channel_initialized=False)
            runner.say("你好")
            runner.greet()

        self.assertIn("nice to meet you", output.getvalue())
        self.assertIn("你好", output.getvalue())


if __name__ == "__main__":
    unittest.main()
