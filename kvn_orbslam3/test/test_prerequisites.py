# Copyright (c) 2026, Alessio Morale
# Licensed under the MIT License.

import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from kvn_orbslam3.prerequisites import (
    ENV_PREFIX,
    default_prefix,
    missing_prerequisites,
    warn_if_missing,
)


class TestPrerequisites(unittest.TestCase):
    def test_empty_prefix_reports_both_missing_and_warns_with_script_path(self):
        with tempfile.TemporaryDirectory() as tmp:
            stream = io.StringIO()
            script = '/path/to/scripts/install_prerequisites.sh'
            missing = warn_if_missing(script, prefix=tmp, stream=stream)
            self.assertEqual(
                missing,
                [
                    os.path.join(tmp, 'lib', 'libORB_SLAM3.so'),
                    os.path.join(tmp, 'Vocabulary', 'ORBvoc.txt'),
                ],
            )
            warning = stream.getvalue()
            self.assertIn(script, warning)
            self.assertIn('libORB_SLAM3.so', warning)
            self.assertIn('ORBvoc.txt', warning)

    def test_partial_prefix_reports_only_missing_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            vocab = Path(tmp) / 'Vocabulary' / 'ORBvoc.txt'
            vocab.parent.mkdir(parents=True)
            vocab.write_text('dummy', encoding='utf-8')

            missing = missing_prerequisites(tmp)
            self.assertEqual(missing, [os.path.join(tmp, 'lib', 'libORB_SLAM3.so')])

    def test_complete_prefix_is_quiet(self):
        with tempfile.TemporaryDirectory() as tmp:
            for rel in (('lib', 'libORB_SLAM3.so'), ('Vocabulary', 'ORBvoc.txt')):
                path = Path(tmp).joinpath(*rel)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('ok', encoding='utf-8')

            stream = io.StringIO()
            missing = warn_if_missing('/unused/install_prerequisites.sh', prefix=tmp, stream=stream)
            self.assertEqual(missing, [])
            self.assertEqual(stream.getvalue(), '')

    def test_env_var_overrides_prefix(self):
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch.dict(os.environ, {ENV_PREFIX: tmp}):
                self.assertEqual(default_prefix(), tmp)
                missing = missing_prerequisites()
                self.assertEqual(
                    missing,
                    [
                        os.path.join(tmp, 'lib', 'libORB_SLAM3.so'),
                        os.path.join(tmp, 'Vocabulary', 'ORBvoc.txt'),
                    ],
                )


if __name__ == '__main__':
    unittest.main()
