import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import sys

ROOT = Path(__file__).resolve().parent.parent

def load(name, path):
  spec=importlib.util.spec_from_file_location(name, ROOT / path)
  module=importlib.util.module_from_spec(spec)
  sys.modules[name]=module
  spec.loader.exec_module(module)
  return module

runner=load('xnor_runner_under_test','openpilot/jetlink_shadow_bench.py')
inst=load('xnor_installer_under_test','tools/install_into_repo.py')
service=load('xnor_supervisor_under_test','openpilot/jetlink_onroad_shadow.py')

class TestLiveGate(unittest.TestCase):
  def sm(self, *, car=True, dev=True, started=True, alive=True):
    return SimpleNamespace(seen={'carState':car,'deviceState':dev},
      alive={'carState':alive,'deviceState':alive},
      __getitem__=None)
  def state(self, dev_started=True, car_seen=True, dev_seen=True, alive=True):
    class SM:
      def __init__(self):
        self.seen={'carState':car_seen, 'deviceState':dev_seen}
        self.alive={'carState':alive, 'deviceState':alive}
      def __getitem__(self, key):
        return SimpleNamespace(started=dev_started)
    return SM()
  def test_live_onroad_allowed(self):
    self.assertEqual(runner.onroad_observation_allowed(self.state()), (True,None))
  def test_offroad_rejected(self):
    self.assertFalse(runner.onroad_observation_allowed(self.state(dev_started=False))[0])
  def test_stale_car_rejected(self):
    self.assertFalse(runner.onroad_observation_allowed(self.state(alive=False))[0])
  def test_unseen_device_rejected(self):
    self.assertFalse(runner.onroad_observation_allowed(self.state(dev_seen=False))[0])
  def test_explicit_opt_in(self):
    args=runner.parse_args(['--mode','camera','--onroad-shadow','--transport','udp','--pipeline-depth','2'])
    self.assertTrue(args.onroad_shadow)
    self.assertFalse(runner.parse_args(['--mode','camera','--transport','udp','--pipeline-depth','2']).onroad_shadow)
  def test_no_nonusb_optin(self):
    for bad in (['--transport','tcp'], ['--allow-other-network'], ['--skip-panda-check']):
      with self.subTest(bad=bad),self.assertRaises(SystemExit):
        runner.parse_args(['--onroad-shadow','--mode','camera','--transport','udp','--pipeline-depth','2']+bad)

class TestPatching(unittest.TestCase):
  STUB='''import os\n\nclass Params: pass\nclass car:\n  CarParams = object\ndef xnor_jetlink_probe(started,params,CP):\n  return False\nprocs = [\n  PythonProcess("xnorjetlinkd", "openpilot.jetlink_xnor_probe", xnor_jetlink_probe),\n  PythonProcess("modeld", "selfdrive.modeld.modeld", only_onroad),\n]\n'''
  def test_patches_once_only(self):
    p=inst.patch_manager(self.STUB)
    self.assertEqual(inst.patch_manager(p),p)
    self.assertEqual(p.count('PythonProcess("xnorjetlinkshadowd"'),1)
    self.assertEqual(p.count('PythonProcess("modeld"'),1)
    self.assertEqual(p.count('PythonProcess("xnorjetlinkd"'),1)
  def test_refuses_unrecognised_manager(self):
    with self.assertRaises(RuntimeError):
      inst.patch_manager('from cereal import car')
  def test_refuses_partial_marker(self):
    with self.assertRaises(RuntimeError):
      inst.patch_manager(self.STUB+inst.PRED_BEGIN)
  def test_no_authority_imports(self):
    text=(ROOT/'openpilot/jetlink_onroad_shadow.py').read_text()
    runner_text=(ROOT/'openpilot/jetlink_shadow_bench.py').read_text()
    for source in [text,runner_text]:
      self.assertNotIn('PubMaster(',source)
      self.assertNotIn('CANPacker(',source)
      self.assertNotIn('params.put(',source)

class TestSupervisor(unittest.TestCase):
  def test_bounded_session(self):
    self.assertEqual(service.RUN_SECONDS,180)
    self.assertLessEqual(service.RUN_SECONDS*service.MAX_RUNS,900)
    self.assertIn('--onroad-shadow',service.runner_args(Path('/tmp/test'), '10.169.232.180'))
    self.assertNotIn('--tcp-quickack',service.runner_args(Path('/tmp/test'),'10.169.232.180'))
    self.assertEqual(service.runner_args(Path('/tmp/test'),'10.169.232.180').count('--mode'),1)

if __name__=='__main__':
  unittest.main()
