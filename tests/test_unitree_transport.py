import copy
import os
from pathlib import Path
import threading
import unittest
from unittest.mock import Mock, patch
import numpy as np

os.environ.setdefault('CYCLONEDDS_HOME',str(Path(__file__).resolve().parents[1]/'.deps/cyclonedds'))
from deploy_real.unitree_transport import UnitreeTransport
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.utils.crc import CRC

SLOTS=list(range(13))+list(range(15,20))+list(range(22,27))


class TransportTests(unittest.TestCase):
    def test_monitor_does_not_create_publisher_or_release_control(self):
        with patch('unitree_sdk2py.core.channel.ChannelFactoryInitialize') as factory, \
             patch('unitree_sdk2py.core.channel.ChannelSubscriber') as subscriber, \
             patch('unitree_sdk2py.core.channel.ChannelPublisher') as publisher:
            transport=UnitreeTransport('lo',0,SLOTS,enable_output=False)
            factory.assert_called_once_with(0,'lo')
            publisher.assert_not_called()
            with self.assertRaises(RuntimeError):transport.release_control()
            with self.assertRaises(RuntimeError):transport.send(np.zeros(23),np.zeros(23),np.zeros(23),4)
            transport.close()
            subscriber.return_value.Close.assert_called_once()

    def transport(self):
        t=UnitreeTransport.__new__(UnitreeTransport)
        t.motor_indices=SLOTS;t.lock=threading.Lock();t.latest=None;t.last_error=None;t.last_tick=None;t.crc=CRC()
        return t

    def message(self,tick=100):
        m=unitree_hg_msg_dds__LowState_();m.mode_pr=0;m.mode_machine=4;m.tick=tick
        m.imu_state.quaternion=[1,0,0,0]
        for i,slot in enumerate(SLOTS):
            m.motor_state[slot].q=i*.01;m.motor_state[slot].dq=i*.02
            m.motor_state[slot].mode=i % 2;m.motor_state[slot].tau_est=i*.1
        m.crc=CRC().Crc(m)
        return m

    def test_callback_sparse_mapping_crc_and_duplicate_freshness(self):
        t=self.transport();m=self.message();t._callback(m)
        first=t.snapshot()
        np.testing.assert_allclose(first.q,np.arange(23)*.01)
        np.testing.assert_array_equal(first.motor_modes,np.arange(23)%2)
        np.testing.assert_allclose(first.tau_est,np.arange(23)*.1)
        t._callback(m);self.assertEqual(t.snapshot().received_at_ns,first.received_at_ns)
        first.q[:]=999;self.assertNotEqual(t.snapshot().q[0],999)
        m.crc^=1;t._callback(m)
        with self.assertRaisesRegex(RuntimeError,'CRC'):t.snapshot()

    def test_tick_regression_fault_and_wrap(self):
        t=self.transport();t._callback(self.message(100));t._callback(self.message(99))
        with self.assertRaisesRegex(RuntimeError,'backwards'):t.snapshot()
        t=self.transport();t._callback(self.message(0xffffffff));t._callback(self.message(0))
        self.assertEqual(t.snapshot().tick,0)

    def test_command_motor_slot_initialization_validates_indices(self):
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from deploy_real.unitree_transport import fill_motor_command
        for slots in ([0]*23,list(range(22))+[35]):
            with self.assertRaises(ValueError):fill_motor_command(unitree_hg_msg_dds__LowCmd_(),slots,np.zeros(23),np.zeros(23),np.zeros(23),4)


if __name__=='__main__':unittest.main()
