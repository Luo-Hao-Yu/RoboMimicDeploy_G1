import hashlib
import json
from pathlib import Path
import unittest

import mujoco
import numpy as np
import yaml

from deploy_mujoco.deploy_mujoco_env import (
    DepthHistoryBuffer, _deploy_env_cfg_for_interface, _setup_simulation,
    _load_depth_encoder_actor_onnx, _run_depth_encoder_actor_onnx, _validate_export_interface,
    build_mujoco_sdk_maps, DeployEnv,
)

ROOT = Path(__file__).resolve().parents[1]
POLICY = ROOT / 'config/g1_23dof/parkour'
ASSETS = ROOT / 'g1_description/g1_23dof_parkour'


class G123Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = yaml.safe_load((POLICY / 'params/deploy.yaml').read_text())
        cls.metadata = json.loads((POLICY / 'exported/policy_metadata.json').read_text())
        cls.model = mujoco.MjModel.from_xml_path(str(ASSETS / 'scene.xml'))

    def test_source_identity(self):
        meta = self.metadata
        self.assertEqual(hashlib.sha256((ASSETS / 'g1_23dof.urdf').read_bytes()).hexdigest(), meta['urdf_sha256'])
        verification = json.loads((POLICY / 'exported/verification.json').read_text())
        for name, spec in verification['models'].items():
            self.assertEqual(hashlib.sha256((POLICY / 'exported' / name).read_bytes()).hexdigest(), spec['sha256'])

    def test_named_joint_mapping_and_inertias(self):
        self.assertEqual((self.model.nq, self.model.nv, self.model.nu), (30, 29, 23))
        mapping = build_mujoco_sdk_maps(self.model, self.cfg['sdk_joint_names'])
        for i, policy_name in enumerate(self.metadata['observation_joint_names']):
            sdk = self.cfg['joint_ids_map'][i]
            jid = self.model.joint(policy_name).id
            self.assertEqual(mapping.qposadr[sdk], self.model.jnt_qposadr[jid])
            self.assertEqual(mapping.qveladr[sdk], self.model.jnt_dofadr[jid])
            self.assertEqual(self.model.actuator_trnid[mapping.actuator[sdk], 0], jid)
            np.testing.assert_allclose(self.model.actuator_ctrlrange[mapping.actuator[sdk]],
                                       [-self.metadata['effort_limits_nm'][i], self.metadata['effort_limits_nm'][i]])
        np.testing.assert_allclose(self.model.dof_armature[mapping.qveladr], self.cfg['armature'])

    def test_action_mapping_and_observation_layout(self):
        model, data, env, maps, kp, kd = _setup_simulation(
            _deploy_env_cfg_for_interface(self.cfg, 'depth_encoder_actor'), ASSETS / 'scene.xml', .005, None, np.zeros(3, np.float32))
        env.sync_robot_from_mujoco(data.qpos, data.qvel, maps.qposadr, maps.qveladr)
        obs = env.compute_obs()['obs']
        self.assertEqual(obs.shape, (624,))
        np.testing.assert_allclose(obs[:24], 0)
        np.testing.assert_allclose(obs[24:48].reshape(8,3), np.tile([0,0,-1],(8,1)))
        np.testing.assert_allclose(obs[48:], 0)
        action = np.arange(23, dtype=np.float32) * .01
        env.process_action(action)
        target = env.target_sdk_joint_pos()
        expected = action * np.array(self.metadata['action_scale']) + self.metadata['action_offset_rad']
        np.testing.assert_allclose(target[env.joint_ids_map], expected, atol=1e-7)
        np.testing.assert_allclose(kp[env.joint_ids_map], self.metadata['stiffness_nm_per_rad'])
        np.testing.assert_allclose(kd[env.joint_ids_map], self.metadata['damping_nm_s_per_rad'])

    def test_scene_matches_supplied_parameters(self):
        manifest = json.loads((ASSETS / 'terrain_manifest.json').read_text())
        cfg = manifest['planks']
        self.assertEqual(manifest['ground_z'], -.05)
        self.assertEqual(cfg['num_planks'], 16)
        self.assertIsNone(cfg.get('perlin_cfg'))
        x, y, z = cfg['start']
        for i in range(16):
            geom = self.model.geom(f'wooden_planks_board_{i:02}')
            np.testing.assert_allclose(geom.size, [.3,.45,.025])
            np.testing.assert_allclose(geom.pos, [x+.3+i*.8, y, z-.025])
        self.assertEqual(self.model.geom('floor').pos[2], -.05)
        cam = self.model.camera('depth_cam')
        self.assertEqual(self.model.body(int(self.model.cam_bodyid[cam.id])).name, 'torso_link')
        np.testing.assert_allclose(self.model.cam_fovy[cam.id], 58.29)

    def test_depth_history_old_to_new_stride_and_delay(self):
        history = DepthHistoryBuffer(37, (18,32))
        history.append(np.ones((18,32)))
        np.testing.assert_allclose(history.buf, 1)
        for i in range(1,40):
            history.append(np.full((18,32), i))
        np.testing.assert_allclose(history.delayed_frames(8,5,0)[:,0,0], np.arange(4,40,5))
        np.testing.assert_allclose(history.delayed_frames(8,5,1)[:,0,0], np.arange(3,39,5))

    def test_onnx_contract_and_inference(self):
        encoder, actor = _load_depth_encoder_actor_onnx(POLICY / 'exported', self.cfg['policy_interface'])
        env = DeployEnv(_deploy_env_cfg_for_interface(self.cfg, 'depth_encoder_actor'))
        env.reset()
        env.compute_obs()
        _validate_export_interface(encoder, actor, env, POLICY / 'exported')
        action = _run_depth_encoder_actor_onnx(encoder, actor, np.zeros(624, np.float32), np.zeros((8,18,32), np.float32))
        self.assertEqual(action.shape, (23,))
        self.assertTrue(np.isfinite(action).all())
        env.joint_ids_map[[0,1]] = env.joint_ids_map[[1,0]]
        with self.assertRaisesRegex(ValueError, 'joint order'):
            _validate_export_interface(encoder, actor, env, POLICY / 'exported')
        env.joint_ids_map[[0,1]] = env.joint_ids_map[[1,0]]
        env.obs_terms[0], env.obs_terms[1] = env.obs_terms[1], env.obs_terms[0]
        with self.assertRaisesRegex(ValueError, 'term order'):
            _validate_export_interface(encoder, actor, env, POLICY / 'exported')

    def test_export_rejects_wrong_obs_scales_and_root_frame(self):
        encoder, actor = _load_depth_encoder_actor_onnx(POLICY / 'exported', self.cfg['policy_interface'])
        env = DeployEnv(_deploy_env_cfg_for_interface(self.cfg, 'depth_encoder_actor'))
        env.reset()
        env.compute_obs()
        for index in (0, 1, 2, 3, 4, 5):
            original = env.obs_terms[index].scale.copy()
            env.obs_terms[index].scale *= 2
            with self.assertRaisesRegex(ValueError, 'Observation scale'):
                _validate_export_interface(encoder, actor, env, POLICY / 'exported')
            env.obs_terms[index].scale = original
        env.obs_terms[0].clip = (-1, 1)
        with self.assertRaisesRegex(ValueError, 'observation clipping'):
            _validate_export_interface(encoder, actor, env, POLICY / 'exported')

    def test_default_entry_resolves_local_23dof_config(self):
        from deploy_mujoco.deploy_mujoco import build_parser
        from deploy_mujoco.deploy_mujoco_env import DEPLOY_ROOT, load_deploy_config, resolve_path
        args = build_parser().parse_args([])
        path, cfg = load_deploy_config(args.config, DEPLOY_ROOT)
        self.assertEqual(path, POLICY / 'params/deploy.yaml')
        self.assertEqual(resolve_path(cfg['xml_path'], DEPLOY_ROOT), ASSETS / 'scene.xml')
        self.assertEqual(len(cfg['sdk_joint_names']), 23)


if __name__ == '__main__':
    unittest.main()
