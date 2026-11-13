"""G1 23DoF Parkour real deployment. Default mode subscribes/infer only."""
from __future__ import annotations

import argparse
from collections import deque
import json
from pathlib import Path
import sys
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from deploy_real.real_config import DEFAULT_CONFIG, load_config
from deploy_real.parkour_policy import ParkourPolicy
from deploy_real.robot_state import RobotState
from deploy_real.control import Controller, Phase, ActionSample, JointTrackingError, PostureNotReadyError, validate_state, velocity_command, START, SELECT, F1, A
from deploy_real.depth_source import create_depth_source, preprocess_depth


def checked_depth(source, now_ns, policy, control):
    frame = source.read()
    if frame is None:
        raise ValueError('No depth frame received')
    stamp, image = frame
    now_ns = max(now_ns, time.monotonic_ns())
    age = (now_ns-stamp)*1e-9
    if not 0 <= age <= control['depth_timeout_s']:
        raise ValueError(f'Depth timeout: {age:.3f}s')
    if image.shape != (36, 64):
        raise ValueError('Depth source must produce 36x64 optical-Z metres')
    top,bottom,left,right = policy.depth_cfg['crop_region']
    crop = image[top:36-bottom, left:64-right]
    valid = np.isfinite(crop) & (crop >= policy.depth_cfg['min_distance'])
    if valid.mean() < control['min_depth_valid_fraction']:
        raise ValueError(f'Insufficient valid depth inside policy crop: {valid.mean():.1%}')
    return stamp, image


class InferenceWorker:
    """Never owns DDS publisher; motor watchdog runs independently of inference."""
    def __init__(self, policy, transport, source, cfg):
        self.policy, self.transport, self.source, self.cfg = policy, transport, source, cfg
        self.lock = threading.Lock()
        self.generation = 0
        self.use_remote_command = False
        self.latest = None
        self.error = None
        self.count = 0
        self.metrics = dict(total_inference_s=0.0, max_inference_s=0.0,
                            max_state_age_s=0.0, max_depth_age_s=0.0,
                            max_abs_action=0.0, depth_frame_changes=0)
        self.last_depth_stamp = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, name='parkour-inference', daemon=True)

    def request(self, generation, use_remote_command):
        with self.lock:
            self.generation, self.use_remote_command = generation, use_remote_command

    def snapshot(self):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            return self.latest, self.count

    def statistics(self):
        with self.lock:
            result = dict(self.metrics)
            result['policy_steps'] = self.count
            result['mean_inference_s'] = result.pop('total_inference_s') / max(1, self.count)
            return result

    def run(self):
        generation = None
        deadline = time.monotonic()
        try:
            while not self.stop.is_set():
                with self.lock:
                    requested_generation = self.generation
                    use_remote = self.use_remote_command
                state = self.transport.snapshot()
                validate_state(state, time.monotonic_ns(), self.cfg['control'], self.policy,
                               self.cfg['expected_mode_machine'])
                stamp, image = checked_depth(self.source, time.monotonic_ns(), self.policy, self.cfg['control'])
                if generation != requested_generation:
                    self.policy.reset()
                    generation = requested_generation
                command = velocity_command(state, self.cfg['control']) if use_remote else np.zeros(3,np.float32)
                inference_start = time.monotonic_ns()
                action, target = self.policy.step(state, command, image)
                if np.max(np.abs(action)) > self.cfg['control']['max_abs_action']:
                    raise ValueError('Excessive raw policy action')
                result = ActionSample(target, action, time.monotonic_ns(), state.received_at_ns, stamp, generation,
                                      command=command.copy(), proprio=self.policy.last_proprio,
                                      depth_history=self.policy.last_depth)
                with self.lock:
                    self.latest = result
                    self.count += 1
                    elapsed = (result.created_ns - inference_start) * 1e-9
                    self.metrics['total_inference_s'] += elapsed
                    self.metrics['max_inference_s'] = max(self.metrics['max_inference_s'], elapsed)
                    self.metrics['max_state_age_s'] = max(self.metrics['max_state_age_s'],
                                                         (result.created_ns-state.received_at_ns)*1e-9)
                    self.metrics['max_depth_age_s'] = max(self.metrics['max_depth_age_s'],
                                                         (result.created_ns-stamp)*1e-9)
                    self.metrics['max_abs_action'] = max(self.metrics['max_abs_action'], float(np.abs(action).max()))
                    if stamp != self.last_depth_stamp:
                        self.metrics['depth_frame_changes'] += 1
                        self.last_depth_stamp = stamp
                deadline += self.policy.dt
                remaining = deadline-time.monotonic()
                if remaining < -self.policy.dt:
                    deadline = time.monotonic()
                self.stop.wait(max(0, remaining))
        except Exception as exc:
            with self.lock:
                self.error = f'Inference failed: {exc}'

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)


def self_test(policy, steps):
    state = RobotState(policy.default_q.copy(), np.zeros(23,np.float32),
                       np.array([1,0,0,0],np.float32), np.zeros(3,np.float32),
                       np.zeros(23), np.zeros(23,dtype=np.uint32), 0,np.zeros(4),4,1,time.monotonic_ns())
    policy.reset()
    for _ in range(steps):
        action,target=policy.step(state,np.zeros(3,np.float32),np.ones((36,64),np.float32))
        assert action.shape == target.shape == (23,) and np.isfinite(target).all()
    print(f'Offline self-test PASS: {steps} steps; proprio=624, depth=(8,18,32), action=23')
    print('compact_to_motor_slots=',policy.motor_indices)


def camera_check(policy, cfg, seconds, report=None):
    source = create_depth_source(cfg['camera'])
    stamps=set(); start=time.monotonic(); deadline=start+cfg['control']['startup_timeout_s']
    try:
        while source.read() is None:
            if time.monotonic()>deadline: raise TimeoutError('Camera startup timed out')
            time.sleep(.01)
        start=time.monotonic(); ages=[]; valid=[]
        while time.monotonic()-start<seconds:
            now=time.monotonic_ns()
            stamp,image=checked_depth(source,now,policy,cfg['control'])
            processed=preprocess_depth(image,policy.depth_cfg)
            assert processed.shape==(18,32) and np.isfinite(processed).all()
            stamps.add(stamp); ages.append((time.monotonic_ns()-stamp)*1e-9)
            valid.append(float((np.isfinite(image[18:,16:48]) & (image[18:,16:48]>=.1)).mean()))
            time.sleep(.02)
        result = {'camera_check':'PASS','unique_frames':len(stamps),'max_age_s':max(ages),
                  'min_valid_crop_fraction':min(valid), 'seconds':time.monotonic()-start}
        print(json.dumps(result,indent=2))
        if report:
            path=Path(report);path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(json.dumps(result,indent=2)+'\n')
    finally:
        source.close()


def run(cfg, policy, enable, seconds, report=None, state_only=False):
    from deploy_real.unitree_transport import UnitreeTransport
    control=cfg['control']
    if control['max_forward_m_s']>policy.cfg['commands']['base_velocity']['ranges']['lin_vel_x'][1] or control['max_yaw_rad_s']>policy.cfg['commands']['base_velocity']['ranges']['ang_vel_z'][1]:
        raise ValueError('Real velocity limits exceed policy command limits')
    transport=UnitreeTransport(cfg['network'],cfg['domain_id'],policy.motor_indices,
                               cfg['lowcmd_topic'],cfg['lowstate_topic'],enable_output=enable)
    source=worker=None
    controller=Controller(policy,control)
    owns_control=False
    mode_machine=None
    count=0; start=time.monotonic(); last_print=start
    result={'mode':'enabled' if enable else 'monitor','state_samples':0,'policy_steps':0}
    # Keep the last five seconds at policy frequency in RAM, write only after stopping.
    recent_commands = deque(maxlen=250)
    last_trace_ns = None
    action = None
    try:
        print('ENABLED: Start -> posture ramp, release Start, then A -> policy; Select/F1 stop.' if enable
              else 'MONITOR ONLY: no LowCmd publisher, no control release; Select exits.',flush=True)
        deadline=start+control['startup_timeout_s']
        state=None
        while state is None:
            state=transport.snapshot()
            if time.monotonic()>deadline:raise TimeoutError('No valid LowState received')
            time.sleep(.01)
        mode_machine=state.mode_machine
        print(f'LowState mode_machine={mode_machine}; expected={cfg["expected_mode_machine"]}; tick={state.tick}',flush=True)
        if state_only:
            # Observe current hardware without taking control or requiring the standing policy posture.
            pass
        else:
            validate_state(state,time.monotonic_ns(),control,policy,cfg['expected_mode_machine'])
            source=create_depth_source(cfg['camera'])
            deadline=time.monotonic()+control['startup_timeout_s']
            while source.read() is None:
                if time.monotonic()>deadline:raise TimeoutError('No camera frame received')
                time.sleep(.01)
            worker=InferenceWorker(policy,transport,source,cfg)
            worker.request(0,not enable)
            worker.thread.start()
        start=time.monotonic(); last_print=start
        next_motor=start
        while seconds is None or time.monotonic()-start<seconds:
            state=transport.snapshot()
            now=time.monotonic_ns()
            if state is None:raise ValueError('LowState disappeared')
            if state.buttons & (SELECT|F1):
                print('Remote stop requested',flush=True)
                break
            rising=controller.buttons(state)
            if state_only:
                age=(now-state.received_at_ns)*1e-9
                if age>control['state_timeout_s']:raise ValueError('LowState timeout')
                count+=1
                result.update(mode_machine=state.mode_machine,tick=state.tick,
                              quaternion_wxyz=state.quaternion_wxyz.tolist(),gyro=state.gyro.tolist(),
                              q=state.q.tolist(),dq=state.dq.tolist(),
                              motor_errors=state.motor_errors.tolist(),temperature_c=state.temperature_c.tolist(),
                              motor_modes=state.motor_modes.tolist() if state.motor_modes is not None else None,
                              tau_est_nm=state.tau_est.tolist() if state.tau_est is not None else None)
            else:
                validate_state(state,now,control,policy,cfg['expected_mode_machine'])
                checked_depth(source,now,policy,control)
                action,steps=worker.snapshot()
                # Inference may complete after the loop's earlier timestamp.
                now=time.monotonic_ns()
                result['policy_steps']=steps
                if action is None:
                    if time.monotonic()-start>control['action_timeout_s']:raise ValueError('Policy startup timeout')
                elif (now-action.created_ns)*1e-9>control['action_timeout_s']:
                    raise ValueError('Policy inference watchdog expired')
                if enable:
                    if controller.phase==Phase.WAITING and rising&START and action is not None:
                        if np.any(np.abs(state.sticks)>control['joystick_deadzone']):
                            raise ValueError('Center sticks before Start')
                        transport.release_control()
                        owns_control=True
                        state=transport.snapshot();now=time.monotonic_ns()
                        validate_state(state,now,control,policy,cfg['expected_mode_machine'])
                        if state.buttons & (SELECT|F1):
                            raise ValueError('Remote stop during control handover')
                        controller.begin_ramp(state,now)
                        print('Phase: ramp',flush=True)
                    elif controller.phase==Phase.READY and rising&A:
                        generation=controller.begin_policy(state,now)
                        worker.request(generation,True)
                        print('Phase: policy',flush=True)
                    old_phase=controller.phase
                    output=controller.output(state,now,action)
                    if output is not None:
                        transport.send(*output,mode_machine=mode_machine)
                        if last_trace_ns is None or now-last_trace_ns >= int(policy.dt*1e9):
                            recent_commands.append(dict(
                                monotonic_ns=now, phase=controller.phase.value, tick=int(state.tick),
                                measured_q_rad=state.q.tolist(), measured_dq_rad_s=state.dq.tolist(),
                                sent_target_rad=output[0].tolist(),
                                sent_kp=output[1].tolist(), sent_kd=output[2].tolist(),
                                quaternion_wxyz=state.quaternion_wxyz.tolist(), gyro=state.gyro.tolist(),
                                motor_modes=state.motor_modes.tolist() if state.motor_modes is not None else None,
                                motor_errors=state.motor_errors.tolist(),
                                tau_est_nm=state.tau_est.tolist() if state.tau_est is not None else None,
                                state_age_s=(now-state.received_at_ns)*1e-9,
                                raw_action=action.raw_action.tolist() if action is not None else None,
                                policy_target_rad=action.target.tolist() if action is not None else None,
                                action_generation=action.generation if action is not None else None,
                                controller_generation=controller.generation,
                                command=action.command.tolist() if action is not None and action.command is not None else None,
                            ))
                            last_trace_ns = now
                    if old_phase!=controller.phase:print('Phase:',controller.phase.value,flush=True)
                count+=1
            if time.monotonic()-last_print>=1:
                if enable and controller.phase in (Phase.RAMP, Phase.SETTLING, Phase.READY):
                    ready = controller.readiness(state)
                    print(f"posture: worst={ready['worst_joint']} q_error={ready['max_q_error_rad']:.3f} rad "
                          f"max_dq={ready['max_dq_rad_s']:.3f} gyro={ready['gyro_norm_rad_s']:.3f} rad/s",flush=True)
                print(f'phase={controller.phase.value} tick={state.tick} policy_steps={result["policy_steps"]} state_age_ms={(now-state.received_at_ns)*1e-6:.1f}',flush=True)
                last_print=time.monotonic()
            next_motor+=control['motor_dt']
            delay=next_motor-time.monotonic()
            if delay< -control['motor_dt']:next_motor=time.monotonic()
            time.sleep(max(0,delay))
        result.update(status='PASS',state_samples=count,seconds=time.monotonic()-start)
        if worker is not None:
            result.update(worker.statistics())
            result['policy_hz'] = result['policy_steps']/result['seconds']
        if not state_only and result['policy_steps']==0:raise ValueError('No inference completed')
        print(json.dumps(result,indent=2),flush=True)
    except BaseException as exc:
        result['phase_at_failure'] = controller.phase.value
        controller.fault(exc)
        result.update(status='FAIL',error=str(exc),state_samples=count)
        if isinstance(exc, JointTrackingError):
            result['tracking_error'] = exc.details
        if isinstance(exc, PostureNotReadyError):
            result['posture_not_ready'] = exc.details
        if enable:
            result['recent_motor_commands'] = list(recent_commands)
        if action is not None:
            result['last_inference'] = {
                key: value.tolist() if value is not None else None
                for key, value in dict(command=action.command, proprio=action.proprio,
                                       depth_history=action.depth_history).items()
            }
        result['config'] = cfg
        result['joint_names'] = list(policy.env.sdk_joint_names)
        result['compact_to_motor_slots'] = list(policy.motor_indices)
        if worker is not None:
            result.update(worker.statistics())
        raise
    finally:
        if enable:
            result['recent_motor_commands'] = list(recent_commands)
            result['config'] = cfg
            result['joint_names'] = list(policy.env.sdk_joint_names)
            result['compact_to_motor_slots'] = list(policy.motor_indices)
        # Damping does not depend on camera/ONNX cleanup and runs before those joins.
        if owns_control and mode_machine is not None:
            end=time.monotonic()+control['damping_duration_s']
            while time.monotonic()<end:
                try:
                    # Never write commands for an obsolete hardware mode.
                    with transport.lock:
                        current = transport.latest
                    if current is not None and current.mode_machine != mode_machine:
                        print('Hardware mode changed; LowCmd output stopped',file=sys.stderr)
                        break
                    transport.send_damping(control['damping_kd'],mode_machine)
                except Exception as exc:
                    print(f'Damping publish failed: {exc}',file=sys.stderr);break
                time.sleep(control['motor_dt'])
        for resource in (worker, source, transport):
            if resource is not None:
                try:
                    resource.close()
                except Exception as cleanup_error:
                    result.setdefault('cleanup_errors', []).append(str(cleanup_error))
                    print(f'Cleanup failed: {cleanup_error}',file=sys.stderr)
        if (result.get('status') == 'FAIL' or enable) and not report:
            folder=ROOT/'logs/sim2real'
            try:
                folder.mkdir(parents=True,exist_ok=True)
                prefix = 'real_failure_' if result.get('status') == 'FAIL' else 'real_session_'
                with tempfile.NamedTemporaryFile(mode='w',dir=folder,prefix=prefix,suffix='.json',
                                                 encoding='utf-8',delete=False) as output_file:
                    json.dump(result,output_file,indent=2)
                    output_file.write('\n')
                    print(f'Deployment report saved: {output_file.name}',file=sys.stderr)
            except OSError as save_error:
                print(f'Could not save failure report: {save_error}',file=sys.stderr)
        if report:
            try:
                path=Path(report);path.parent.mkdir(parents=True,exist_ok=True)
                path.write_text(json.dumps(result,indent=2)+'\n')
            except OSError as save_error:
                print(f'Could not save requested report: {save_error}',file=sys.stderr)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--self-test',action='store_true',help='Offline ONNX test, no hardware')
    modes.add_argument('--camera-check',action='store_true',help='Read camera only, no DDS')
    modes.add_argument('--state-only',action='store_true',help='Subscribe LowState only, no camera/control')
    modes.add_argument('--enable',action='store_true',help='Allow Start/A to take motor control')
    parser.add_argument('--network')
    parser.add_argument('--seconds',type=float)
    parser.add_argument('--steps',type=int,default=200)
    parser.add_argument('--report',type=Path)
    args=parser.parse_args()
    if args.steps<1 or (args.seconds is not None and (not np.isfinite(args.seconds) or args.seconds<=0)):
        parser.error('steps and seconds must be positive')
    cfg=load_config(args.config)
    if args.network:cfg['network']=args.network
    policy=ParkourPolicy(cfg['policy_config'])
    if args.self_test:self_test(policy,args.steps)
    elif args.camera_check:camera_check(policy,cfg,args.seconds or 5,args.report)
    else:run(cfg,policy,args.enable,args.seconds,args.report,args.state_only)


if __name__=='__main__':
    main()
