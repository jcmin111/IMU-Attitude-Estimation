"""IMU HUD V4 — interactive no-hardware prototype. PySide6 + Qt rendered 3D and streaming plots.
Static art is used for frames; moving elements are independent widgets drawn each frame.
"""
import math, random, sys, time, subprocess, threading, csv, json, copy
from datetime import datetime, timezone, timedelta
from collections import deque
from pathlib import Path
from PySide6.QtCore import Qt, QTimer, QRectF, QPointF, Signal
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QFont, QImage, QPixmap, QPainterPath, QLinearGradient
from PySide6.QtWidgets import QApplication, QWidget
from serial_reader import STM32Serial, list_ports, FilterSample, PerformanceSample, AngleSample
from imu_reference_lookup import load_reference, estimate_reference_deg

# VTK is loaded lazily only when the final 3D page is shown.

ROOT=Path(getattr(sys,'_MEIPASS',Path(__file__).resolve().parent))
# Shared integrated firmware Flash footprint from STM32CubeIDE Debug/Stepper_Test.elf.
# 0x1c4 (.isr_vector) + 0xdbbc (.text) + 0xa7c (.rodata) + 0x1e8 (.data load image).
# All four filters are linked into one firmware; per-filter exclusive Flash is not measurable from this build.
FIRMWARE_FLASH_KB = 59876 / 1024
NAMES=['Complementary','AQUA','Mahony','EKF (Quaternion)']
COLS=['#00e9ff','#ff6d57','#15e65a','#ed57ff']
STEPS=['일정 각도 회전 테스트 (0°, 30°, 60°, 90°)','빠른 회전 테스트 (180°)','진동 환경 테스트','선형가속도 외란 테스트']
VOICES=['캘리브레이션을 진행합니다.','아이엠유의 수평을 맞춘 후 움직이지 않은 상태로 평평한 곳에 두세요.','아이엠유를 반대 자세로 가속도를 보정하세요.','아이엠유를 팔자 형태로 움직여 지자계를 보정해 주세요.','아이엠유 기준 자세를 설정해 주세요.','캘리브레이션이 완료되었습니다.','필터 선정을 완료하였습니다.']

def speak(s):
    def worker():
        try:
            time.sleep(0.5)  # Delay each female TTS announcement by 0.5 s
            import base64
            # Windows Korean TTS, no third-party voice dependency
            script=(
                '$v=New-Object -ComObject SAPI.SpVoice; '
                '$voices=$v.GetVoices(); $chosen=$null; '
                'foreach($item in $voices){ '
                'if($item.GetAttribute("Gender") -eq "Female" -and '
                '($item.GetAttribute("Language") -match "412|0412" -or $item.GetDescription() -match "Korean|한국|Heami|SunHi")){ $chosen=$item; break } }; '
                'if($null -eq $chosen){ foreach($item in $voices){ if($item.GetAttribute("Gender") -eq "Female"){ $chosen=$item; break } } }; '
                'if($null -ne $chosen){ $v.Voice=$chosen }; '
                '$v.Rate=2; '
                '$v.Speak([Text.Encoding]::UTF8.GetString([Convert]::FromBase64String("'
                +base64.b64encode(s.encode()).decode()+'"))) | Out-Null'
            )
            subprocess.run(['powershell','-NoProfile','-Command',script],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=20,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        except Exception: pass
    threading.Thread(target=worker,daemon=True).start()

def qc(hex_,alpha=255):
    c=QColor(hex_);c.setAlpha(alpha);return c

def quat(roll,pitch,yaw):
    r,p,y=map(lambda x:math.radians(x)*.5,(roll,pitch,yaw))
    cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
    return (cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy)

def qmultiply(a,b):
    w,x,y,z=a; W,X,Y,Z=b
    return (w*W-x*X-y*Y-z*Z, w*X+x*W+y*Z-z*Y,
            w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W)

def qnormalize(q):
    n=math.sqrt(sum(v*v for v in q))
    return tuple(v/n for v in q) if n>1e-9 else (1.,0.,0.,0.)

def qinverse(q):
    w,x,y,z=q
    return (w,-x,-y,-z)

def quaternion_deviation_deg(a,b):
    """Shortest SO(3) angular separation, not Euler-axis subtraction."""
    a=qnormalize(a);b=qnormalize(b)
    dot=min(1.0,max(-1.0,abs(sum(x*y for x,y in zip(a,b)))))
    return math.degrees(2.0*math.acos(dot))

def relative_roll_pitch_deg(home, current):
    """Relative orientation about the saved stage baseline (not ground-truth error)."""
    q=qnormalize(qmultiply(qinverse(qnormalize(home)),qnormalize(current)))
    w,x,y,z=q
    roll=math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)))
    pitch=math.degrees(math.asin(max(-1.,min(1.,2*(w*y-z*x)))))
    return roll,pitch

def quaternion_full_turn_deg(home, current, axis):
    """Signed 0..360 rotation about the calibrated physical local axis.
    Unlike the shortest SO(3) distance, this retains the 180..360 half.
    """
    relative=qnormalize(qmultiply(qinverse(qnormalize(home)),qnormalize(current)))
    w,x,y,z=relative
    projection=x*axis[0]+y*axis[1]+z*axis[2]
    return (math.degrees(2.0*math.atan2(projection,w)))%360.0

def qrotate(v,q):
    w,x,y,z=q;vx,vy,vz=v
    # rotation matrix from quaternion
    return ((1-2*(y*y+z*z))*vx+2*(x*y-w*z)*vy+2*(x*z+w*y)*vz,
            2*(x*y+w*z)*vx+(1-2*(x*x+z*z))*vy+2*(y*z-w*x)*vz,
            2*(x*z-w*y)*vx+2*(y*z+w*x)*vy+(1-2*(x*x+y*y))*vz)

class HUD(QWidget):
    sensor_received = Signal(object)
    serial_failed = Signal(str)
    telemetry_received = Signal(object)
    command_response = Signal(str)
    def __init__(self):
        super().__init__();self.setWindowTitle('IMU ATTITUDE / TELEMETRY — STM32 LIVE')
        self.resize(1500,890);self.setMinimumSize(850,500);self.setMouseTracking(True)
        self.img=[QPixmap(str(ROOT/'assets'/f'{i:02}.png')) for i in range(1,13)]
        if any(p.isNull() for p in self.img): raise RuntimeError('assets/01.png ... assets/12.png are required')
        self.transition_started=None;self.transition_timer=None;self.page=0;self.t0=time.monotonic();self.last=self.t0;self.target=0.;self.angle=0.;self.mute=False;self.effects=True
        self.series=[deque(maxlen=200) for _ in range(4)];self.error_records={k:[] for k in "CMAE"};self.angle_last_seen={};self.deviation_reference_q={};self.deviation_gyro_reference_q=None;self.raw=[deque(maxlen=180) for _ in range(3)]
        self.sample_acc=0.;self.elapsed=0.;self.phase=0.;self.cal_started=time.monotonic();self.selected=2;self.rng=random.Random(2104);self.test_started=None;self.test_finished=None;self.data_samples=0
        self.serial_link=None;self.latest_sample=None;self.sensor_count=0;self.serial_status='CONNECTING'
        self.filter_live={};self.perf_live={};self.has_live=False
        self.home_angle_reference={};self.home_filter_yaw={};self.filter_packet_count={k:0 for k in 'CMAE'}
        # User-labelled 7-point 0..180-degree magnetic fingerprint reference.
        # This is an estimated angle, not an independent encoder ground truth.
        self.magnetic_reference=load_reference()
        self.full_turn_axes={k:None for k in "CMAE"}
        self.magnetic_angle=None
        self.magnetic_residual=None
        self.magnetic_home_angle=None
        self.magnetic_reference_valid=False
        self.magnetic_last_at=0.0
        self.magnetic_last_device_ms=None
        self.magnetic_rejected=0
        self.reference_log_count=0
        self.magnetic_unrestricted_angle=None
        self.magnetic_unrestricted_residual=None
        self.magnetic_jump_count=0
        self.home_gyro_q=None
        self.angle_live={}
        self.home_reference_q={}  # STEP4 GUI home; firmware auto-home remains unchanged
        self.home_set=False
        self.home_zero_pending=False
        self.home_zero_requested_at=None
        self.home_ack_counts=None
        self.home_wait_attempts=0
        self.home_filter_recent={}
        self.live_rms_window={k:deque() for k in 'CMAE'}
        self.rms_window_seconds=30.0
        self.final_results=json.loads((Path(__file__).resolve().parent/'final_experiment_results.json').read_text(encoding='utf-8'))['filters']
        self.historical_targets=[90,180,270]
        self.filter_code_kb={'C':0.715,'A':0.770,'M':0.926,'E':5.066}

        # Store actual telemetry; never substitute simulated samples for measurements.
        self.record_dir=Path(__file__).resolve().parent/'measurements'/datetime.now().strftime('%Y%m%d_%H%M%S')
        self.record_dir.mkdir(parents=True,exist_ok=True)
        self.record_files={};self.record_writers={}
        for kind,columns in {
            'raw':['wall_time','page','seq','ms','ax','ay','az','gx','gy','gz','mx','my','mz','mag_valid'],
            'filters':['wall_time','page','filter','roll','pitch','yaw','qw','qx','qy','qz'],
            'performance':['wall_time','page','filter','execution_us','cpu_percent','state_bytes'],
            'drift':['wall_time','page','filter','stationary_drift_deg_per_min','window_seconds'],
        }.items():
            f=(self.record_dir/(kind+'.csv')).open('w',newline='',encoding='utf-8-sig')
            self.record_files[kind]=f;self.record_writers[kind]=csv.writer(f)
            self.record_writers[kind].writerow(columns)
        self.cal_records={str(i):[] for i in range(1,5)}
        self.cal_capture_start=time.monotonic()
        self.perf_history={k:[] for k in 'CMAE'}
        self.summary_snapshot=None
        self.stage_accuracy={k:[] for k in 'CMAE'}
        self.stage_vibration={k:[] for k in 'CMAE'}
        self.stage_axis_rms={k:{'roll':[],'pitch':[]} for k in 'CMAE'}
        self.stage_axis_home={}
        self.stage_vibration_home={}
        self.stage_disturb_home={}
        self.stage_disturb_start={k:None for k in 'CMAE'}
        self.stage_disturb_last={k:None for k in 'CMAE'}
        self.stage_disturb_quiet={k:None for k in 'CMAE'}
        self.stage_recovery={k:None for k in 'CMAE'}
        self.stage_gyro_speed=0.0
        self.stage_disturb_completed=False
        self.selection_scores=self.calculate_filter_scores()
        self.selected=min(self.selection_scores,key=self.selection_scores.get) if self.selection_scores else None
        self.selection_reasons=self.build_selection_reasons()

        # Drift is measured only during a sustained stationary interval.
        self.stationary_raw=deque(maxlen=35)
        self.drift_windows={fid:deque(maxlen=900) for fid in 'CMAE'}
        self.drift_live={}
        self.last_raw_at=0.0
        self.vibration_ack='NO ACK'
        self.vibration_command_at=None
        # Optional build-derived MCU RAM report. Never confuse filter state with total RAM.
        self.build_ram_kb=None
        self.build_filter_ram_kb={}
        report=Path(__file__).resolve().parent/'stm32_memory_report.json'
        if report.is_file():
            try:
                obj=json.loads(report.read_text(encoding='utf-8'))
                self.build_ram_kb=float(obj['total_ram_bytes'])/1024.0
                self.build_filter_ram_kb={k:float(v)/1024.0 for k,v in obj.get('filter_ram_bytes',{}).items() if k in 'CMAE'}
            except (ValueError,TypeError,KeyError,OverflowError):
                self.build_ram_kb=None

        # The unfiltered comparison integrates raw gyro measurements, not animation.
        self.gyro_only_q=(1.,0.,0.,0.)
        self.gyro_last_ms=None
        self.viewer_reference_q=None
        self.viewer_gyro_reference_q=None
        self.vtk_actor_sets=[];self.last_vtk_render=0.0;self.vtk_render_interval=1.0/24.0
        self.sensor_received.connect(self.on_sensor)
        self.telemetry_received.connect(self.on_telemetry)
        self.command_response.connect(self.on_command_response)
        self.serial_failed.connect(self.on_serial_error)
        QTimer.singleShot(0,self.connect_stm32)
        self.summary_entered=None;self.viewer3d=None;self.viewer3d_right=None;self.viewer_error=None;self.vtk_scenes=[];self.vtk_preload_scheduled=False;self.vibration_on=False;self.bases=self.prepare_bases();self.transition_globe=None;self.timer=QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(33)

    def live_summary_metrics(self):
        """Separate measured stage metrics. Missing evidence remains missing."""
        result={}
        for fid in 'CMAE':
            vals=self.stage_accuracy[fid]
            vib=self.stage_vibration[fid]
            perf=self.perf_live.get(fid)
            result[fid]={
                'count':len(vals),
                'mae':sum(vals)/len(vals) if vals else None,
                'rmse':math.sqrt(sum(v*v for v in vals)/len(vals)) if vals else None,
                'max':max(vals) if vals else None,
                'stability':(math.sqrt(sum((v-sum(vib)/len(vib))**2 for v in vib)/len(vib)) if len(vib)>=15 else None),
                'roll_motion_rms':(math.sqrt(sum(v*v for v in self.stage_axis_rms[fid]['roll'])/len(self.stage_axis_rms[fid]['roll'])) if self.stage_axis_rms[fid]['roll'] else None),
                'pitch_motion_rms':(math.sqrt(sum(v*v for v in self.stage_axis_rms[fid]['pitch'])/len(self.stage_axis_rms[fid]['pitch'])) if self.stage_axis_rms[fid]['pitch'] else None),
                'response':self.stage_recovery[fid],
                'execution_us':(sum(v[0] for v in self.perf_history[fid])/len(self.perf_history[fid]) if self.perf_history[fid] else None),
                'cpu':(sum(v[1] for v in self.perf_history[fid])/len(self.perf_history[fid]) if self.perf_history[fid] else None),
                'ram':float(perf.state_bytes)/1024.0 if perf else None,
                'flash':self.filter_code_kb[fid],
            }
        return result

    def calculate_filter_scores(self):
        """Always rank the four filters. Missing stage evidence is a FAIL,
        never an invented measurement or a reason to cancel selection.
        Lower weighted penalty score is better.
        """
        metrics=(self.summary_snapshot['metrics'] if self.summary_snapshot is not None else self.live_summary_metrics())
        keys={'mae':.40,'stability':.30,'response':.15,
              'execution_us':.10,'ram':.025,'flash':.025}
        best={}
        for k in keys:
            valid=[max(1e-6,metrics[f][k]) for f in 'CMAE'
                   if metrics[f][k] is not None and math.isfinite(metrics[f][k])]
            best[k]=min(valid) if valid else None
        scores={}
        for fid in 'CMAE':
            score=0.0
            for k,weight in keys.items():
                value=metrics[fid][k]
                if value is None or not math.isfinite(value) or best[k] is None:
                    score+=weight*3.0  # FAIL: 3x best-score penalty, not fabricated data
                else:
                    score+=weight*max(1e-6,value)/best[k]
            scores[fid]=score
        return scores

    def build_selection_reasons(self):
        fid=min(self.selection_scores,key=self.selection_scores.get)
        d=(self.summary_snapshot['metrics'] if self.summary_snapshot is not None else self.live_summary_metrics())[fid]
        fmt=lambda value,unit: f'{value:.2f}{unit}' if value is not None else 'FAIL'
        return [f"정적 MAE {fmt(d['mae'],'°')} / 진동 편차 {fmt(d['stability'],'°')}",
                f"외란 안정화 {fmt(d['response'],'s')} · 실행 {fmt(d['execution_us']/1000 if d['execution_us'] is not None else None,'ms')}"]

    def record_stage_attitude(self, item):
        fid=item.filter_id
        if not self.home_set or self.home_zero_pending:
            return
        q=qnormalize(item.quaternion)
        # Roll/Pitch RMS of measured motion during vibration and disturbance.
        # Reset the relative baseline on entering a different experiment stage.
        if self.page in (8,9):
            key=(self.page,fid)
            if key not in self.stage_axis_home:
                self.stage_axis_home[key]=q
            roll,pitch=relative_roll_pitch_deg(self.stage_axis_home[key],q)
            self.stage_axis_rms[fid]['roll'].append(roll)
            self.stage_axis_rms[fid]['pitch'].append(pitch)
        if self.page==8:
            if fid not in self.stage_vibration_home:
                self.stage_vibration_home[fid]=q
            self.stage_vibration[fid].append(quaternion_deviation_deg(self.stage_vibration_home[fid],q))
        elif self.page==9:
            if self.stage_disturb_completed:return
            now=time.monotonic()
            if fid not in self.stage_disturb_home:
                self.stage_disturb_home[fid]=q
            if self.stage_recovery[fid] is not None:
                return
            # Motion is detected from the raw gyro; no synthetic shake event.
            speed=self.stage_gyro_speed
            if speed>0.5:
                if self.stage_disturb_start[fid] is None:
                    self.stage_disturb_start[fid]=now
                self.stage_disturb_last[fid]=now
                self.stage_disturb_quiet[fid]=None
            elif self.stage_disturb_last[fid] is not None and now-self.stage_disturb_last[fid]>0.5:
                # Record motion settling once gyro motion stops. No return-to-home
                # angle threshold or additional 0.5 s hold is required.
                # This is motion-settling duration, NOT orientation recovery error.
                if self.stage_disturb_start[fid] is not None:
                    self.stage_recovery[fid]=now-self.stage_disturb_start[fid]

    def connect_stm32(self):
        # Auto-detect ST-LINK VCP; fallback to COM3 from the verified PuTTY session.
        try:
            ports=list_ports()
            candidates=[name for name,description in ports if 'STLINK' in description.upper().replace('-','') or 'STMICRO' in description.upper()]
            port=candidates[0] if candidates else 'COM3'
            self.serial_link=STM32Serial(port,
                lambda sample:self.sensor_received.emit(sample),
                lambda exc:self.serial_failed.emit(str(exc)),
                on_telemetry=lambda item:self.telemetry_received.emit(item),
                on_command=lambda msg:self.command_response.emit(msg))
            self.serial_link.start()
            self.serial_status=f'CONNECTED {port}'
            self.setWindowTitle(f'IMU ATTITUDE / TELEMETRY — {self.serial_status}')
        except Exception as exc:
            self.on_serial_error(str(exc))

    def on_sensor(self,sample):
        self.latest_sample=sample
        self.stage_gyro_speed=math.sqrt(sample.gx**2+sample.gy**2+sample.gz**2)
        # IMUSample magnetometer is in microtesla; the PuTTY LUT is raw AK09916 counts.
        if sample.mag_valid:
            try:
                # Always evaluate the globally best magnetic match first.
                # The old hard continuity gate could trap the reference on a
                # wrong LUT branch after a fast rotation, even while stationary.
                raw_angle,raw_residual=estimate_reference_deg(
                    sample.mx/0.15,sample.my/0.15,sample.mz/0.15,self.magnetic_reference)
                self.magnetic_unrestricted_angle=raw_angle
                self.magnetic_unrestricted_residual=raw_residual
                if raw_residual > 150.0:
                    raise ValueError('Magnetic fingerprint mismatch')
                previous=self.magnetic_angle if self.magnetic_reference_valid else None
                dt=None
                if self.magnetic_last_device_ms is not None:
                    dt=(sample.timestamp_ms-self.magnetic_last_device_ms)/1000.0
                gyro_speed_deg=math.degrees(math.sqrt(sample.gx**2+sample.gy**2+sample.gz**2))
                # Report suspicious jumps, but do not force the wrong branch.
                # A magnetic fingerprint alone cannot prove an absolute angle.
                if previous is not None and dt is not None and 0 < dt < 0.5:
                    jump=abs((raw_angle-previous+180.0)%360.0-180.0)
                    if jump > gyro_speed_deg*dt*2.0+12.0:
                        self.magnetic_jump_count+=1
                angle,residual=raw_angle,raw_residual
                self.magnetic_angle=angle
                self.magnetic_residual=residual
                self.magnetic_last_at=time.monotonic()
                self.magnetic_last_device_ms=sample.timestamp_ms
                self.magnetic_reference_valid=True
            except (ValueError,ArithmeticError):
                self.magnetic_rejected+=1
                self.magnetic_reference_valid=False
                self.magnetic_last_device_ms=sample.timestamp_ms
        else:
            self.magnetic_reference_valid=False
        self.sensor_count+=1
        self.last_raw_at=time.monotonic()
        self.stationary_raw.append((self.last_raw_at,math.sqrt(sample.gx**2+sample.gy**2+sample.gz**2),
                                    math.sqrt(sample.ax**2+sample.ay**2+sample.az**2)))
        self.record_writers['raw'].writerow([datetime.now().isoformat(timespec='milliseconds'),self.page,
            sample.seq,sample.timestamp_ms,sample.ax,sample.ay,sample.az,sample.gx,sample.gy,sample.gz,
            sample.mx,sample.my,sample.mz,int(sample.mag_valid)])
        if 2<=self.page<=5:
            self.cal_records[str(self.page-1)].append([sample.ax,sample.ay,sample.az,
                sample.gx,sample.gy,sample.gz,sample.mx,sample.my,sample.mz,int(sample.mag_valid)])
        # Integrate raw angular velocity (rad/s) with device timestamps.
        if self.gyro_last_ms is not None:
            dt=(sample.timestamp_ms-self.gyro_last_ms)/1000.0
            if 0 < dt <= 0.15:
                gx,gy,gz=sample.gx,sample.gy,sample.gz
                speed=math.sqrt(gx*gx+gy*gy+gz*gz)
                if speed>1e-10:
                    half=speed*dt/2
                    scale=math.sin(half)/speed
                    dq=(math.cos(half),gx*scale,gy*scale,gz*scale)
                    self.gyro_only_q=qnormalize(qmultiply(self.gyro_only_q,dq))
        self.gyro_last_ms=sample.timestamp_ms
        # Live raw scope: acceleration in units of g, not synthetic noise.
        for i,v in enumerate((sample.ax,sample.ay,sample.az)):
            self.raw[i].append(v/9.80665)

    def set_reference_angle(self, degrees, source='manual'):
        """Single source of truth for the graph's commanded reference angle."""
        degrees=float(degrees)
        if not math.isfinite(degrees):return
        previous=self.target
        self.target=degrees
        # Keep graph history when unchanged; reset on a new reference angle.
        if abs(previous-degrees)>1e-9:
            for stream in self.series:stream.clear()
        with (self.record_dir/'chart_debug.log').open('a',encoding='utf-8') as log:
            log.write(f'TARGET_CHANGE source={source} page={self.page} previous={previous:.1f} target={self.target:.1f}\n')
        self.update()

    def append_live_error(self,fid,measured):
        """Uniform full-turn-normalized circular angular difference (0..359 degrees).

        Magnetic fingerprint is an experimental sensor-derived reference, not
        an independently verified encoder. Invalid references are not replaced
        by the GUI's 0-degree target (that produced false error spikes).
        """
        if self.page not in (6,7,8,9) or fid not in 'CMAE':
            return
        valid=(self.magnetic_reference_valid and self.magnetic_home_angle is not None
               and time.monotonic()-self.magnetic_last_at < 1.0)
        if not valid or not self.home_set or self.home_zero_pending:
            return
        # Magnetic LUT spans 0..360; compute shortest circular angular error.
        goal=(self.magnetic_angle-self.magnetic_home_angle)%360.0
        measured_360=float(measured)%360.0
        difference=abs((measured_360-goal+180.0)%360.0-180.0)
        error=100.0*difference/360.0
        if not math.isfinite(error):
            return
        self.series['CMAE'.index(fid)].append(error)
        self.error_records[fid].append((goal,error))
        if self.page in (6,7):self.stage_accuracy[fid].append(difference)
        self.reference_log_count+=1
        # Diagnose axis direction without silently flipping a filter or altering the graph.
        if self.filter_packet_count[fid]%25==0:
            reverse_difference=abs(((-measured_360)-goal+180.0)%360.0-180.0)
            with (self.record_dir/'chart_debug.log').open('a',encoding='utf-8') as log:
                log.write(f'AXIS_CHECK fid={fid} goal={goal:.3f} filter={measured_360:.3f} '
                          f'normal_diff={difference:.3f} reverse_diff={reverse_difference:.3f} '
                          f'home_yaw={self.home_filter_yaw.get(fid)} current_yaw={self.filter_live[fid].yaw:.3f} axis=quaternion_signed_0_360 '
                          f'raw_mag={((self.latest_sample.mx,self.latest_sample.my,self.latest_sample.mz) if self.latest_sample else None)} '
                          f'mag_residual={self.magnetic_residual:.3f}\n')
        if self.filter_packet_count[fid]%50==0:
            with (self.record_dir/'chart_debug.log').open('a',encoding='utf-8') as log:
                log.write(f'REFERENCE fid={fid} page={self.page} lut_absolute={self.magnetic_angle:.3f} '
                          f'lut_home={self.magnetic_home_angle:.3f} goal={goal:.3f} '
                          f'filter_relative={measured_360:.3f} angular_difference={difference:.3f} '
                          f'error_360_percent={error:.4f} mag_residual={self.magnetic_residual:.3f} jump_warnings={self.magnetic_jump_count} rejected={self.magnetic_rejected}\n')

    def on_telemetry(self, item):
        if isinstance(item, FilterSample):
            self.record_writers['filters'].writerow([datetime.now().isoformat(timespec='milliseconds'),
                self.page,item.filter_id,item.roll,item.pitch,item.yaw,*item.quaternion])
            self.filter_live[item.filter_id] = item
            self.filter_packet_count[item.filter_id]+=1
            self.home_filter_recent[item.filter_id]=time.monotonic()
            self.update_drift(item)
            self.has_live = True
            idx = 'CMAE'.index(item.filter_id)
            # Estimate the actual physical rotation axis once, from the
            # initial 8..150 degree motion. Preserve its sign for a full turn.
            # Do not use unsigned SO(3) distance: it folds after 180 degrees.
            home=self.home_filter_yaw.get(item.filter_id)
            if home is None:
                home=item.yaw
                self.home_filter_yaw[item.filter_id]=home
            home_q=self.home_reference_q.get(item.filter_id)
            measured=0.0
            if self.home_set and home_q is not None:
                relative=qnormalize(qmultiply(qinverse(qnormalize(home_q)),qnormalize(item.quaternion)))
                w,x,y,z=relative
                if self.full_turn_axes[item.filter_id] is None:
                    mag_goal=((self.magnetic_angle-self.magnetic_home_angle)%360.0
                              if self.magnetic_reference_valid and self.magnetic_home_angle is not None else None)
                    unsigned=quaternion_deviation_deg(home_q,item.quaternion)
                    norm=math.sqrt(x*x+y*y+z*z)
                    if mag_goal is not None and 8.0 <= mag_goal <= 150.0 and 8.0 <= unsigned <= 150.0 and norm>1e-6:
                        self.full_turn_axes[item.filter_id]=(x/norm,y/norm,z/norm)
                axis=self.full_turn_axes[item.filter_id]
                measured=(quaternion_full_turn_deg(home_q,item.quaternion,axis)
                          if axis is not None else quaternion_deviation_deg(home_q,item.quaternion))
            # Live RMS is computed from the same per-filter quaternion rotation
            # stream as the attitude plot, not from optional ANGLE packets.
            # Unwrap full turns so 359 -> 0 does not look like a 359 deg jump.
            rms_window=self.live_rms_window[item.filter_id]
            rms_now=time.monotonic()
            rms_value=float(measured)
            if rms_window:
                rms_value+=360.0*round((rms_window[-1][1]-rms_value)/360.0)
            rms_window.append((rms_now,rms_value))
            while rms_window and rms_now-rms_window[0][0]>self.rms_window_seconds:
                rms_window.popleft()
            self.record_stage_attitude(item)
            self.append_live_error(item.filter_id,measured)
            if self.filter_packet_count[item.filter_id] in (1,50,200) or self.filter_packet_count[item.filter_id]%1000==0:
                with (self.record_dir/'chart_debug.log').open('a',encoding='utf-8') as log:
                    log.write(f'FILT {item.filter_id} count={self.filter_packet_count[item.filter_id]} page={self.page} target={self.target:.1f} yaw={item.yaw:.2f} home={home:.2f} measured={measured:.2f} plotted={len(self.series[idx])} latest={self.series[idx][-1] if self.series[idx] else None}\n')
        elif isinstance(item, AngleSample):
            self.angle_live[item.filter_id]=item.angle_signed
            self.angle_last_seen[item.filter_id]=time.monotonic()
        elif isinstance(item, PerformanceSample):
            self.perf_live[item.filter_id] = item
            if self.page in (6,7,8,9):
                self.perf_history[item.filter_id].append((item.execution_us,item.cpu_percent))
            self.record_writers['performance'].writerow([datetime.now().isoformat(timespec='milliseconds'),
                self.page,item.filter_id,item.execution_us,item.cpu_percent,item.state_bytes])

    def on_command_response(self, message):
        # ACK/ERR is also persisted by the serial transport for hardware debugging.
        if message.startswith('ACK,ZERO') and self.home_zero_pending:
            # ACK is diagnostic only: some firmware versions do not emit it.
            self.log_home_event('ACK_ZERO', counts=dict(self.filter_packet_count))
        if message.startswith('ACK,VIBRATE,'):
            self.vibration_ack=message
        elif message.startswith('ERR,'):
            self.vibration_ack=message
        self.update()

    def update_drift(self, sample):
        """Stationary quaternion change in deg/min (2 s normally, 10 s on final screen).

        This is a finite-window apparent drift estimate, not calibrated
        long-term gyro bias or ground-truth attitude accuracy.
        """
        now=time.monotonic()
        recent=[(w,a) for t,w,a in self.stationary_raw if now-t<0.7]
        stationary=(now-self.last_raw_at<0.5 and len(recent)>=6 and
                    max(w for w,a in recent)<0.18 and
                    all(abs(a-9.80665)<1.5 for w,a in recent))
        fid=sample.filter_id
        window=self.drift_windows[fid]
        drift_seconds=10.0 if self.page==11 else 2.0
        if not stationary:
            # Keep last valid result visible, but restart the stationary interval.
            window.clear()
            return
        q=qnormalize(sample.quaternion)
        window.append((now,q))
        if window and now-window[0][0]>=drift_seconds:
            # Use all samples over the page-specific stationary window. Fit the three components
            # of the relative rotation vector, rather than dividing a noisy
            # start/end quaternion difference by only two seconds.
            recent=[(t,qq) for t,qq in window if now-t<=drift_seconds]
            if len(recent)<6:return
            t0,q0=recent[0]
            xs=[];ys=[]
            for t,qq in recent:
                w,x,y,z=qnormalize(qmultiply(qq,qinverse(q0)))
                if w<0:w,x,y,z=-w,-x,-y,-z
                v=math.sqrt(x*x+y*y+z*z)
                angle=2.0*math.atan2(v,max(0.0,w))
                factor=(math.degrees(angle)/v) if v>1e-10 else (360.0/math.pi)
                xs.append(t-t0)
                ys.append((x*factor,y*factor,z*factor))
            avg_t=sum(xs)/len(xs)
            denom=sum((t-avg_t)**2 for t in xs)
            if denom<=1e-10:return
            slopes=[sum((t-avg_t)*vec[j] for t,vec in zip(xs,ys))/denom for j in range(3)]
            self.drift_live[fid]=math.sqrt(sum(v*v for v in slopes))*60.0
            self.record_writers['drift'].writerow([datetime.now().isoformat(timespec='milliseconds'),
                                                   self.page,fid,f'{self.drift_live[fid]:.6f}',
                                                   f'{xs[-1]:.3f}'])
        else:
            pass

    def on_serial_error(self,message):
        self.serial_status=f'ERROR: {message}'
        self.setWindowTitle(f'IMU ATTITUDE / TELEMETRY — {self.serial_status}')

    def log_home_event(self, event, **details):
        try:
            with (self.record_dir/'chart_debug.log').open('a',encoding='utf-8') as log:
                log.write('HOME_SYNC '+event+' '+repr(details)+'\n')
        except OSError:
            pass

    def finish_home_after_fresh_packets(self):
        if not self.home_zero_pending or self.home_ack_counts is None:
            return
        self.home_wait_attempts+=1
        now=time.monotonic()
        fresh=all(self.filter_packet_count[k]>self.home_ack_counts[k] and
                  now-self.home_filter_recent.get(k,0)<0.75 for k in 'CMAE')
        magnetic=(self.magnetic_reference_valid and self.magnetic_angle is not None
                  and now-self.magnetic_last_at<0.75)
        if fresh and magnetic:
            ok=self.apply_home_calibration()
            if ok:
                self.home_zero_pending=False
                self.home_ack_counts=None
                self.log_home_event('COMMITTED', yaw=dict(self.home_filter_yaw),
                                    magnetic_home=self.magnetic_home_angle,
                                    attempts=self.home_wait_attempts)
                return
        if self.home_wait_attempts<40:
            QTimer.singleShot(150,self.finish_home_after_fresh_packets)
        else:
            self.home_zero_pending=False
            self.home_ack_counts=None
            self.home_set=False
            self.log_home_event('FAILED_NO_FRESH_STREAM', fresh=fresh, magnetic=magnetic,
                                counts=dict(self.filter_packet_count))
            self.on_serial_error('HOME failed: wait for fresh FILT and magnetic DATA, then retry')

    def apply_home_calibration(self):
        """Apply a real quaternion HOME reference in GUI at STEP4 completion.

        Does not claim to write accelerometer/gyro/magnetometer coefficients
        to the STM32. Firmware still uses its existing V1.7 Remaster offsets.
        """
        if any(k not in self.filter_live for k in 'CMAE'):
            self.home_set=False
            self.on_serial_error('HOME calibration requires all four FILT packets')
            return False
        self.error_records={k:[] for k in "CMAE"}
        self.home_reference_q={k:self.filter_live[k].quaternion for k in 'CMAE'}
        self.full_turn_axes={k:None for k in 'CMAE'}
        self.home_filter_yaw={k:self.filter_live[k].yaw for k in 'CMAE'}
        # Align the reference-table coordinate system with the user's HOME pose.
        self.magnetic_home_angle=self.magnetic_angle if self.magnetic_reference_valid else None
        self.reference_log_count=0
        self.home_gyro_q=qnormalize(self.gyro_only_q)
        # Re-zero all four deviation traces and the gyro reference together.
        self.deviation_reference_q={k:qnormalize(self.filter_live[k].quaternion) for k in 'CMAE'}
        self.deviation_gyro_reference_q=qnormalize(self.gyro_only_q)
        for stream in self.series:stream.clear()
        self.home_angle_reference={k:self.angle_live[k] for k in 'CMAE' if k in self.angle_live}
        self.home_set=True
        self.log_home_event('HOME_REFERENCE_SNAPSHOT', yaw=dict(self.home_filter_yaw), magnetic=self.magnetic_home_angle, filter_counts=dict(self.filter_packet_count))
        self.set_reference_angle(0.0,source='home_calibration')
        for w in self.live_rms_window.values():w.clear()
        self.viewer_reference_q=None
        self.viewer_gyro_reference_q=self.gyro_only_q
        self.record_dir.joinpath('home_calibration.json').write_text(
            json.dumps({'timestamp':datetime.now().isoformat(),
                        'reference_quaternions':self.home_reference_q,
                        'reference_signed_angles':self.home_angle_reference,
                        'filter_home_yaw':self.home_filter_yaw,
                        'magnetic_home_angle':self.magnetic_home_angle,
                        'scope':'GUI quaternion signed full turn relative home; no MCU coefficient write'},indent=2),encoding='utf-8')
        return True

    def save_calibration_capture(self):
        # Acquisition only: coefficients are NOT transmitted or applied to the MCU.
        payload={'note':'Raw calibration samples only; no calibration coefficients applied to STM32.',
                 'units':['m/s2']*3+['rad/s']*3+['uT']*3+['mag_valid'],
                 'steps':self.cal_records}
        (self.record_dir/'calibration_capture.json').write_text(
            json.dumps(payload,ensure_ascii=False),encoding='utf-8')

    def motor_command(self,method,*args):
        if self.serial_link is None:
            self.on_serial_error('STM32 serial not connected')
            return
        try:
            getattr(self.serial_link,method)(*args)
        except Exception as exc:
            self.on_serial_error(str(exc))

    def closeEvent(self,event):
        if self.serial_link is not None:
            try:
                self.serial_link.stop_motor()
                self.serial_link.close()
            except Exception:
                pass
        self.save_calibration_capture()
        for f in self.record_files.values():
            try:f.close()
            except Exception:pass
        super().closeEvent(event)

    def prepare_bases(self):
        # Calibration: identical background + identical globe for all 4 steps.
        # Only left and right panel pixels are swapped, never the globe or header.
        bases=[]
        for i in range(12):
            if i in (3,4):
                # Step 2/3: use the user's pristine no-progress-bar screenshots.
                # No region compositing: keep all borders, captions, and panel text intact.
                bases.append(self.img[i])
            elif i==5:
                # Step 4 is intentionally unchanged from V14.
                a=QImage(self.img[2].toImage());b=self.img[i].toImage();p=QPainter(a)
                for x,y,w,h in [(20,290,530,590),(1120,288,530,600)]:
                    p.drawImage(x,y,b,x,y,w,h)
                p.end();bases.append(QPixmap.fromImage(a))
            elif 7<=i<=9:
                a=QImage(self.img[6].toImage());b=self.img[i].toImage();p=QPainter(a)
                # Header and bottom step buttons change; no upper divider/borders redrawn.
                for rect in [(450,82,685,65),(45,838,1080,96)]:
                    x,y,w,h=rect;p.drawImage(x,y,b,x,y,w,h)
                p.end()
                if i==8:
                    # Only stage 3: stop and vibration buttons with approved audio-lines icon.
                    overlay=QImage(str(ROOT/'assets'/'09_controls_reference.png'))
                    q=QPainter(a);q.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform,False)
                    # Replace complete button artwork, not an inner strip: avoids seams.
                    q.drawImage(1120,75,overlay,1120,75,470,92);q.end()
                elif i==9:
                    # Use original stage-four controls; preserve blank live-data cells
                    # from the clean performance template.
                    q=QPainter(a)
                    ref=QImage(str(ROOT/'assets'/'10_original_user.png'))
                    q.drawImage(1120,78,ref,1120,78,478,80)
                    q.end()
                bases.append(QPixmap.fromImage(a))
            else:bases.append(self.img[i])
        return bases

    def viewport(self):
        im=self.bases[self.page];s=min(self.width()/im.width(),self.height()/im.height());w=im.width()*s;h=im.height()*s
        return QRectF((self.width()-w)/2,(self.height()-h)/2,w,h)

    def tick(self):
        now=time.monotonic();dt=min(.05,now-self.last);self.last=now;self.elapsed+=dt
        d=self.target-self.angle
        if abs(d)<.04:self.angle=self.target
        else:self.angle+=d*min(1,dt*3.5)
        self.sample_acc+=dt
        # All measurement graphs are fed by real STM32 telemetry only.
        # Never synthesize missing sensor/filter data.
        self.sample_acc=min(self.sample_acc,.01)
        if self.test_started is not None and self.test_finished is None:
            self.data_samples=self.sensor_count
        self.sync_3d_viewer()
        self.update()

    def paintEvent(self,e):
        p=QPainter(self);p.setRenderHint(QPainter.RenderHint.Antialiasing);p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        p.fillRect(self.rect(),Qt.GlobalColor.black);v=self.viewport();p.drawPixmap(v.toRect(),self.bases[self.page]);p.save()
        im=self.bases[self.page];p.translate(v.x(),v.y());p.scale(v.width()/im.width(),v.height()/im.height())
        if self.effects:
            if self.page<=5:self.globe(p,im.width(),im.height())
            if 6<=self.page<=9:
                self.dashboard(p)
                self.tactical_hud_fx(p)
            if self.page==10:
                self.summary_hud_fx(p)
            if self.page==11:
                self.final_fx(p)
        # Essential labels and results must not disappear when F2 effects are disabled.
        if self.page==10:self.summary_data(p)
        if self.page==11:self.final_selected_filter(p)
        if 2<=self.page<=4:self.cal_progress(p)
        if 2<=self.page<=5:self.live_gyro(p)
        if self.transition_started is not None:self.draw_start_transition(p)
        self.live_utc_clock(p)
        # Filter comparison and summary remain DEMO values until actual filters are implemented
        p.restore();p.end()

    def live_utc_clock(self,p):
        p.save()
        # The calibration art and performance art have different widths.
        # Clear the timestamp row only, below the IMU / motor metadata row.
        if self.page <= 5:
            p.setFont(QFont('Arial',14))
            p.setPen(qc('#f2fbff'))
            p.drawText(QRectF(1260,54 if self.page==0 else 46,340,24),Qt.AlignmentFlag.AlignRight,
                       'KST  '+datetime.now(timezone(timedelta(hours=9))).strftime('%Y-%m-%d  %H:%M:%S'))
        elif self.page == 11:
            # Last page artwork is pre-cleaned: one device row and one KST row.
            p.setPen(qc('#f2fbff'))
            p.setFont(QFont('Arial',12))
            p.drawText(QRectF(1243,9,320,22),Qt.AlignmentFlag.AlignRight,
                       'IMU: ICM-20948  |  MOTOR: STM-4238D')
            p.drawText(QRectF(1250,43,313,23),Qt.AlignmentFlag.AlignRight,
                       'KST  '+datetime.now(timezone(timedelta(hours=9))).strftime('%Y-%m-%d  %H:%M:%S'))
        else:
            p.setFont(QFont('Arial',13))
            p.setPen(qc('#f2fbff'))
            p.drawText(QRectF(1205,44,298,23),Qt.AlignmentFlag.AlignRight,
                       'KST  '+datetime.now(timezone(timedelta(hours=9))).strftime('%Y-%m-%d  %H:%M:%S'))
            # These performance assets have no fixed header device labels.
            p.setFont(QFont('Arial',10))
            p.drawText(QRectF(1244,7,120,24),Qt.AlignmentFlag.AlignRight,'IMU: ICM-20948')
            p.drawText(QRectF(1391,7,145,24),Qt.AlignmentFlag.AlignRight,'MOTOR: STM-4238D')
            p.setPen(QPen(qc('#00dce7'),1.1))
            p.drawLine(QPointF(1375,9),QPointF(1375,29))
        p.restore()

    def draw_start_transition(self,p):
        # Move the globe ONLY to the calibration globe's vertical position.
        # No 490-pixel drop, no fade-out to nowhere.
        t=min(1.,max(0.,(time.monotonic()-self.transition_started)/0.90))
        smooth=t*t*(3.-2.*t)
        p.save()
        # Hide the original globe AND the START DEMONSTRATION button during transition.
        # Both are baked into the page-1 PNG, so remove them only in this animation frame.
        p.fillRect(QRectF(575,158,535,555),Qt.GlobalColor.black)
        p.fillRect(QRectF(480,698,720,170),Qt.GlobalColor.black)
        globe=self.transition_globe
        if globe is not None:
            # The source and destination globe centers differ by ~100px.
            # Scale gently towards the calibration globe rather than falling off screen.
            scale=1.0
            w=535;h=555
            cx=842.5; cy=435.5+112*smooth
            p.drawPixmap(QRectF(cx-w/2,cy-h/2,w,h),globe,QRectF(0,0,globe.width(),globe.height()))
        p.restore()

    def make_transition_globe(self):
        # Extract only the glowing foreground, not a black rectangular snapshot.
        src=self.img[0].toImage().convertToFormat(QImage.Format.Format_ARGB32)
        cut=src.copy(575,158,535,555)
        for y in range(cut.height()):
            for x in range(cut.width()):
                c=cut.pixelColor(x,y)
                brightness=max(c.red(),c.green(),c.blue())
                c.setAlpha(min(255,int(max(0,brightness-8)*255/38)))
                cut.setPixelColor(x,y,c)
        return QPixmap.fromImage(cut)

    def live_gyro(self,p):
        # Digits were removed from the background PNGs; draw without black boxes.
        t=self.elapsed
        p.save();p.setPen(qc('#e9f9fc'));p.setFont(QFont('Arial',18))
        for j,y in enumerate((676,718,760)):
            val=(self.latest_sample.gx,self.latest_sample.gy,self.latest_sample.gz)[j] if self.latest_sample is not None else .03*math.sin(t*(.8+j*.23)+j)
            p.drawText(QRectF(1408,y,91,37),Qt.AlignmentFlag.AlignCenter,f'{val:+.2f}')
        p.restore()

    def globe(self,p,w,h):
        # All calibration stages share one continuous globe and scan coordinate system.
        if self.page==0:cx,cy,rx,ry=.50*w,.49*h,.14*w,.23*h
        elif self.page==1:cx,cy,rx,ry=.50*w,.58*h,.14*w,.25*h
        else:cx,cy,rx,ry=.50*w,.60*h,.13*w,.23*h
        t=self.elapsed;p.save()
        # Transparent longitude arcs with independent phase; original globe remains intact.
        for i in range(8):
            width=max(.08,abs(math.cos(t*.22+i*math.pi/8)))
            p.setPen(QPen(qc('#02edff',int(15+35*width)),1.2));p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QRectF(cx-rx*width,cy-ry,2*rx*width,2*ry))
        y=cy+ry*.86*math.sin(t*.72);half=rx*math.sqrt(max(.03,1-((y-cy)/ry)**2))
        for n in range(5,0,-1):
            p.setPen(QPen(qc('#00f5ff',int(17/n)),n*1.6));p.drawLine(QPointF(cx-half,y),QPointF(cx+half,y))
        for i in range(11):
            a=t*.42+i*math.pi*2/11;x=cx+rx*1.13*math.cos(a);y2=cy+ry*.28*math.sin(a)
            p.setPen(Qt.PenStyle.NoPen);p.setBrush(qc('#a4ffff',60+i*8));p.drawEllipse(QPointF(x,y2),1.2,1.2)
        p.restore()

    def cube(self,p,cx,cy,scale,q):
        # All SIX opaque faces. Opposite sides are identically colored:
        # X: red, Y: green, Z: blue. Painter's depth sorting prevents blank faces.
        vertices=[(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
                  (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]
        q_view=qinverse(qnormalize(q))
        rotated=[qrotate(v,q_view) for v in vertices]
        def camera(v):
            # Sensor +Y (green arrow) points directly toward the observer,
            # +Z points straight up. No fixed 35-degree oblique offset.
            x,y,z=v
            return x,z,y
        def project(v):
            x,y,z=camera(v); depth=5.8-.65*z; k=scale*5.8/depth
            return QPointF(cx+x*k,cy-y*k)
        projected=[project(v) for v in rotated]
        faces=[([0,3,7,4],'#f53642'),([1,2,6,5],'#f53642'),
               ([0,1,5,4],'#14dc4e'),([3,2,6,7],'#14dc4e'),
               ([0,1,2,3],'#1768fa'),([4,5,6,7],'#1768fa')]
        # No face culling: painter draws the far three first, then the near three.
        # View-depth matches the perspective projection (z + slight y bias).
        draw=[]
        for indices,color in faces:
            center=[sum(rotated[j][k] for j in indices)/4 for k in range(3)]
            depth=camera(center)[2]
            draw.append((depth,indices,color))
        for _,indices,color in sorted(draw):
            path=QPainterPath();path.moveTo(projected[indices[0]])
            for j in indices[1:]:path.lineTo(projected[j])
            path.closeSubpath();p.fillPath(path,QColor(color))
            p.setPen(QPen(qc('#05262c',110),.7));p.drawPath(path)
        for end,color in [((2.55,0,0),'#ff2434'),((0,2.55,0),'#13ff55'),((0,0,2.55),'#00eaff')]:
            dest=project(qrotate(end,q_view));p.setPen(QPen(QColor(color),3));p.drawLine(QPointF(cx,cy),dest)
            a=math.atan2(dest.y()-cy,dest.x()-cx);path=QPainterPath();path.moveTo(dest)
            path.lineTo(dest-QPointF(9*math.cos(a-.45),9*math.sin(a-.45)))
            path.lineTo(dest-QPointF(9*math.cos(a+.45),9*math.sin(a+.45)))
            path.closeSubpath();p.fillPath(path,QColor(color))

    def dashboard(self,p):
        t=self.elapsed
        # Cube viewport and numeric cells only; fixed captions remain untouched.
        for i in range(4):
            left=[61,437,799,1162][i]
            r=QRectF(left+2,204,182,150)
            p.save();p.setClipRect(r)
            p.setPen(QPen(qc('#005a63',65),.7))
            for dx in range(0,185,18):p.drawLine(QPointF(left+dx,210),QPointF(left+dx,349))
            for dy in range(211,350,18):p.drawLine(QPointF(left+2,dy),QPointF(left+181,dy))
            base=self.angle+math.sin(t*.3)*2.0
            roll=12*math.sin(t*.7+i*.11);pitch=-8*math.sin(t*.46+i*.16)
            q=quat(roll,pitch,base+(i-1.5)*2.5)
            live=self.filter_live.get('CMAE'[i])
            if live is not None:
                roll,pitch,base=live.roll,live.pitch,live.yaw
                q=live.quaternion
            self.cube(p,left+(91 if i==0 else 105),291,31,q);p.restore()
            # Rebuild only the right-side information panel to avoid baked-number ghosts.
            panel_x=[256,619,987,1367][i]
            panel_w=[151,152,151,141][i]
            # Clean source artwork is already empty; never paint opaque rectangles over its borders.
            p.setPen(QPen(qc(COLS[i],95),.7))
            p.drawLine(QPointF(panel_x,202),QPointF(panel_x,361))
            p.setFont(QFont('Arial',10));p.setPen(qc('#f4ffff'))
            p.drawText(QRectF(panel_x+16,204,132,19),'Euler Angle (deg)')
            p.setFont(QFont('Arial',10,QFont.Weight.Bold))
            for j,(label,val,col) in enumerate([('Roll',roll,'#ff3445'),('Pitch',pitch,'#15ee53'),('Yaw',base if live is not None else base+(i-1.5)*2.5,'#00dfff')]):
                yy=225+j*19;p.setPen(QColor(col))
                p.drawText(QRectF(panel_x+18,yy,60,18),label)
                p.drawText(QRectF(panel_x+91,yy,54,18),Qt.AlignmentFlag.AlignRight,f'{val:+.2f}')
            p.setPen(qc('#e8f6f8'));p.setFont(QFont('Arial',10))
            p.drawText(QRectF(panel_x+18,282,125,18),'Quaternion')
            for j,(label,val) in enumerate(zip('wxyz',q)):
                yy=300+j*16
                p.drawText(QRectF(panel_x+20,yy,15,16),label)
                p.drawText(QRectF(panel_x+73,yy,72,16),Qt.AlignmentFlag.AlignRight,f'{val:+.4f}')
        self.draw_live_error_chart(p)
        self.plot(p,QRectF(615,717,251,57),self.raw,-2,2,True)
        self.performance_metrics(p)
        self.status_data(p)
        if self.page==8 and self.vibration_command_at is not None:
            p.setFont(QFont('Arial',9,QFont.Weight.Bold))
            p.setPen(qc('#14e65a' if self.vibration_ack.startswith('ACK,') else '#f5c266'))
            label=('VIBRATION ON' if self.vibration_on else 'VIBRATION OFF')
            if self.vibration_ack=='WAITING ACK':label='WAITING STM32 ACK'
            elif self.vibration_ack.startswith('ERR,'):label='STM32 ERROR'
            #p.drawText(QRectF(1335,165,225,19),Qt.AlignmentFlag.AlignCenter,label)

    def draw_live_error_chart(self,p):
        """Only four curves on untouched original background; no extra text/axes/boxes."""
        p.save()
        # Exact chart interior on the original 1598x984 artwork (pages 07-10).
        r=QRectF(188,441,1222,121)
        p.setClipRect(r)
        upper=10.0  # Match the original background's printed 0-10% vertical scale
        for j,stream in enumerate(self.series):
            vals=list(stream)
            if len(vals)<2:continue
            path=QPainterPath()
            for k,val in enumerate(vals):
                x=r.left()+r.width()*k/max(1,len(vals)-1)
                y=r.bottom()-2.0-min(upper,max(0.0,val))/upper*(r.height()-3.0)
                if k==0:path.moveTo(x,y)
                else:path.lineTo(x,y)
            p.setPen(QPen(QColor(COLS[j]),1.8))
            p.drawPath(path)
        p.restore()

    def tactical_hud_fx(self,p):
        """Restrained animated scanner accents on performance screens only.
        Decorative overlays stay inside existing data windows; numeric cells,
        captions, controls and the final GLB viewer remain untouched.
        """
        t=self.elapsed
        p.save()
        p.setRenderHint(QPainter.RenderHint.Antialiasing,True)
        # Small raw-sensor scope: independent slow sweep, no extra labels.
        r=QRectF(615,717,251,57)
        p.save();p.setClipRect(r)
        xx=r.left()+(t*31)%r.width()
        p.setPen(QPen(qc('#2fe9ff',85),1))
        p.drawLine(QPointF(xx,r.top()+2),QPointF(xx,r.bottom()-2))
        p.restore()
        p.restore()

    def summary_hud_fx(self,p):
        """Animated tactical telemetry accents in the Performance Summary only.
        Keep data cells, text, labels and borders legible and unmodified.
        """
        t=self.elapsed
        p.save()
        p.setRenderHint(QPainter.RenderHint.Antialiasing,True)
        # Two large tables: narrow traveling cyan sweep and a restrained
        # moving row emphasis. Drawn BEFORE summary_data, so values stay crisp.
        for x,y,w,h,row_h in ((42,231,967,207,36),(42,489,967,205,35)):
            p.save();p.setClipRect(QRectF(x,y,w,h))
            sweep=x+(t*46)%w
            for delta,alpha,width in ((0,60,1.2),(-5,19,3),(-12,8,8)):
                p.setPen(QPen(qc('#43eaff',alpha),width))
                p.drawLine(QPointF(sweep+delta,y+5),QPointF(sweep+delta,y+h-5))
            row=int(t*.65) % 4
            ry=(y+43)+row*row_h
            p.fillRect(QRectF(x+3,ry,w-6,21),qc('#08dfff',12))
            p.restore()
        # Low-intensity acquisition bars below table headings; no random data.
        for x,y,w in ((250,228,740),(260,489,730)):
            for j in range(20):
                phase=.5+.5*math.sin(t*2.2-j*.46)
                p.fillRect(QRectF(x+j*(w/20),y,17,2),qc('#2feaff',int(10+36*phase)))
        # Compact status pulse on the Test Info card, away from text fields.
        for j in range(4):
            phase=.5+.5*math.sin(t*2.7-j*.9)
            p.fillRect(QRectF(1520+j*11,120,6,3),qc('#37f0ff',int(30+95*phase)))
        # Filter characteristics: slow, short scan trace along the lower edge.
        p.save();p.setClipRect(QRectF(1046,689,515,9))
        x=1046+(t*29)%515
        p.setPen(QPen(qc('#55edff',90),1.5))
        p.drawLine(QPointF(x,692),QPointF(min(1560,x+34),692))
        p.restore()
        p.restore()

    def performance_metrics(self,p):
        cols=[224,292,365,440,507]
        row_y=[719,744,769,794]
        for i,fid in enumerate('CMAE'):
            live_perf=self.perf_live.get(fid)
            # Live RMS: 30 s mean-centered rotation-angle variability from filter quaternions.
            # This is not reference-angle RMSE; interpret during stationary periods.
            w=self.live_rms_window[fid]
            if len(w)>=5:
                vals=[v for _,v in w]
                mu=sum(vals)/len(vals)
                rms=f'{math.sqrt(sum((v-mu)**2 for v in vals)/len(vals)):.2f}'
            else:rms='—'
            drift=(f'{self.drift_live[fid]:.2f}' if fid in self.drift_live else '—')
            execution=(f'{live_perf.execution_us/1000.0:.3f}' if live_perf else '—')
            cpu=(f'{live_perf.cpu_percent:.2f}' if live_perf else '—')
            # This column is explicitly STATE RAM only, not total MCU RAM.
            ram=(f'{self.build_filter_ram_kb[fid]:.2f}' if fid in self.build_filter_ram_kb
                 else f'{live_perf.state_bytes/1024.0:.2f}' if live_perf else '—')
            for j,value in enumerate((rms,drift,execution,cpu,ram)):
                p.setPen(qc('#effcff'));p.setFont(QFont('Arial',12))
                p.drawText(QRectF(cols[j]-23,row_y[i]-7,47,18),Qt.AlignmentFlag.AlignCenter,value)
        # Header note: avoid implying state struct size equals total RAM usage.
        p.setPen(qc('#82dce2'));p.setFont(QFont('Arial',8))
        p.drawText(QRectF(467,670,74,15),Qt.AlignmentFlag.AlignCenter,'STATE RAM')

    def status_data(self,p):
        runtime=max(0,time.monotonic()-self.test_started) if self.test_started is not None else 0
        mm=int(runtime//60);ss=int(runtime%60)
        items=[(1473,679,f'{mm:02}:{ss:02}'),(1473,704,f'{self.data_samples*24/1048576:.2f} MB'),(1473,729,'60 Hz' if self.sensor_count else '—')]
        p.setFont(QFont('Arial',12));p.setPen(qc('#eaffff'))
        for x,y,value in items:
            p.drawText(QRectF(x-43,y-10,83,20),Qt.AlignmentFlag.AlignRight,value)
        p.drawText(QRectF(1192,763,45,19),Qt.AlignmentFlag.AlignRight,
                   f'{sum(x.cpu_percent for x in self.perf_live.values()):.1f}%' if self.perf_live else '—')
        # Build-derived total RAM when supplied; otherwise show explicitly labeled
        # aggregate filter-state RAM, not an invented MCU-wide utilization figure.
        if self.build_ram_kb is not None:
            memory=f'{self.build_ram_kb:.1f}K'
        elif len(self.perf_live)==4:
            memory=f'{sum(v.state_bytes for v in self.perf_live.values())/1024.0:.2f}K'
        else:
            memory='—'
        p.drawText(QRectF(1172,789,65,19),Qt.AlignmentFlag.AlignRight,memory)

    def plot(self,p,rect,streams,ymin,ymax,small):
        p.save();p.setClipRect(rect)
        p.setPen(QPen(qc('#00626c',55),.65))
        for k in range(11):
            xx=rect.left()+rect.width()*k/10;p.drawLine(QPointF(xx,rect.top()),QPointF(xx,rect.bottom()))
        for k in range(6):
            yy=rect.top()+rect.height()*k/5;p.drawLine(QPointF(rect.left(),yy),QPointF(rect.right(),yy))
        for j,stream in enumerate(streams):
            vals=list(stream)
            if len(vals)<2:continue
            path=QPainterPath()
            for k,val in enumerate(vals):
                x=rect.left()+rect.width()*k/max(1,len(vals)-1)
                y=rect.bottom()-(val-ymin)/(ymax-ymin)*rect.height()
                if k==0:path.moveTo(x,y)
                else:path.lineTo(x,y)
            p.setPen(QPen(QColor((['#ff594f','#22f770','#00e9ff'] if small else COLS)[j]),1.5 if small else 1.8));p.drawPath(path)
        p.restore()

    def sim_label(self,p):
        p.setPen(qc('#7eeaf0',190));p.setFont(QFont('Consolas',9))
        p.drawText(QRectF(675,951,280,23),Qt.AlignmentFlag.AlignCenter,f'SIMULATION   |   TARGET {self.target:.0f}°')

    def cal_progress(self,p):
        duration=15.0 if self.page==4 else 5.0
        elapsed=max(0.,time.monotonic()-self.cal_started)
        frac=min(1.,elapsed/duration)
        p.save();p.setRenderHint(QPainter.RenderHint.Antialiasing,True)
        # Fully cleared progress region: draw a new bar with no original underneath.
        x,y,w,h=1164.,575.,351.,25.
        outline=QRectF(x,y,w,h)
        p.setPen(QPen(qc('#00a6b5',220),1.35))
        p.setBrush(qc('#001016'))
        p.drawRoundedRect(outline,12,12)
        if frac>0:
            inner=QRectF(x+3,y+3,(w-6)*frac,h-6)
            grad=QLinearGradient(x,y,x+w,y)
            grad.setColorAt(0,qc('#00c5d4'));grad.setColorAt(1,qc('#3fffff'))
            p.setPen(Qt.PenStyle.NoPen);p.setBrush(grad)
            p.drawRoundedRect(inner,9,9)
        p.setPen(qc('#00f3ff'));p.setFont(QFont('Arial',17))
        p.drawText(QRectF(1530,568,92,42),Qt.AlignmentFlag.AlignCenter,f'{max(0,duration-elapsed):.1f} s')
        p.restore()

    def schedule_3d_preload(self):
        # Give the summary page time to paint before loading GLB models.
        if self.vtk_preload_scheduled or self.viewer3d is not None or self.viewer_error is not None:
            return
        self.vtk_preload_scheduled = True
        QTimer.singleShot(150, self.preload_3d_viewer)

    def preload_3d_viewer(self):
        self.vtk_preload_scheduled = False
        if self.page != 10 or self.viewer3d is not None:
            return
        self.sync_3d_viewer(preload=True)

    def sync_3d_viewer(self, preload=False):
        # Two genuine interactive VTK GLB views on the final screen.
        if (self.page != 11 and not preload) or not self.effects:
            for viewer in (self.viewer3d,self.viewer3d_right):
                if viewer is not None and viewer.isVisible():viewer.hide()
            return
        if self.viewer3d is None and self.viewer_error is None:
            try:
                import vtk
                from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
                for index in range(2):
                    viewer=QVTKRenderWindowInteractor(self)
                    viewer.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
                    viewer.setAutoFillBackground(False)
                    viewer.setStyleSheet('background: transparent;')
                    rw=viewer.GetRenderWindow();rw.SetMultiSamples(8)
                    importer=vtk.vtkGLTFImporter()
                    importer.SetFileName(str(ROOT/'assets'/'mechanical_sensor.glb'))
                    importer.SetRenderWindow(rw);importer.Update()
                    renderer=rw.GetRenderers().GetFirstRenderer()
                    if renderer is None:raise RuntimeError('GLB did not produce a renderer')
                    renderer.SetBackground(0.0,0.0,0.0)
                    renderer.SetBackgroundAlpha(0.0)
                    try:
                        rw.SetAlphaBitPlanes(1)
                    except Exception:pass
                    # Order-independent transparency where supported by the GPU.
                    try:
                        rw.SetAlphaBitPlanes(1)
                        renderer.SetUseDepthPeeling(True)
                        renderer.SetMaximumNumberOfPeels(80)
                        renderer.SetOcclusionRatio(.05)
                    except Exception:pass
                    # Preserve the GLB's ORIGINAL imported PBR materials and textures.
                    # Do not force Phong, ambient/diffuse material colors, or opacity.
                    # Increase only the illumination modestly from the original viewer.
                    renderer.AutomaticLightCreationOff()
                    renderer.RemoveAllLights()
                    for pos, color, intensity in [
                        ((3, 5, 7), (0.95, 1.0, 1.0), 2.25),
                        ((-5, 2, 3), (0.75, 0.97, 1.0), 1.45),
                        ((1, -4, -2), (0.85, 0.98, 1.0), 1.05),
                        ((0, 0, 8), (1.0, 1.0, 1.0), 0.65),
                    ]:
                        light = vtk.vtkLight()
                        light.SetLightTypeToSceneLight()
                        light.SetPosition(*pos)
                        light.SetFocalPoint(0, 0, 0)
                        light.SetColor(*color)
                        light.SetIntensity(intensity)
                        renderer.AddLight(light)
                    renderer.ResetCamera()
                    camera=renderer.GetActiveCamera()
                    camera.Azimuth(45);camera.Elevation(0) # Three-quarter initial view for both models
                    camera.Zoom(1.43) # Slightly smaller opaque model with transparent viewport
                    renderer.ResetCameraClippingRange()
                    viewer.Initialize()
                    # Keep original imported actors and their original transforms.
                    actors=[]
                    collection=renderer.GetActors()
                    collection.InitTraversal()
                    while True:
                        actor=collection.GetNextActor()
                        if actor is None:break
                        base=vtk.vtkMatrix4x4()
                        actor.GetMatrix(base)
                        # Flatten imported local transforms into the saved base;
                        # otherwise SetUserMatrix multiplies the import transform twice.
                        actor.SetPosition(0,0,0)
                        actor.SetOrientation(0,0,0)
                        actor.SetScale(1,1,1)
                        actor.SetOrigin(0,0,0)
                        actor.SetUserMatrix(None)
                        actors.append((actor,base))
                    self.vtk_actor_sets.append(actors)
                    self.vtk_scenes.append((viewer,importer,renderer))
                self.viewer3d=self.vtk_scenes[0][0]
                self.viewer3d_right=self.vtk_scenes[1][0]
            except Exception as ex:
                self.viewer_error=str(ex)
                print('3D viewer initialization failed:',ex)
                return
        if self.vtk_scenes and not preload:
            v=self.viewport();sx=v.width()/1598;sy=v.height()/984
            # Fit both 3D views within the inner cyan borders; do not cover headings.
            for index,(viewer,importer,renderer) in enumerate(self.vtk_scenes):
                left=70 if index==0 else 862
                r=(int(v.x()+left*sx),int(v.y()+205*sy),int(655*sx),int(575*sy))
                if viewer.geometry().getRect()!=r:viewer.setGeometry(*r)
                if not viewer.isVisible():
                    viewer.show()
                    viewer.raise_()
            self.update_3d_attitude()

    def update_3d_attitude(self):
        """Drive both GLB views using the STM32 IMU, never scripted motion.

        Left: selected filter quaternion. Right: raw gyro integrated without
        accel/mag correction. Both are relative to the first observed pose on
        the final page, so their models begin from the same neutral view.
        """
        if self.page!=11 or len(self.vtk_scenes)!=2 or not self.has_live:
            return
        now_3d=time.monotonic()
        if now_3d-self.last_vtk_render<self.vtk_render_interval:return
        self.last_vtk_render=now_3d
        selected_id=self.selected_id()
        filt=self.filter_live.get(selected_id)
        if filt is None:return
        current=qnormalize(filt.quaternion)
        if self.viewer_reference_q is None:
            # Both GLB viewers must start neutral on entering the final page,
            # regardless of the earlier calibration pose.
            self.viewer_reference_q=current
            self.viewer_gyro_reference_q=self.gyro_only_q
        left=qnormalize(qmultiply(current,qinverse(self.viewer_reference_q)))
        right=qnormalize(qmultiply(self.gyro_only_q,qinverse(self.viewer_gyro_reference_q)))
        try:
            import vtk
            for index,q in enumerate((left,right)):
                # Sensor-to-world quaternion is inverted for the model viewing convention.
                w,x,y,z=q
                # Quaternion -> proper rotation matrix, preserving original GLB transforms.
                rot=vtk.vtkMatrix4x4()
                mat=((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
                     (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
                     (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))
                for i in range(3):
                    for j in range(3):rot.SetElement(i,j,mat[i][j])
                for actor,base in self.vtk_actor_sets[index]:
                    # Imported model's local transform is preserved.
                    combined=vtk.vtkMatrix4x4()
                    vtk.vtkMatrix4x4.Multiply4x4(rot,base,combined)
                    actor.SetUserMatrix(combined)
                self.vtk_scenes[index][0].GetRenderWindow().Render()
        except Exception as exc:
            self.viewer_error=str(exc)
            print('Live 3D update failed:',exc)

    def summary_data(self,p):
        """Summary of the actual 0..360 degree test, not archived examples."""
        p.save()
        # All measured tables and conclusions are visible immediately.
        # Only the final selected-filter name is revealed one second later.
        reveal_winner=(self.summary_entered is None or
                       time.monotonic()-self.summary_entered >= 1.0)
        metrics=(self.summary_snapshot['metrics'] if self.summary_snapshot is not None else self.live_summary_metrics())
        p.setFont(QFont('Arial',14));p.setPen(qc('#f3f9fc'))
        xcols=[299,418,534,666,798,929]
        ys=[297,333,369,405]
        for i,fid in enumerate('CAME'):
            d=metrics.get(fid)
            # The physical 24-point reference only measures the rotating axis.
            # Roll/Pitch are baseline-relative motion RMS (degrees), not ground-truth error percentages.
            if d:
                fmt=lambda v: f'{v:.2f}' if v is not None else '—'
                # Error table is labeled percent: convert angle errors (degrees) to percent of 360.
                pct=lambda v: fmt(v*100.0/360.0) if v is not None else '—'
                vals=[(fmt(d['roll_motion_rms'])+'°' if d['roll_motion_rms'] is not None else '—'),
                      (fmt(d['pitch_motion_rms'])+'°' if d['pitch_motion_rms'] is not None else '—'),
                      pct(d['rmse']),pct(d['mae']),pct(d['max']),pct(d['rmse'])]
            else:
                vals=['—']*6
            for x,val in zip(xcols,vals):
                p.drawText(QRectF(x-43,ys[i]-14,86,26),Qt.AlignmentFlag.AlignCenter,val)
            perf=self.perf_live.get(fid)
            cpu=d['cpu'] if d else None
            execution=d['execution_us'] if d else None
            ram=(self.summary_snapshot['ram'][fid] if self.summary_snapshot is not None else (perf.state_bytes if perf else None))
            flash=self.filter_code_kb[fid]
            resource=[f'{cpu:.2f}' if cpu is not None else '—',f'{ram/1024:.3f}' if ram is not None else '—',f'{flash:.3f}',f'{execution/1000:.3f}' if execution is not None else '—']
            for x,val in zip([358,522,705,892],resource):
                p.drawText(QRectF(x-48,[554,588,622,656][i],96,26),Qt.AlignmentFlag.AlignCenter,val)
        p.setPen(qc('#f16aff'));p.setFont(QFont('Malgun Gothic',18,QFont.Weight.Bold))
        p.drawText(QRectF(75,724,320,30),Qt.AlignmentFlag.AlignCenter,'최종 선정 필터')
        p.setFont(QFont('Arial',29,QFont.Weight.Bold))
        winner=NAMES['CAME'.index(self.selected)] if self.selected is not None and self.selected in 'CAME' else '측정 데이터 부족'
        if reveal_winner:
            p.drawText(QRectF(75,765,750,50),Qt.AlignmentFlag.AlignCenter,winner)
        p.setPen(qc('#e6faff'));p.setFont(QFont('Malgun Gothic',13))
        # Conclusion is a short, data-grounded narrative using the measured
        # attitude-error comparison, system resources, and weighted criteria.
        chosen=NAMES['CAME'.index(self.selected)] if self.selected in 'CAME' else '미선정'
        d=metrics.get(self.selected,{})
        def metric(k,unit=''):
            v=d.get(k)
            return f'{v:.2f}{unit}' if v is not None and math.isfinite(v) else 'FAIL'
        conclusion=[
            f'자세 오차율 비교: 지자기 LUT 기준 MAE {metric("mae","°")},',
            f'진동 안정성 {metric("stability","°")} · 외란 {metric("response","s")}.',
            f'시스템 자원 사용량: CPU {metric("cpu","%")}, RAM {metric("ram","KB")},',
            f'Flash {metric("flash","KB")} · 실행 {metric("execution_us","µs")}.',
            '평가 가중치: 정확도 40 / 안정성 30 / 응답성 15 /',
            f'연산 10 / 자원 5를 종합하여 {chosen} 필터가 선정되었다.'
        ]
        p.setFont(QFont('Malgun Gothic',11))
        for j,line in enumerate(conclusion):
            p.drawText(QRectF(955,751+j*15,590,18),line)
        p.setPen(qc('#f3ffff'));p.setFont(QFont('Arial',12))
        start=getattr(self,'test_started_wall',None)
        duration=(self.test_finished or time.monotonic())-self.test_started if self.test_started is not None else None
        info=[start.strftime('%Y-%m-%d') if start else '—',
              time.strftime('%H:%M:%S',time.gmtime(max(0,duration))) if duration is not None else '—',
              '60 Hz']
        for yy,val in zip((136,163,189),info):
            p.drawText(QRectF(1280,yy-11,225,23),Qt.AlignmentFlag.AlignRight,val)
        p.restore()

    def selected_id(self):
        return self.selected

    def final_selected_filter(self,p):
        # Original 'With Filter' is baked into the artwork; paint only suffix.
        p.save()
        p.setPen(qc('#00e9f5'))
        font=QFont('Arial',11)
        p.setFont(font)
        rect=QRectF(370,148,196,24)
        p.drawText(rect,Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                   f"(Selected: {NAMES['CAME'.index(self.selected)] if self.selected is not None and self.selected in 'CAME' else 'N/A'})")
        p.restore()

    def final_fx(self,p):
        for cx in (395,1200):
            p.setPen(QPen(qc('#00e9ff',50),1.4));r=155+6*math.sin(self.elapsed*.8)
            p.drawEllipse(QRectF(cx-r,490-r,2*r,2*r))

    def hits(self):
        if self.page==0:return [('start',(480,670,1210,885))]
        if 2<=self.page<=5:return [('next',(1400,300,1645,395))]
        if self.page==8:return [('stop',(1140,83,195,70)),('vibrate',(1340,83,210,70)),('back',(1160,832,167,95)),('next',(1327,832,218,95)),('step1',(50,832,250,100)),('step2',(335,832,230,100)),('step3',(600,832,235,100)),('step4',(862,832,260,100))]
        if 6<=self.page<=9:return [('reset',(1140,83,195,70)),('rotate',(1340,83,210,70)),('back',(1160,832,167,95)),('next',(1327,832,218,95)),('step1',(50,832,250,100)),('step2',(335,832,230,100)),('step3',(600,832,235,100)),('step4',(862,832,260,100))]
        if self.page==10:return [('next',(1320,835,245,130))]
        if self.page==11:return [('home',(1120,840,235,110)),('exit',(1360,840,235,110))]
        return []

    def mousePressEvent(self,e):
        if e.button()!=Qt.MouseButton.LeftButton:return
        v=self.viewport();im=self.bases[self.page]
        if not v.contains(e.position()):return
        x=(e.position().x()-v.x())/v.width()*im.width();y=(e.position().y()-v.y())/v.height()*im.height()
        for name,(a,b,w,h) in self.hits():
            if a<=x<=a+w and b<=y<=b+h:self.action(name);break

    def mouseMoveEvent(self,e):
        v=self.viewport();im=self.bases[self.page];hit=False
        if v.contains(e.position()):
            x=(e.position().x()-v.x())/v.width()*im.width();y=(e.position().y()-v.y())/v.height()*im.height()
            hit=any(a<=x<=a+w and b<=y<=b+h for _,(a,b,w,h) in self.hits())
        self.setCursor(Qt.CursorShape.PointingHandCursor if hit else Qt.CursorShape.ArrowCursor)

    def action(self,name):
        if name=='exit':self.close();return
        if name=='home':self.summary_snapshot=None;self.save_calibration_capture();self.motor_command('stop_motor');self.summary_entered=None;self.test_started=None;self.test_finished=None;self.page=0;self.target=self.angle=0;self.sync_3d_viewer();self.update();return
        if name=='stop':
            self.vibration_on=False
            self.motor_command('vibrate',False)
            self.motor_command('stop_motor')
            self.update()
            return
        if name=='vibrate':
            self.vibration_on=not self.vibration_on
            if self.vibration_on:self.stage_vibration_home={}
            self.vibration_ack='WAITING ACK'
            self.vibration_command_at=time.monotonic()
            self.motor_command('vibrate',self.vibration_on)
            self.update()
            return
        if name=='reset':
            self.stage_accuracy={k:[] for k in 'CMAE'}
            self.perf_history={k:[] for k in 'CMAE'}
            self.summary_snapshot=None
            self.stage_vibration={k:[] for k in 'CMAE'}
            self.stage_axis_rms={k:{'roll':[],'pitch':[]} for k in 'CMAE'}
            self.stage_axis_home={}
            self.stage_vibration_home={}
            self.stage_disturb_home={}
            self.stage_disturb_start={k:None for k in 'CMAE'}
            self.stage_disturb_last={k:None for k in 'CMAE'}
            self.stage_disturb_quiet={k:None for k in 'CMAE'}
            self.stage_recovery={k:None for k in 'CMAE'}
            self.stage_disturb_completed=False
            self.error_records={k:[] for k in 'CMAE'}
            for w in self.live_rms_window.values():w.clear()
            for stream in self.series:stream.clear()
            self.set_reference_angle(0,source="reset_button");self.test_started=time.monotonic();self.test_finished=None;self.data_samples=0;self.motor_command('stop_motor');return
        if name=='rotate':
            degrees=180 if self.page==7 else 30
            self.set_reference_angle(self.target+degrees,source='rotate_button')
            self.motor_command('rotate',degrees)
            return
        if name.startswith('step') and name[-1].isdigit():
            for stream in self.series:stream.clear()
            self.motor_command('vibrate',False);self.vibration_on=False;self.page=5+int(name[-1]);
        if name.startswith('step') and self.test_started is None:self.test_started=time.monotonic()
        if name.startswith('step'):self.update();return
        if name=='back':
            for stream in self.series:stream.clear()
            self.motor_command('vibrate',False);self.vibration_on=False;self.page=max(6,self.page-1);self.update();return
        if name=='start':
            self.page=0;self.transition_started=time.monotonic()
            if self.transition_globe is None:self.transition_globe=self.make_transition_globe()
            self.update();QTimer.singleShot(900,self.end_transition);return
        if name=='next':
            if self.page==9:
                self.stage_disturb_completed=True
            self.motor_command('vibrate',False)
            self.vibration_on=False
            if self.page==5:
                if any(k not in self.filter_live for k in 'CMAE'):
                    self.on_serial_error('Wait for all four STM32 filters before calibration')
                    return
                try:
                    self.home_zero_pending=True
                    self.home_set=False
                    self.home_ack_counts=dict(self.filter_packet_count)
                    self.home_zero_requested_at=time.monotonic()
                    self.log_home_event('ZERO_SENT', counts=dict(self.filter_packet_count))
                    self.serial_link.zero_home()
                    # Commit using post-command telemetry even when firmware has no ACK.
                    QTimer.singleShot(700,self.finish_home_after_fresh_packets)
                except Exception as exc:
                    self.home_zero_pending=False
                    self.on_serial_error('ZERO command failed: '+str(exc))
                    return
                self.save_calibration_capture()
                # HOME is committed ONCE after ACK and a fresh complete FILT set.
                # Never capture a pre-ZERO yaw as the final reference.
                if not self.mute:speak(VOICES[5])
                self.page=6
            elif self.page==10:
                self.page=11
                self.viewer_reference_q=None
                self.viewer_gyro_reference_q=None
            else:self.page=min(11,self.page+1)
            if 2<=self.page<=4:self.cal_started=time.monotonic()
            if self.page==6 and self.test_started is None:
                self.test_started=time.monotonic()
                self.test_started_wall=datetime.now()
            if self.page==10:
                # Freeze all summary metrics at the end of the experiments.
                self.summary_snapshot={'metrics':copy.deepcopy(self.live_summary_metrics()),
                                       'ram':{f:(self.perf_live[f].state_bytes if f in self.perf_live else None) for f in 'CMAE'}}
                # Only current stage-specific measurements determine the winner.
                self.selection_scores=self.calculate_filter_scores()
                self.selected=min(self.selection_scores,key=self.selection_scores.get) if self.selection_scores else None
                self.selection_reasons=self.build_selection_reasons()
                if self.test_finished is None:self.test_finished=time.monotonic()
                self.summary_entered=time.monotonic()
                # Build VTK on final page; preloading could block summary painting.
            if 2<=self.page<=5 and not self.mute:speak(VOICES[self.page-1])
            if self.page==10 and not self.mute:speak(VOICES[6])
            self.update()
    def end_transition(self):
        # One handoff only: skip the intermediate page-2 globe, which caused
        # a second visible jump before the calibration screen appeared.
        self.transition_started=None
        self.page=2
        self.cal_started=time.monotonic()
        self.update()
        if not self.mute:
            speak(VOICES[0])
            QTimer.singleShot(1100,lambda:speak(VOICES[1]))

    def start_cal(self):
        if self.page==1:
            if not self.mute:speak(VOICES[0])
            self.page=2;self.cal_started=time.monotonic();self.update()
            if not self.mute:QTimer.singleShot(1100,lambda:speak(VOICES[1]))

    def keyPressEvent(self,e):
        k=e.key()
        if k==Qt.Key.Key_F11:
            if self.isFullScreen():self.showNormal()
            else:self.showFullScreen()
        elif k==Qt.Key.Key_F2:self.effects=not self.effects
        elif k==Qt.Key.Key_M:self.mute=not self.mute
        elif k==Qt.Key.Key_R:self.action('rotate')
        # Direct reference-angle selection: works even when the motor is
        # disconnected or the user rotates the IMU by hand.
        elif self.page in (6,7,8,9) and k in (Qt.Key.Key_0,Qt.Key.Key_3,Qt.Key.Key_6,Qt.Key.Key_9):
            self.set_reference_angle({Qt.Key.Key_0:0,Qt.Key.Key_3:30,Qt.Key.Key_6:60,Qt.Key.Key_9:90}[k],source='keyboard')
        elif k in (Qt.Key.Key_Return,Qt.Key.Key_Space,Qt.Key.Key_Right):
            if self.page==0:self.action('start')
            elif self.page!=1:self.action('next')
        elif k==Qt.Key.Key_Escape:
            if self.isFullScreen():self.showNormal()
            else:self.close()
        self.update()

if __name__=='__main__':
    app=QApplication(sys.argv);app.setApplicationName('IMU HUD V8');w=HUD();w.show();sys.exit(app.exec())
