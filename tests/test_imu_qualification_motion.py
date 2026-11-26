"""Synthetic plan/source adapters only: no fresh seed draws or firmware runs."""
import copy
from collections import Counter
import unittest
from unittest.mock import patch

from kinisi_virtual_board import imu_qualification_motion as motion
from kinisi_virtual_board.imu_qualification_plan import (make_plan, schedule, source_key, PRESETS,
    ROLES, FAULTS, DELAYS, PUBLIC_SEED_RANGE)


def synthetic_source(case):
    """Distinct values per acquired die make every held-channel identity visible."""
    rows = []
    for i in range(902):
        slot, gyro = divmod(i,2)
        a = 600000+10000*slot
        g = a+(2500 if gyro else -7500)
        rows.append(dict(delivered_us=a+10000+2500*gyro,accel_us=a,gyro_us=g,
                         ax=float(a),ay=float(a+1),az=float(a+2),
                         gx=float(g),gy=float(g+1),gz=float(g+2)))
    preset,role,seed = source_key(case)
    return dict(metadata=dict(source_id=case["source_id"],preset=preset,role=role,seed=seed,build_id="invented"),
                sensor_rows=rows,reference_rows=[dict(at_us=580000,x=0,y=0,yaw=0,vx=0,vy=0,wz=0)],
                targets=[dict(at_us=600000,x=0,y=0,yaw=0)])


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.plan = make_plan([131,277,419])  # Previously disclosed public seeds.

    def test_plan_complete_groups_controls_and_randomized_order(self):
        self.assertEqual(len(self.plan),144)
        self.assertEqual(Counter(c["class"] for c in self.plan),{"healthy":135,"fault":9})
        self.assertEqual(Counter(c["preset"] for c in self.plan),dict.fromkeys(PRESETS,48))
        self.assertEqual(len({c["source_id"] for c in self.plan}),45)
        self.assertEqual(self.plan,make_plan([131,277,419]))
        names=[c["case"] for c in self.plan]
        self.assertNotEqual(names,sorted(names))
        self.assertEqual(names,[c["case"] for c in make_plan([1,2,3])])
        by_case={c["case"]:c for c in self.plan}
        for case in self.plan:
            control=by_case[case["control_case"]]
            self.assertEqual(case["source_id"],control["source_id"])
            self.assertEqual(case["targets_ms"],control["targets_ms"])
            self.assertEqual(control["delay_us"],case["delay_us"] if case["role"] in FAULTS else 10000)
            if case["role"] in FAULTS:
                self.assertEqual(control["role"],"stationary")

    def test_schedules_depend_on_seed_and_keep_declared_roles(self):
        for preset in PRESETS:
            for role in ROLES[1:]:
                programs=[schedule(preset,role,s)[0] for s in (131,277,419)]
                self.assertGreater(len({repr(p) for p in programs}),1)
                for program in programs:
                    times=[r[0] for r in program]
                    self.assertEqual(times,sorted(set(times)))
                    self.assertTrue(all(t%10==0 for t in times))
                    self.assertEqual(program[0],[0,[0.,0.,0.]])
                    self.assertTrue(all(row[1][1]==0 for row in program) if preset=="differential" else True)
                    if role=="reverse":
                        self.assertEqual(program[2][1],[-program[1][1][0],-program[1][1][1],0.])
                    if role=="brake":
                        self.assertEqual(program[2][1],[0.,0.,0.])
        self.assertLess(PUBLIC_SEED_RANGE[1]-1+10000,2**32)
        for seeds in ([1,1,2],[1,2],[-1,1,2],[1,2,0xffffffff],[True,2,3]):
            with self.assertRaises(ValueError): make_plan(seeds)

    def test_phase_and_fault_substitutions_preserve_originals(self):
        for role in FAULTS:
            case=next(c for c in self.plan if c["role"]==role)
            source=synthetic_source(case); before=copy.deepcopy(source)
            capture=motion.capture_inputs(source,case)
            actual=capture["sensor_rows"]; nominal=source["sensor_rows"]
            self.assertEqual(source,before)
            for i,row in enumerate(actual):
                slot,gyro=divmod(i,2)
                expected_a=158 if role in ("producer_freeze","accel_freeze") and 80<=slot<110 else i
                expected_g=159 if role=="producer_freeze" and 80<=slot-(1-gyro)<110 else 153 if role=="gyro_reorder" and slot-(1-gyro)==80 else i
                self.assertEqual(row["delivered_us"],nominal[i]["delivered_us"]+case["delay_us"]-10000)
                for k in ("accel_us","ax","ay","az"): self.assertEqual(row[k],nominal[expected_a][k])
                for k in ("gyro_us","gx","gy","gz"): self.assertEqual(row[k],nominal[expected_g][k])
            capture["reference_rows"][0]["x"]=99
            capture["targets"][0]["x"]=99
            capture["nominal_sensor_rows"][0]["ax"]=99
            self.assertEqual(source,before)

    def test_partial_source_fault_transform_and_error_retained(self):
        case=next(c for c in self.plan if c["role"]=="producer_freeze")
        source=synthetic_source(case)
        source["metadata"]["error"]="invented partial source"
        source["sensor_rows"]=source["sensor_rows"][:170]
        capture=motion.capture_inputs(source,case)
        self.assertEqual(len(capture["sensor_rows"]),170)
        self.assertEqual(capture["sensor_rows"][169]["gyro_us"],source["sensor_rows"][159]["gyro_us"])
        self.assertEqual(capture["metadata"]["source_error"],"invented partial source")
        with self.assertRaises(ValueError):
            motion.capture_inputs(dict(source,metadata=dict(source["metadata"],seed=999)),case)

    def test_source_constructor_failure_keeps_empty_ledger_and_cause(self):
        case=self.plan[0]
        with patch.object(motion,"FirmwareBoard",side_effect=RuntimeError("invented constructor failure")):
            result=motion.source_for_case("unused-explicit-checkout",case)
        self.assertIn("invented constructor failure",result["metadata"]["error"])
        self.assertEqual(result["targets"],[])
        self.assertEqual(result["sensor_rows"],[])
        self.assertEqual(result["reference_rows"],[])

    def test_sensor_seed_derivation_and_nominal_independent_grid(self):
        sensor=motion._Sensor(131)
        events=[]
        for now in range(2500,5120001,2500):
            if sensor.step(now,(0.,0.,9.80665),0.) and 610000<=now<=5112500:
                events.append((now,*sensor.stamps))
        self.assertEqual(len(events),902)
        for i,(t,a,g) in enumerate(events):
            slot,gyro=divmod(i,2)
            expected_a=600000+10000*slot
            self.assertEqual((t,a,g),(expected_a+10000+2500*gyro,expected_a,expected_a+(2500 if gyro else -7500)))


if __name__=="__main__":
    unittest.main()
