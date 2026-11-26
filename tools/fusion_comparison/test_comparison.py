"""Public synthetic contract checks; no native run or qualification generation."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(HERE),str(HERE.parent)]
import core
import score


class ComparisonTests(unittest.TestCase):
    def setUp(self):
        self.spec=core.load(HERE/"development.json")

    def validate(self,spec):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"spec.json"
            path.write_text(json.dumps(spec))
            return core.spec_at(path)

    def test_explicit_new_public_seed_and_wheel1_schedule(self):
        self.spec["seeds"]=[10001,10002,10003]
        self.spec["surfaces"]["wheel1_slip"]=self.spec["surfaces"].pop("wheel0_slip")
        self.assertEqual(self.validate(self.spec),self.spec)

    def test_invalid_grid_duplicate_seed_and_heldout_rejected(self):
        for change in (lambda s:s["seeds"].append(s["seeds"][0]),
                       lambda s:s["courses"]["position"]["targets"].append([801,[0,0,0]]),
                       lambda s:s.update(held_out={}),
                       lambda s:s["surfaces"].update(**{"../escape":[]})):
            with self.subTest(change=change):
                spec=copy.deepcopy(self.spec);change(spec)
                with self.assertRaises(ValueError):self.validate(spec)

    def test_original_dispersion_six_entries(self):
        self.assertEqual(len(core.nominal_sd()),6)

    def test_dispersion_survives_git_line_ending_conversion(self):
        raw=(HERE/"nominal_dispersion.json").read_bytes().replace(b"\r\n",b"\n")
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)
            for data in (raw,raw.replace(b"\n",b"\r\n")):
                (p/"nominal_dispersion.json").write_bytes(data)
                with patch.object(core,"HERE",p):self.assertEqual(len(core.nominal_sd()),6)

    def test_changed_materialized_spec_rejected_before_trace_reads(self):
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)
            (p/"run.json").write_text(json.dumps(dict(complete=True,materialized_spec_sha256="wrong")))
            with self.assertRaisesRegex(ValueError,"Materialized spec"):
                score.read_arm(p,self.spec,"candidate-fused")

    def fake_arm(self,root,spec,arm):
        data={}
        files={}
        for key in core.keys(spec):
            value=.08 if arm.endswith("fused") and key[2] != "nominal" else .1
            data[key]=dict(primary=value,health=None,dispatches=100,scores={k:value for k in
                ("tracking_end_position_m","tracking_end_heading_rad","tracking_heading_rmse_rad",
                 "est_position_rmse_m","est_linear_velocity_rmse_m_s","held_sample_age_max_ms")})
            files[core.case_name(key)]={f:"same" for f in core.FILES}
        return data,[],files,dict(firmware=arm.split('-')[0],simulator="same")

    def comparison(self,read=None):
        with patch.object(score,"read_arm",side_effect=read or self.fake_arm),patch.object(score,"sha",return_value="sha"):
            return score.compare({a:Path(a) for a in core.ARMS},self.spec)

    def test_full_paired_pass(self):
        self.assertEqual(self.comparison()["outcome"],"PASS")

    def test_nominal_cannot_replace_disturbed_primary(self):
        with self.assertRaises(ValueError):
            score.compare({a:Path(a) for a in core.ARMS},self.spec,"nominal")

    def test_empty_disturbance_cannot_establish_full_coverage(self):
        self.spec["surfaces"]["wheel0_slip"]=[]
        self.assertEqual(self.comparison()["tracking_gate"],"UNASSESSABLE")

    def test_missing_arm_is_incomplete_not_invented_parity_failure(self):
        def missing(root,spec,arm):
            if arm=="candidate-default":raise FileNotFoundError("missing")
            return self.fake_arm(root,spec,arm)
        result=self.comparison(missing)
        self.assertEqual(result["outcome"],"INCOMPLETE")
        self.assertFalse(any(result["parity"].values()))

    def test_failure_survives_incomplete_other_arm(self):
        def mixed(root,spec,arm):
            if arm=="candidate-default":raise FileNotFoundError("missing")
            data,bad,files,run=self.fake_arm(root,spec,arm)
            if arm=="candidate-fused":
                for row in data.values():row["primary"]=.3
            return data,bad,files,run
        result=self.comparison(mixed)
        self.assertEqual(result["outcome"],"FAIL")
        self.assertEqual(result["tracking_gate"],"UNASSESSABLE")
        self.assertTrue(result["failed_tracking_groups"])

    def test_subset_not_full_acceptance(self):
        self.spec["seeds"]=[131]
        self.assertEqual(self.comparison()["tracking_gate"],"UNASSESSABLE")

    def test_known_three_seed_subset_not_full_acceptance(self):
        self.spec["seeds"]=[131,277,419]
        self.assertEqual(self.comparison()["tracking_gate"],"UNASSESSABLE")

    def test_known_missing_supplementary_surface_keeps_scope_loss(self):
        del self.spec["surfaces"]["common_slip"]
        self.assertEqual(self.comparison()["tracking_gate"],"UNASSESSABLE")

    def test_separate_materialized_three_seed_contract_can_pass(self):
        self.spec["seeds"]=[10101,10102,10103]
        del self.spec["surfaces"]["common_slip"]
        self.assertEqual(self.comparison()["outcome"],"PASS")

    def test_three_seed_majority_rejects_one_favorable_pair(self):
        self.spec["seeds"]=[131,277,419]
        def one_win(root,spec,arm):
            data,bad,files,run=self.fake_arm(root,spec,arm)
            if arm=="candidate-fused":
                for key,row in data.items():
                    if key[2]=="wheel0_slip":row["primary"]=0 if key[-1]==131 else .11
            return data,bad,files,run
        result=self.comparison(one_win)
        self.assertTrue(result["failed_tracking_groups"])

    def test_changed_default_trace_is_product_parity_failure(self):
        def changed(root,spec,arm):
            data,bad,files,run=self.fake_arm(root,spec,arm)
            if arm=="candidate-default":next(iter(files.values()))["firmware.csv"]="changed"
            return data,bad,files,run
        self.assertEqual(self.comparison(changed)["outcome"],"FAIL")

    def test_zero_baseline_unassessable(self):
        def zero(root,spec,arm):
            data,bad,files,run=self.fake_arm(root,spec,arm)
            for row in data.values():row["primary"]=0
            return data,bad,files,run
        self.assertEqual(self.comparison(zero)["outcome"],"INCOMPLETE")

    def test_unhealthy_state_is_failure_even_with_valid_flag(self):
        def unhealthy(root,spec,arm):
            data,bad,files,run=self.fake_arm(root,spec,arm)
            if arm=="candidate-fused":next(iter(data.values()))["health"]=dict(invalid=0,faults=0,unhealthy=1)
            return data,bad,files,run
        self.assertEqual(self.comparison(unhealthy)["outcome"],"FAIL")


if __name__=="__main__":unittest.main()
