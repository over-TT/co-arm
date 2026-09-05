"""Laptop-only recovery preparation and frozen backup provenance regressions."""

import hashlib
import gzip
import importlib.util
import io
import json
import os
from pathlib import Path
import re
import subprocess
import shutil
import sys
import tarfile

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "scripts"
if ROOT.name == "python":
    SCRIPTS = ROOT.parent / "operations" / "scripts"
SPEC = importlib.util.spec_from_file_location("recovery_preparer", SCRIPTS / "prepare_arm_recovery.py")
assert SPEC is not None and SPEC.loader is not None
HELPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HELPER)


def calibration_bytes():
    return json.dumps({f"joint_{index}": {
        "servoId": index, "rawMin": 0, "rawMax": 1023 if index == 4 else 4095,
        "rawZero": 512 if index == 4 else 2048, "ratio": 4 if index == 1 else 1,
        "direction": 1, "speed": 400, "accel": 10,
    } for index in range(1, 5)}, separators=(",", ":")).encode()


def archive_fixture(tmp_path, data=None, checksum=None, extras=(), omit=()):
    data = calibration_bytes() if data is None else data
    digest = hashlib.sha256(data).hexdigest() if checksum is None else checksum
    rows = [
        (HELPER.CALIBRATION_MEMBER, data, tarfile.REGTYPE),
        ("metadata/SHA256SUMS", f"{digest}  {HELPER.CALIBRATION_MEMBER}\n".encode(), tarfile.REGTYPE),
        *extras,
    ]
    archive = tmp_path / "snapshot.tar.gz"
    with tarfile.open(archive, "w:gz") as tar:
        for name, payload, kind in rows:
            if name in omit:
                continue
            member = tarfile.TarInfo(name)
            member.type = kind
            member.size = len(payload)
            if kind in (tarfile.SYMTYPE, tarfile.LNKTYPE):
                member.linkname = HELPER.CALIBRATION_MEMBER
            tar.addfile(member, io.BytesIO(payload))
    sha = hashlib.sha256(archive.read_bytes()).hexdigest()
    return archive, sha


def test_fresh_archive_needs_no_old_provenance_and_never_claims_base_continuity(tmp_path):
    archive, sha = archive_fixture(tmp_path)
    output = tmp_path / "output"
    result = HELPER.prepare(archive, sha, output, create=True)
    calibration = Path(result["calibrationPath"])
    marker = json.loads(Path(result["baseReferenceMarkerPath"]).read_bytes())
    provenance = json.loads(Path(result["provenancePath"]).read_bytes())
    assert calibration.read_bytes() == calibration_bytes()
    assert marker == {
        "schema": "arm-base-reference-required.v1", "baseReferenceRequired": True,
        "reason": "RECOVERY_ARCHIVE_BASE_FRAME_UNTRUSTED",
        "sourceArchiveSha256": sha, "calibrationSha256": hashlib.sha256(calibration_bytes()).hexdigest(),
    }
    assert provenance["recovery"]["baseContinuityClaimed"] is False
    assert provenance["recovery"]["physicalBaseRezeroRequired"] is True
    assert provenance["recovery"]["sourceArchiveSha256"] == sha
    assert "priorArtifactSha256" not in provenance["recovery"]
    before = {file.name: (file.read_bytes(), file.stat().st_mtime_ns) for file in calibration.parent.iterdir()}
    assert HELPER.prepare(archive, sha, output, create=False) == result
    assert before == {file.name: (file.read_bytes(), file.stat().st_mtime_ns) for file in calibration.parent.iterdir()}


def test_archived_stale_provenance_is_history_not_current_attestation(tmp_path):
    old = b'{"artifact":{"sha256":"old"}}'
    archive, sha = archive_fixture(tmp_path, extras=[
        ("var/lib/arm-gateway/arm-joints.recovery-provenance.json", old, tarfile.REGTYPE),
    ])
    result = HELPER.prepare(archive, sha, tmp_path / "out", create=True)
    provenance = json.loads(Path(result["provenancePath"]).read_bytes())
    assert provenance["artifact"]["sha256"] == hashlib.sha256(calibration_bytes()).hexdigest()
    assert provenance["recovery"]["sourceMemberSha256"] == provenance["artifact"]["sha256"]


@pytest.mark.parametrize("name,kind", [
    ("../escaped", tarfile.REGTYPE), ("/absolute", tarfile.REGTYPE),
    ("x/../escaped", tarfile.REGTYPE), ("x\\escaped", tarfile.REGTYPE),
    ("C:/escaped", tarfile.REGTYPE), ("x:stream", tarfile.REGTYPE),
    ("x\nescaped", tarfile.REGTYPE), ("x//escaped", tarfile.REGTYPE),
    ("link", tarfile.SYMTYPE), ("link", tarfile.LNKTYPE),
    ("device", tarfile.CHRTYPE), ("fifo", tarfile.FIFOTYPE),
    ("./" + HELPER.CALIBRATION_MEMBER, tarfile.REGTYPE),
])
def test_unsafe_archive_never_materializes_output(tmp_path, name, kind):
    archive, sha = archive_fixture(tmp_path, extras=[(name, b"unsafe", kind)])
    with pytest.raises(ValueError):
        HELPER.prepare(archive, sha, tmp_path / "out", create=True)
    assert not (tmp_path / "out").exists()


@pytest.mark.parametrize("problem", ["digest", "internal", "calibration", "checksums"])
def test_archive_identity_and_member_binding_fail_closed(tmp_path, problem):
    omit = (HELPER.CALIBRATION_MEMBER,) if problem == "calibration" else (
        ("metadata/SHA256SUMS",) if problem == "checksums" else ())
    archive, sha = archive_fixture(tmp_path, checksum="0" * 64 if problem == "internal" else None, omit=omit)
    with pytest.raises(ValueError):
        HELPER.prepare(archive, "0" * 64 if problem == "digest" else sha, tmp_path / "out", create=True)
    assert not (tmp_path / "out").exists()


@pytest.mark.parametrize("joint,field,value", [
    (1, "servoId", True), (1, "rawZero", "2048"), (1, "rawMin", -40000),
    (1, "rawMax", 40000), (2, "rawZero", -1), (4, "rawMax", 4095),
    (3, "ratio", float("nan")), (3, "ratio", float("inf")),
    (3, "ratio", True), (2, "direction", 0), (1, "speed", 0),
    (1, "accel", 51), (1, "rawMin", 4095),
])
def test_invalid_calibration_never_becomes_a_recovery_input(tmp_path, joint, field, value):
    document = json.loads(calibration_bytes())
    document[f"joint_{joint}"][field] = value
    archive, sha = archive_fixture(tmp_path, data=json.dumps(document).encode())
    with pytest.raises(ValueError):
        HELPER.prepare(archive, sha, tmp_path / "out", create=True)


def test_duplicate_json_and_duplicate_checksum_rows_are_rejected(tmp_path):
    with pytest.raises(ValueError, match="Duplicate JSON"):
        HELPER.validate_calibration(b'{"joint_1":{},"joint_1":{}}')
    archive, sha = archive_fixture(tmp_path, extras=[
        ("./metadata/SHA256SUMS", b"duplicate", tarfile.REGTYPE),
    ])
    with pytest.raises(ValueError, match="Duplicate archive"):
        HELPER.prepare(archive, sha, tmp_path / "out", create=True)


@pytest.mark.parametrize("mutation", ["calibration", "provenance", "only-one"])
def test_explicit_legacy_pair_must_match_frozen_archive(tmp_path, mutation):
    archive, sha = archive_fixture(tmp_path)
    calibration = tmp_path / "calibration.json"
    provenance = tmp_path / "provenance.json"
    calibration.write_bytes(calibration_bytes() + (b"\n" if mutation == "calibration" else b""))
    provenance.write_text(json.dumps({
        "schema": "arm-joints-recovery-provenance.v1",
        "artifact": {"sha256": "0" * 64 if mutation == "provenance" else
                     hashlib.sha256(calibration_bytes()).hexdigest(), "jointCount": 4},
    }))
    with pytest.raises(ValueError):
        HELPER.prepare(archive, sha, tmp_path / "out", create=True,
                       calibration=calibration, provenance=None if mutation == "only-one" else provenance)


@pytest.mark.parametrize("filename", ["arm-joints.json", "arm-joints.provenance.json", "base-reference-required.json"])
def test_changed_or_missing_prepared_artifacts_are_not_silently_rewritten(tmp_path, filename):
    archive, sha = archive_fixture(tmp_path)
    result = HELPER.prepare(archive, sha, tmp_path / "out", create=True)
    changed = Path(result["calibrationPath"]).parent / filename
    changed.write_bytes(b"modified")
    for create in (False, True):
        with pytest.raises(ValueError, match="differs"):
            HELPER.prepare(archive, sha, tmp_path / "out", create=create)
    assert changed.read_bytes() == b"modified"
    changed.unlink()
    with pytest.raises(ValueError, match="absent"):
        HELPER.prepare(archive, sha, tmp_path / "out", create=False)


def test_verify_requires_prepare(tmp_path):
    archive, sha = archive_fixture(tmp_path)
    with pytest.raises(ValueError, match="Prepare first"):
        HELPER.prepare(archive, sha, tmp_path / "out", create=False)


def test_noncreate_permission_check_is_read_only(tmp_path, monkeypatch):
    archive, sha = archive_fixture(tmp_path)
    result = HELPER.prepare(archive, sha, tmp_path / "out", create=True)
    calls = []
    original = HELPER.protect_outputs
    def record(root, names, *, mutate):
        calls.append(mutate)
        return original(root, names, mutate=mutate)
    monkeypatch.setattr(HELPER, "protect_outputs", record)
    assert HELPER.prepare(archive, sha, tmp_path / "out", create=False) == result
    assert calls == [False]


def test_recovery_install_durably_gates_before_calibration_and_uses_prepared_digests():
    source = (SCRIPTS / "restore-arm-pi.ps1").read_text(encoding="utf-8")
    marker_install = source.index('"$stage/base-reference-required.json" /var/lib/arm-gateway/base-reference-required.json')
    calibration_install = source.index('"$stage/arm-joints.json" /var/lib/arm-gateway/arm-joints.json')
    block = source[marker_install:calibration_install]
    assert "os.fsync(source.fileno())" in block
    assert "os.fsync(directory)" in block
    assert "hashlib.sha256(source.read()).hexdigest() != sys.argv[1]" in block
    assert "$digest = [string]$preparedRows[0].sha256" in source
    assert "Recovery input changed after the prepared manifest was validated." in source


@pytest.mark.parametrize("wrong_identity", [False, True])
def test_generator_compatible_gnu_tar_full_snapshot_layout(tmp_path, wrong_identity):
    tar = shutil.which("tar")
    if os.name == "nt":
        candidate = Path(os.environ.get("ProgramFiles", r"C:\Program Files")) / "Git/usr/bin/tar.exe"
        tar = str(candidate) if candidate.is_file() else None
    if not tar:
        pytest.skip("GNU tar is unavailable")
    snapshot = tmp_path / "snapshot"
    gateway_prefix = "home/armtest/arm-gateway"
    payloads = {
        HELPER.CALIBRATION_MEMBER: calibration_bytes(),
        f"{gateway_prefix}/.robot-gateway.token": b"T" * 64 + b"\n",
        f"{gateway_prefix}/requirements-pi.txt": (SCRIPTS.parent / "requirements-pi.txt").read_bytes(),
        "etc/systemd/system/arm-gateway.service": b"[Service]\nUser=armtest\n",
        "etc/fstab": b"LABEL=test-root / ext4 defaults 0 1\n",
        "etc/netplan/direct-lan.yaml": b"network: {version: 2}\n",
        "etc/hostname": b"arm-test-host\n",
        "etc/machine-id": b"0" * 32 + b"\n",
        "boot/firmware/config.txt": b"enable_uart=1\n",
        "boot/firmware/cmdline.txt": b"root=LABEL=test-root\n",
    }
    modules = sorted((ROOT / "robot_gateway").glob("*.py"))
    assert modules
    for module in modules:
        payloads[f"{gateway_prefix}/robot_gateway/{module.name}"] = module.read_bytes()
    sums = "".join(f"{hashlib.sha256(data).hexdigest()}  {name}\n"
                   for name, data in sorted(payloads.items())).encode()
    payloads["metadata/SHA256SUMS"] = sums
    payloads["metadata/snapshot.tsv"] = (
        b"schema\tarm-pi-recovery-backup.v3\nhostname\tarm-test-host\nuser\tarmtest\n")
    payloads["metadata/canonical-source.sha256"] = "".join(
        f"{hashlib.sha256(data).hexdigest()}  {name}\n" for name, data in sorted(payloads.items())
        if name.startswith(gateway_prefix + "/") and not name.endswith(".token")).encode()
    payloads["metadata/gateway-modules.txt"] = "".join(f"{module.name}\n" for module in modules).encode()
    payloads["metadata/METADATA_SHA256SUMS"] = "".join(
        f"{hashlib.sha256(data).hexdigest()}  {name}\n" for name, data in sorted(payloads.items())
        if name.startswith("metadata/")).encode()
    for name, data in payloads.items():
        target = snapshot / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    completed = subprocess.run([
        tar, "--create", "--file", "snapshot.tar", "--numeric-owner", "--acls", "--xattrs",
        "--directory", "snapshot", ".",
    ], cwd=tmp_path, capture_output=True, text=True, check=False, timeout=20)
    assert completed.returncode == 0, completed.stderr
    archive = tmp_path / "snapshot.tar.gz"
    archive.write_bytes(gzip.compress((tmp_path / "snapshot.tar").read_bytes(), mtime=0))
    with tarfile.open(archive, "r:gz") as archive_reader:
        members = archive_reader.getmembers()
        assert members[0].name in (".", "./")
        assert any(member.name.startswith("./var/") for member in members)
    archive_hash = hashlib.sha256(archive.read_bytes()).hexdigest()
    if wrong_identity:
        with pytest.raises(ValueError, match="different hostname or user"):
            HELPER.prepare(archive, archive_hash, tmp_path / "out", create=True,
                           expected_host="another-arm", expected_user="armtest")
        assert not (tmp_path / "out").exists()
    else:
        result = HELPER.prepare(archive, archive_hash, tmp_path / "out", create=True,
                                expected_host="arm-test-host", expected_user="armtest")
        assert Path(result["calibrationPath"]).read_bytes() == calibration_bytes()
        assert len(list(Path(result["calibrationPath"]).parent.iterdir())) == 3
        assert not (tmp_path / "out" / "etc").exists()


@pytest.mark.parametrize("prior,status", [
    (None, "absent"), (b"not JSON", "malformed"), (b"[]", "malformed"),
    (json.dumps({"artifact": {"sha256": "0" * 64}}).encode(), "stale"),
    (json.dumps({"artifact": {"sha256": hashlib.sha256(calibration_bytes()).hexdigest()}}).encode(),
     "matches-frozen-bytes"),
])
def test_backup_binds_current_frozen_calibration_without_rewriting_history(tmp_path, prior, status):
    source = (SCRIPTS / "backup-arm-pi.ps1").read_text(encoding="utf-8")
    match = re.search(r"<<'PY_CALIBRATION_SNAPSHOT'\n(.*?)\nPY_CALIBRATION_SNAPSHOT", source, re.S)
    assert match is not None
    root = tmp_path / "snapshot"
    state = root / "var/lib/arm-gateway"
    state.mkdir(parents=True)
    (root / "metadata").mkdir()
    calibration = state / "arm-joints.json"
    calibration.write_bytes(calibration_bytes())
    prior_path = state / "arm-joints.recovery-provenance.json"
    if prior is not None:
        prior_path.write_bytes(prior)
    result = subprocess.run([sys.executable, "-", str(root)], input=match[1], text=True,
                            capture_output=True, check=False, timeout=10)
    assert result.returncode == 0, result.stderr
    metadata = json.loads((root / "metadata/calibration-snapshot.json").read_bytes())
    assert metadata["historicalProvenanceStatus"] == status
    assert metadata["artifact"]["sha256"] == hashlib.sha256(calibration_bytes()).hexdigest()
    assert metadata["baseContinuityClaimed"] is False
    assert metadata["physicalBaseRezeroRequiredOnRestore"] is True
    assert calibration.read_bytes() == calibration_bytes()
    assert (prior_path.read_bytes() if prior_path.exists() else None) == prior
    assert "require_file var/lib/arm-gateway/arm-joints.recovery-provenance.json" not in source
    assert "require_file var/lib/arm-gateway/recovery-manifest.json" not in source
    assert "requiredCalibrationProvenancePresent" not in source
    assert "requiredRecoveryManifestPresent" not in source
