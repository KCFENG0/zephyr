"""Tests for soc_manifest.py, on a manifest it generates over a fixed runtime."""
import hashlib
from pathlib import Path

import soc_manifest as sm

# Any fixed bytes will do: only their SHA-384 ends up in the manifest.
RUNTIME = bytes(range(256)) * 34
IMC = 24292                              # image metadata collection offset


def _entries():
    d = sm.MCU_ENTRY_DEFAULTS
    return [sm.make_entry(
        d["fw_id"], sm.sha384(RUNTIME), component_id=d["component_id"],
        flags=sm.entry_flags(d["source"], d["exec_bit"]),
        load=d["load"], staging=d["staging"])]


G = sm.generate(_entries(), svn=0, version=1)


def test_parse_manifest():
    m = sm.parse_manifest(G)
    assert m["marker"] == 0x324D5441
    assert m["size"] == 24292
    assert m["version"] == 1
    assert m["svn"] == 0
    assert m["flags"] == 1
    assert m["entry_count"] == 1
    e = m["entries"][0]
    assert e["fw_id"] == 2
    assert e["component_id"] == 2
    assert e["classification"] == 0
    assert e["flags"] == 0x201          # image_source=in_request(1), exec_bit=2
    assert e["load"] == 0
    assert e["staging"] == 0xA8C00000
    assert e["digest"] == hashlib.sha384(RUNTIME).digest()


def test_embedded_keys_land_in_the_preamble():
    assert G[20:116] == sm.swap4(sm.DEFAULT_KEYS["vendor_man_ecc_pub"])
    assert G[116:164] == sm.DEFAULT_KEYS["vendor_man_lms_pub"]
    assert G[7432:7528] == sm.swap4(sm.DEFAULT_KEYS["owner_ecc_pub"])
    assert G[7528:7576] == sm.DEFAULT_KEYS["owner_lms_pub"]


def test_parse_rejects_bad_marker():
    import pytest
    with pytest.raises(ValueError):
        sm.parse_manifest(b"\x00" * len(G))


def _digests():
    d_vendor = sm.sha384(G[8:2708])       # vendor_signed_data_range
    d_owner = sm.sha384(G[7432:10120])    # owner_pub_keys block
    d_imc = sm.sha384(G[IMC:])            # image metadata collection
    return d_vendor, d_owner, d_imc


def test_ecdsa_verifies_manifest_signatures():
    d_vendor, d_owner, d_imc = _digests()
    k = sm.DEFAULT_KEYS
    assert sm.ecdsa_verify(d_vendor, k["vendor_fw_ecc_pub"], G[2708:2804])
    assert sm.ecdsa_verify(d_owner, k["owner_ecc_pub"], G[10120:10216])
    assert sm.ecdsa_verify(d_imc, k["vendor_man_ecc_pub"], G[14844:14940])
    assert sm.ecdsa_verify(d_imc, k["owner_ecc_pub"], G[19568:19664])
    # negative: wrong digest must not verify
    assert not sm.ecdsa_verify(d_imc, k["vendor_fw_ecc_pub"], G[2708:2804])


def test_ecdsa_sign_roundtrip_and_pub_derivation():
    k = sm.DEFAULT_KEYS
    for role in ("vendor_fw", "vendor_man", "owner"):
        assert sm.ecc_pub_from_priv(k[role + "_ecc_priv"]) == k[role + "_ecc_pub"]
    d = sm.sha384(b"test message")
    sig = sm.ecdsa_sign(d, k["vendor_man_ecc_priv"])
    assert len(sig) == 96
    assert sm.ecdsa_verify(d, k["vendor_man_ecc_pub"], sig)
    assert not sm.ecdsa_verify(sm.sha384(b"other"), k["vendor_man_ecc_pub"], sig)
    # RFC 6979: deterministic
    assert sig == sm.ecdsa_sign(d, k["vendor_man_ecc_priv"])


def test_lms_pubkey_regeneration_kat():
    k = sm.DEFAULT_KEYS
    for role in ("vendor_fw", "vendor_man", "owner"):
        assert sm.lms_pub_from_priv(k[role + "_lms_priv"]) == k[role + "_lms_pub"]


def test_lms_verifies_manifest_signatures():
    d_vendor, d_owner, d_imc = _digests()
    k = sm.DEFAULT_KEYS
    # LMS sigs sit after the 96-byte ECC sig in each 4724-byte signature block
    assert sm.lms_verify(d_vendor, k["vendor_fw_lms_pub"], G[2804:2804 + 1620])
    assert sm.lms_verify(d_owner, k["owner_lms_pub"], G[10216:10216 + 1620])
    assert sm.lms_verify(d_imc, k["vendor_man_lms_pub"], G[14940:14940 + 1620])
    assert sm.lms_verify(d_imc, k["owner_lms_pub"], G[19664:19664 + 1620])
    assert not sm.lms_verify(d_imc, k["vendor_fw_lms_pub"], G[2804:2804 + 1620])


def test_lms_sign_roundtrip_and_tree_path_matches_manifest():
    k = sm.DEFAULT_KEYS
    d = sm.sha384(b"lms test message")
    sig = sm.lms_sign(d, k["vendor_man_lms_priv"])
    assert len(sig) == 1620
    assert sm.lms_verify(d, k["vendor_man_lms_pub"], sig)
    assert not sm.lms_verify(sm.sha384(b"other"), k["vendor_man_lms_pub"], sig)
    # the merkle auth path depends only on the key (fixed q=5): must be
    # byte-identical to the path in the manifest's vendor IMC signature
    assert sig[1260:1620] == G[14940 + 1260:14940 + 1620]
    assert sig[0:8] == G[14940:14940 + 8]        # q=5 BE, ots_type=7 BE


def test_lms_verify_never_raises_on_malformed_input():
    d = sm.sha384(b"x")
    pub = sm.DEFAULT_KEYS["owner_lms_pub"]
    assert sm.lms_verify(d, None, b"x" * 1620) is False
    assert sm.lms_verify(d, pub, None) is False
    assert sm.lms_verify(d, pub, b"") is False
    assert sm.lms_verify(d, b"\x00" * 48, b"\x00" * 1620) is False


def test_generate_layout():
    assert len(G) == sm.manifest_size(80)
    # signature slot padding beyond the 1620-byte LMS sig stays zero
    assert G[2804 + 1620:7432] == bytes(7432 - 2804 - 1620)


def test_manifest_verifies():
    checks = sm.verify_manifest(G)
    assert checks and all(checks.values()), checks
    assert len(checks) == 8              # 4 ECC + 4 LMS


def test_verify_detects_tamper():
    bad = bytearray(G)
    bad[24296] ^= 1                      # flip a bit in entry 0 fw_id
    checks = sm.verify_manifest(bytes(bad))
    assert not checks["vendor_imc_ecc"]
    assert not checks["owner_imc_ecc"]


def test_generate_rejects_too_many_entries():
    import pytest
    e = _entries()[0]
    with pytest.raises(ValueError):
        sm.generate([e] * 128, max_entries=127)
    with pytest.raises(ValueError):
        sm.generate([e] * 81)


def test_a1_runtime_manifest_size():
    """127 entries for the pre-#3824 A1 runtime; checked at the wrong size,
    the IMC signature breaks or the manifest is too short."""
    import pytest
    out = sm.generate(_entries(), max_entries=127)
    assert len(out) == sm.manifest_size(127) == 34456
    assert all(sm.verify_manifest(out, max_entries=127).values())
    assert not sm.verify_manifest(out, max_entries=80)["vendor_imc_ecc"]
    with pytest.raises(ValueError):
        sm.verify_manifest(G, max_entries=127)
    with pytest.raises(ValueError):
        sm.manifest_size(100)


import subprocess
import sys


def _run(*args):
    return subprocess.run([sys.executable, "soc_manifest.py", *args],
                          capture_output=True, text=True,
                          cwd=Path(__file__).parent)


def test_cli_verify(tmp_path):
    (tmp_path / "m.bin").write_bytes(G)
    r = _run("verify", str(tmp_path / "m.bin"))
    assert r.returncode == 0, r.stderr
    assert "8/8 signatures OK" in r.stdout


def test_cli_rejects_malformed_spec():
    r = _run("generate", "--soc-image", "path=soc_image_0.bin",
             "--out", "/dev/null")                               # fw_id missing
    assert r.returncode != 0
    assert "fw_id" in r.stderr


def test_cli_requires_at_least_one_entry(tmp_path):
    r = _run("generate", "--out", str(tmp_path / "m.bin"))
    assert r.returncode != 0


def test_cli_generate_fails_on_key_mismatch(tmp_path):
    # pub-only override that disagrees with its default private key: generation
    # must abort (non-zero) and write no artifact, not exit 0 with a broken file
    (tmp_path / "vendor_fw_ecc_pub.bin").write_bytes(sm.DEFAULT_KEYS["owner_ecc_pub"])
    (tmp_path / "rt.bin").write_bytes(RUNTIME)
    out = tmp_path / "m.bin"
    r = _run("generate", "--mcu-runtime", str(tmp_path / "rt.bin"),
             "--key-dir", str(tmp_path), "--out", str(out))
    assert r.returncode != 0
    assert "self-verification" in r.stderr
    assert not out.exists()


def test_cli_selftest():
    r = _run("selftest")
    assert r.returncode == 0, r.stderr
    assert "selftest OK" in r.stdout


def test_key_dir_override(tmp_path):
    # overriding a public key with a wrong-but-valid-size file must break verify
    (tmp_path / "vendor_fw_ecc_pub.bin").write_bytes(
        sm.DEFAULT_KEYS["owner_ecc_pub"])
    keys = sm.load_key_dir(tmp_path)
    assert keys["vendor_fw_ecc_pub"] == sm.DEFAULT_KEYS["owner_ecc_pub"]
    assert keys["owner_lms_priv"] == sm.DEFAULT_KEYS["owner_lms_priv"]  # untouched
    checks = sm.verify_manifest(G, keys=keys)
    assert not checks["vendor_pub_keys_ecc"]
    import pytest
    (tmp_path / "owner_ecc_priv.bin").write_bytes(b"short")
    with pytest.raises(ValueError):
        sm.load_key_dir(tmp_path)


def test_key_dir_derives_pub_from_priv_and_detects_mismatch(tmp_path):
    # priv-only override: pub is derived from the priv, overriding the default
    (tmp_path / "owner_ecc_priv.bin").write_bytes(
        sm.DEFAULT_KEYS["vendor_fw_ecc_priv"])
    (tmp_path / "owner_lms_priv.bin").write_bytes(
        sm.DEFAULT_KEYS["vendor_fw_lms_priv"])
    keys = sm.load_key_dir(tmp_path)
    assert keys["owner_ecc_pub"] == sm.DEFAULT_KEYS["vendor_fw_ecc_pub"]
    assert keys["owner_lms_pub"] == sm.DEFAULT_KEYS["vendor_fw_lms_pub"]
    # priv+pub override that disagree: must raise
    import pytest
    (tmp_path / "owner_ecc_pub.bin").write_bytes(sm.DEFAULT_KEYS["owner_ecc_pub"])
    with pytest.raises(ValueError):
        sm.load_key_dir(tmp_path)
