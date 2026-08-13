#!/usr/bin/env python3
"""soc_manifest.py -- generate & verify Caliptra SoC authorization manifests.

Pure-stdlib port of caliptra-sw AuthManifestGenerator (ECDSA-P384 + LMS) signing
with the development fake keys, so a Zephyr build can produce a normal-boot
flash image without the caliptra codebase or a cargo environment.

Format source of truth: caliptra-sw/auth-manifest/types/src/lib.rs (layout),
auth-manifest/gen/src/generator.rs (signing flow), image/crypto/src/lib.rs (LMS).

Development/test signing only: the embedded private keys are public fake keys
from caliptra-sw/image/fake-keys. Side-channel hardening is a non-goal.
"""
import argparse
import hashlib
import hmac
import struct
import sys
from pathlib import Path

# --------------------------------------------------------------------- layout
MARKER = 0x324D5441                     # "ATM2"
PREAMBLE_SIZE = 24292
MAX_ENTRIES = 127
ENTRY_SIZE = 80
IMC_SIZE = 4 + MAX_ENTRIES * ENTRY_SIZE
MANIFEST_SIZE = PREAMBLE_SIZE + IMC_SIZE          # 34456

ECC_PUB_LEN = 96
ECC_SIG_LEN = 96
PQC_PUB_LEN = 2592                      # MLDSA87-sized slot; LMS uses first 48
PQC_SIG_LEN = 4628                      # MLDSA87-sized slot; LMS uses first 1620
LMS_PUB_LEN = 48
LMS_SIG_LEN = 1620

O_VENDOR_KEYS = 20
O_VENDOR_SIGS = O_VENDOR_KEYS + ECC_PUB_LEN + PQC_PUB_LEN         # 2708
O_OWNER_KEYS = O_VENDOR_SIGS + ECC_SIG_LEN + PQC_SIG_LEN          # 7432
O_OWNER_SIGS = O_OWNER_KEYS + ECC_PUB_LEN + PQC_PUB_LEN           # 10120
O_VENDOR_IMC_SIGS = O_OWNER_SIGS + ECC_SIG_LEN + PQC_SIG_LEN      # 14844
O_OWNER_IMC_SIGS = O_VENDOR_IMC_SIGS + ECC_SIG_LEN + PQC_SIG_LEN  # 19568
O_IMC = PREAMBLE_SIZE                                             # 24292

# vendor_signed_data_range(): version..=vendor_pub_keys
R_VENDOR_SIGNED = (8, O_VENDOR_SIGS)

FLAG_VENDOR_SIGNATURE_REQUIRED = 0x1
IMAGE_SOURCE = {"in_request": 1, "load": 2}

LMS_SHA256_N24_H5 = 10
LMS_SHA256_N24_H15 = 12
LMOTS_SHA256_N24_W4 = 7
LMOTS_SHA256_N24_W8 = 8
LMS_HEIGHT = {LMS_SHA256_N24_H5: 5, LMS_SHA256_N24_H15: 15}
LMOTS_PARAMS = {LMOTS_SHA256_N24_W4: (51, 4, 4), LMOTS_SHA256_N24_W8: (26, 8, 0)}  # p, w, ls
LMS_Q = 5                               # caliptra SUPPORTED_LMS_Q_VALUE
LMS_FULL_TREE_MAX_HEIGHT = 10           # taller trees are sparse (only leaf q real)


def swap4(b):
    """hw word format <-> natural byte order (reverse each 4-byte word)."""
    return b"".join(b[i:i + 4][::-1] for i in range(0, len(b), 4))


def sha384(b):
    return hashlib.sha384(b).digest()


# ------------------------------------------------------------- embedded keys
# caliptra-sw/image/fake-keys/src/lib.rs (Apache-2.0, development-public).
# ECC values are natural big-endian scalars (pub = x||y). LMS keys are the
# 48-byte on-wire structs: tree_type u32 BE | ots_type u32 BE | id[16] |
# seed[24] (priv) or root digest[24] (pub).
_LMS_HDR = bytes.fromhex("0000000c00000007")      # H15, W4
DEFAULT_KEYS = {
    "vendor_fw_ecc_priv": bytes.fromhex(
        "29f939ea41746499d550c6fa6368b0d761e09b4c75b2192286f9624000ea1d99"
        "ace94ba67ae89b0e3f210cf19a45b6b5"),
    "vendor_fw_ecc_pub": bytes.fromhex(
        "c69fe67f97ea3e4221a7a6036c2e070d1657327bc3f1e7c18dccb9e4ffda5c3f"
        "4db0a1c0567e097317bf448439696a07"
        "c126b9135fc825728f1cd40319109430994fe3e874a8b026be14794d27789964"
        "7735fde8328afd84cd4d4aa872d40b42"),
    "vendor_fw_lms_priv": _LMS_HDR + bytes.fromhex(
        "4908a17bcadb18291e289058d5a8e3e8"
        "4dce1e1e775253ec07bc0790cb59b2734586b03286c76974"),
    "vendor_fw_lms_pub": _LMS_HDR + bytes.fromhex(
        "4908a17bcadb18291e289058d5a8e3e8"
        "64ad3eb8be6864f17ccda38bde35edaa6c0da527645407c6"),
    "vendor_man_ecc_priv": bytes.fromhex(
        "f2ee427b4412f46f8fb020a5c23b0154b3fcb201f93c2ee2923fd577f85320bb"
        "289eb2762b6b21d35cdb3925a57d5043"),
    "vendor_man_ecc_pub": bytes.fromhex(
        "a6309750f0a05ddb956a7f862812ec4fec454e953b53dbfb9eb5414015ea7507"
        "084af93cb7fa33fe51811ad5e754232e"
        "ef5a59877a0ce0be2621d2a98bf3c5dfaf7b3d6d97f24183a4a4203858c39b86"
        "272ef548e572b9371ecf19941b8d4ea7"),
    "vendor_man_lms_priv": _LMS_HDR + bytes.fromhex(
        "7cb5369d64e4281d046e977c70d4d0a3"
        "57685bb6e9462b6bd36021ebf043b7560c581ebf7b50c514"),
    "vendor_man_lms_pub": _LMS_HDR + bytes.fromhex(
        "7cb5369d64e4281d046e977c70d4d0a3"
        "8ea4701dadf7d7000564b7d61d1c95879dd6475c9c3aae0b"),
    "owner_ecc_priv": bytes.fromhex(
        "59fdf849e39f425619342ed281d28d3d45ab32195174582cecb4e9df9cc2e991"
        "b75f88fdfa4bc6a46b88340f05dd8890"),
    "owner_ecc_pub": bytes.fromhex(
        "c6f82e2bdcf3e157a162e7f33eca35c455ea08a913811779b6f2646d92c817cd"
        "4094bd1adb215f62cf36f017012d5aeb"
        "a46745936cb5a37999b08264862b2c1c517f12c6573e1f947142291af9624bd7"
        "2733dcddce24ec5e961c00e34372ba17"),
    "owner_lms_priv": _LMS_HDR + bytes.fromhex(
        "e56d3e53a5c25beaf33a90155b273ae3"
        "65c4fbacd3aba28f77e7c51c7f39ba4d59d0b08379a9f75d"),
    "owner_lms_pub": _LMS_HDR + bytes.fromhex(
        "e56d3e53a5c25beaf33a90155b273ae3"
        "47b2156ca3641cc402c0fd957972ba56086f8f8cfa05b5bb"),
}
KEY_SIZES = {"ecc_priv": 48, "ecc_pub": 96, "lms_priv": 48, "lms_pub": 48}


# ------------------------------------------------------------------- parsing
def parse_manifest(data):
    """Parse an authorization manifest into a dict (no signature checks)."""
    if len(data) < MANIFEST_SIZE:
        raise ValueError(f"manifest too short: {len(data)} < {MANIFEST_SIZE}")
    marker, size, version, svn, flags = struct.unpack_from("<IIIII", data, 0)
    if marker != MARKER:
        raise ValueError(f"bad marker {marker:#010x} (want {MARKER:#010x})")
    if size != PREAMBLE_SIZE:
        raise ValueError(f"bad preamble size {size} (want {PREAMBLE_SIZE})")
    entry_count = struct.unpack_from("<I", data, O_IMC)[0]
    if entry_count > MAX_ENTRIES:
        raise ValueError(f"entry_count {entry_count} > {MAX_ENTRIES}")
    entries = []
    for i in range(entry_count):
        off = O_IMC + 4 + i * ENTRY_SIZE
        fw_id, comp, cls, eflags, llo, lhi, slo, shi = struct.unpack_from(
            "<IIIIIIII", data, off)
        entries.append({
            "fw_id": fw_id, "component_id": comp, "classification": cls,
            "flags": eflags, "load": lhi << 32 | llo, "staging": shi << 32 | slo,
            "digest": data[off + 32:off + 80],
        })
    return {
        "marker": marker, "size": size, "version": version, "svn": svn,
        "flags": flags, "entry_count": entry_count, "entries": entries,
        "vendor_ecc_pub": swap4(data[O_VENDOR_KEYS:O_VENDOR_KEYS + ECC_PUB_LEN]),
        "vendor_lms_pub": data[O_VENDOR_KEYS + ECC_PUB_LEN:
                               O_VENDOR_KEYS + ECC_PUB_LEN + LMS_PUB_LEN],
        "owner_ecc_pub": swap4(data[O_OWNER_KEYS:O_OWNER_KEYS + ECC_PUB_LEN]),
        "owner_lms_pub": data[O_OWNER_KEYS + ECC_PUB_LEN:
                              O_OWNER_KEYS + ECC_PUB_LEN + LMS_PUB_LEN],
    }


# ------------------------------------------------------------- ECDSA P-384
# Development-key signing only: no side-channel hardening required or attempted.
_P = 2**384 - 2**128 - 2**96 + 2**32 - 1
_N = int("ffffffffffffffffffffffffffffffffffffffffffffffffc7634d81f4372ddf"
         "581a0db248b0a77aecec196accc52973", 16)
_B = int("b3312fa7e23ee7e4988e056be3f82d19181d9c6efe8141120314088f5013875a"
         "c656398d8a2ed19d2a85c8edd3ec2aef", 16)
_GX = int("aa87ca22be8b05378eb1c71ef320ad746e1d3b628ba79b9859f741e082542a38"
          "5502f25dbf55296c3a545e3872760ab7", 16)
_GY = int("3617de4a96262c6f5d9e98bf9292dc29f8f41dbd289a147ce9da3113b5f0b8c0"
          "0a60b1ce1d7e819d7a431d7c90ea0e5f", 16)


def _ec_add(p1, p2):
    if p1 is None:
        return p2
    if p2 is None:
        return p1
    x1, y1 = p1
    x2, y2 = p2
    if x1 == x2:
        if (y1 + y2) % _P == 0:
            return None
        lam = (3 * x1 * x1 - 3) * pow(2 * y1, -1, _P) % _P
    else:
        lam = (y2 - y1) * pow(x2 - x1, -1, _P) % _P
    x3 = (lam * lam - x1 - x2) % _P
    return (x3, (lam * (x1 - x3) - y1) % _P)


def _ec_mul(k, pt):
    r = None
    while k:
        if k & 1:
            r = _ec_add(r, pt)
        pt = _ec_add(pt, pt)
        k >>= 1
    return r


def _rfc6979_k(x, digest):
    """Deterministic nonce per RFC 6979 with HMAC-SHA384 (qlen == hlen == 384)."""
    xb = x.to_bytes(48, "big")
    hb = (int.from_bytes(digest, "big") % _N).to_bytes(48, "big")
    v, k = b"\x01" * 48, b"\x00" * 48
    k = hmac.new(k, v + b"\x00" + xb + hb, hashlib.sha384).digest()
    v = hmac.new(k, v, hashlib.sha384).digest()
    k = hmac.new(k, v + b"\x01" + xb + hb, hashlib.sha384).digest()
    v = hmac.new(k, v, hashlib.sha384).digest()
    while True:
        v = hmac.new(k, v, hashlib.sha384).digest()
        cand = int.from_bytes(v, "big")
        if 1 <= cand < _N:
            return cand
        k = hmac.new(k, v + b"\x00", hashlib.sha384).digest()
        v = hmac.new(k, v, hashlib.sha384).digest()


def ecc_pub_from_priv(priv):
    x, y = _ec_mul(int.from_bytes(priv, "big"), (_GX, _GY))
    return x.to_bytes(48, "big") + y.to_bytes(48, "big")


def ecdsa_sign(digest, priv):
    """Sign a 48-byte digest; returns the 96-byte hw-format wire signature."""
    z = int.from_bytes(digest, "big")
    x = int.from_bytes(priv, "big")
    k = _rfc6979_k(x, digest)
    r = _ec_mul(k, (_GX, _GY))[0] % _N
    s = pow(k, -1, _N) * (z + r * x) % _N
    if r == 0 or s == 0:                # cannot happen for a fixed test key; guard anyway
        raise ValueError("degenerate ECDSA signature")
    return swap4(r.to_bytes(48, "big")) + swap4(s.to_bytes(48, "big"))


def ecdsa_verify(digest, pub, sig_wire):
    """Verify hw-format wire sig against a natural x||y pubkey. Never raises."""
    r = int.from_bytes(swap4(sig_wire[:48]), "big")
    s = int.from_bytes(swap4(sig_wire[48:96]), "big")
    if not (1 <= r < _N and 1 <= s < _N):
        return False
    xy = (int.from_bytes(pub[:48], "big"), int.from_bytes(pub[48:96], "big"))
    if (xy[1] ** 2 - (xy[0] ** 3 - 3 * xy[0] + _B)) % _P != 0:
        return False
    w = pow(s, -1, _N)
    z = int.from_bytes(digest, "big")
    pt = _ec_add(_ec_mul(z * w % _N, (_GX, _GY)), _ec_mul(r * w % _N, xy))
    return pt is not None and pt[0] % _N == r


# ---------------------------------------------------------------------- LMS
# Port of caliptra-sw/image/crypto/src/lib.rs: LMS-SHA256/N24 with caliptra's
# fixed q=5 and sparse tree for heights > 10 (only leaf 5 has a real LM-OTS
# key; other leaves hash from 24 zero bytes). The fake-key public keys were
# generated with the same shortcut, so this must match it exactly.
_D_PBLC = (0x8080).to_bytes(2, "big")
_D_MESG = (0x8181).to_bytes(2, "big")
_D_LEAF = (0x8282).to_bytes(2, "big")
_D_INTR = (0x8383).to_bytes(2, "big")


def _h24(*parts):
    h = hashlib.sha256()
    for p in parts:
        h.update(p)
    return h.digest()[:24]


def _coef(s, i, w):
    shift = 8 - (w * (i % (8 // w)) + w)
    return (s[(i * w) // 8] >> shift) & ((1 << w) - 1)


def _lms_key_fields(key48):
    tree_t = int.from_bytes(key48[0:4], "big")
    ots_t = int.from_bytes(key48[4:8], "big")
    if tree_t not in LMS_HEIGHT:
        raise ValueError(f"unsupported LMS tree type {tree_t}")
    if ots_t not in LMOTS_PARAMS:
        raise ValueError(f"unsupported LM-OTS type {ots_t}")
    return tree_t, ots_t, key48[8:24], key48[24:48]     # id, seed-or-digest


def _lmots_pub_k(kid, q, p, w, seed):
    qb = q.to_bytes(4, "big")
    y = bytearray()
    for i in range(p):
        ib = i.to_bytes(2, "big")
        tmp = _h24(kid, qb, ib, b"\xff", seed)          # x_i (RFC 8554 App. A)
        for j in range((1 << w) - 1):
            tmp = _h24(kid, qb, ib, bytes([j]), tmp)
        y += tmp
    return _h24(kid, qb, _D_PBLC, bytes(y))


def _lms_tree(kid, height, p, w, seed, q=None):
    """Merkle root and (when q is given) the auth path for leaf q.

    Port of generate_lms_pubkey_helper (RFC 8554 App. C stack algorithm with
    caliptra's sparse-leaf shortcut). Do not reorder the path captures.
    """
    want_path = q is not None
    target = (((1 << height) + q) ^ 1) if want_path else 1
    path, stack = [], []
    for i in range(1 << height):
        if height <= LMS_FULL_TREE_MAX_HEIGHT or i == LMS_Q:
            k = _lmots_pub_k(kid, i, p, w, seed)
        else:
            k = bytes(24)                               # sparse leaf
        r = i + (1 << height)
        k = _h24(kid, r.to_bytes(4, "big"), _D_LEAF, k)
        j, cur = i, (1 << height) + i
        while j % 2 == 1:
            r >>= 1
            j >>= 1
            left = stack.pop()
            if want_path and cur == target + 1:
                path.append(left)
                target = (target >> 1) ^ 1
            if want_path and cur == target:
                path.append(k)
                target = (target >> 1) ^ 1
            cur >>= 1
            k = _h24(kid, r.to_bytes(4, "big"), _D_INTR, left, k)
        stack.append(k)
    return stack.pop(), path


def lms_pub_from_priv(priv48):
    tree_t, ots_t, kid, seed = _lms_key_fields(priv48)
    p, w, _ = LMOTS_PARAMS[ots_t]
    root, _ = _lms_tree(kid, LMS_HEIGHT[tree_t], p, w, seed)
    return priv48[0:24] + root


def _ots_digits(kid, q, msg, nonce, p, w, ls):
    """Message digits + checksum digits for the OTS chains."""
    qb = q.to_bytes(4, "big")
    big_q = _h24(kid, qb, _D_MESG, nonce, msg)
    n_coef = 24 * 8 // w
    chksum = sum((1 << w) - 1 - _coef(big_q, i, w) for i in range(n_coef)) << ls
    digits = big_q + chksum.to_bytes(2, "big")
    return [_coef(digits, i, w) for i in range(p)]


def lms_sign(msg, priv48):
    """Sign a message (a 48-byte digest) at fixed leaf q=5; 1620-byte wire sig."""
    tree_t, ots_t, kid, seed = _lms_key_fields(priv48)
    height = LMS_HEIGHT[tree_t]
    p, w, ls = LMOTS_PARAMS[ots_t]
    if LMS_Q >= (1 << height):
        raise ValueError("q out of range for tree height")
    qb = LMS_Q.to_bytes(4, "big")
    # nonce content is opaque to the verifier; deterministic for reproducibility
    nonce = hmac.new(kid + seed, msg, hashlib.sha256).digest()[:24]
    y = bytearray()
    for i, a in enumerate(_ots_digits(kid, LMS_Q, msg, nonce, p, w, ls)):
        ib = i.to_bytes(2, "big")
        tmp = _h24(kid, qb, ib, b"\xff", seed)          # x_i
        for j in range(a):
            tmp = _h24(kid, qb, ib, bytes([j]), tmp)
        y += tmp
    _, path = _lms_tree(kid, height, p, w, seed, q=LMS_Q)
    return (qb + ots_t.to_bytes(4, "big") + nonce + bytes(y)
            + tree_t.to_bytes(4, "big") + b"".join(path))


def lms_verify(msg, pub48, sig):
    """Candidate-root LMS verification (RFC 8554 5.4.2). Never raises."""
    try:
        tree_t, ots_t, kid, root = _lms_key_fields(pub48)
        height = LMS_HEIGHT[tree_t]
        p, w, ls = LMOTS_PARAMS[ots_t]
        if len(sig) < 8 + 24 + 24 * p + 4 + 24 * height:
            return False
        q = int.from_bytes(sig[0:4], "big")
        if q >= (1 << height):
            return False
        if int.from_bytes(sig[4:8], "big") != ots_t:
            return False
        nonce = sig[8:32]
        y_off = 32
        if int.from_bytes(sig[y_off + 24 * p:y_off + 24 * p + 4], "big") != tree_t:
            return False
        path_off = y_off + 24 * p + 4
        qb = sig[0:4]
        z = bytearray()
        for i, a in enumerate(_ots_digits(kid, q, msg, nonce, p, w, ls)):
            ib = i.to_bytes(2, "big")
            tmp = sig[y_off + 24 * i:y_off + 24 * (i + 1)]
            for j in range(a, (1 << w) - 1):
                tmp = _h24(kid, qb, ib, bytes([j]), tmp)
            z += tmp
        kc = _h24(kid, qb, _D_PBLC, bytes(z))
        node = (1 << height) + q
        tmp = _h24(kid, node.to_bytes(4, "big"), _D_LEAF, kc)
        for lvl in range(height):
            e = sig[path_off + 24 * lvl:path_off + 24 * (lvl + 1)]
            half = (node >> 1).to_bytes(4, "big")
            tmp = _h24(kid, half, _D_INTR, e, tmp) if node % 2 else \
                  _h24(kid, half, _D_INTR, tmp, e)
            node >>= 1
        return tmp == root
    except (ValueError, IndexError, TypeError):
        return False


# ------------------------------------------------------------- generation
# Rust-flow defaults for the NPCM500 MCU-runtime entry
# (NTC_caliptra-mcu-sw/builder/src/caliptra.rs get_mcu_manifest_metadata):
# exec_bit must be >= 2 (bits 0/1 reserved by Caliptra); staging is the FPGA
# DMA-reachable SRAM offset.
MCU_ENTRY_DEFAULTS = {
    "fw_id": 2, "component_id": 2, "source": "in_request",
    "exec_bit": 2, "load": 0, "staging": 0xA8C00000,
}


def entry_flags(source, exec_bit=0, ignore_auth=False):
    """ImageMetadataFlags: image_source[1:0], ignore_auth_check[2], exec_bit[14:8]."""
    if source not in IMAGE_SOURCE:
        raise ValueError(f"image source must be one of {sorted(IMAGE_SOURCE)}")
    if not 0 <= exec_bit <= 0x7F:
        raise ValueError("exec_bit out of range (7 bits)")
    return IMAGE_SOURCE[source] | (ignore_auth << 2) | (exec_bit << 8)


def make_entry(fw_id, digest, *, component_id, classification=0, flags,
               load=0, staging=0):
    if len(digest) != 48:
        raise ValueError("digest must be 48 bytes (SHA-384)")
    for name, val, bits in (("fw_id", fw_id, 32), ("component_id", component_id, 32),
                            ("classification", classification, 32), ("flags", flags, 32),
                            ("load", load, 64), ("staging", staging, 64)):
        if not 0 <= val < (1 << bits):
            raise ValueError(f"{name} out of range ({bits}-bit unsigned): {val}")
    return struct.pack("<IIIIIIII", fw_id, component_id, classification, flags,
                       load & 0xFFFFFFFF, load >> 32,
                       staging & 0xFFFFFFFF, staging >> 32) + digest


_UNUSED_ENTRY = struct.pack("<II", 0xFFFFFFFF, 0xFFFFFFFF) + bytes(ENTRY_SIZE - 8)


def _pack_imc(entries):
    if len(entries) > MAX_ENTRIES:
        raise ValueError(f"{len(entries)} entries > max {MAX_ENTRIES}")
    body = b"".join(entries) + _UNUSED_ENTRY * (MAX_ENTRIES - len(entries))
    return struct.pack("<I", len(entries)) + body


def _sign_pair(digest, ecc_priv, lms_priv):
    """One 4724-byte signature block: ECC sig + zero-padded LMS sig."""
    lms = lms_sign(digest, lms_priv)
    return ecdsa_sign(digest, ecc_priv) + lms + bytes(PQC_SIG_LEN - len(lms))


def generate(entries, svn=0, version=1, keys=None):
    """Build and sign an authorization manifest (AuthManifestGenerator port)."""
    k = DEFAULT_KEYS if keys is None else keys
    imc = _pack_imc(entries)
    pre = bytearray(PREAMBLE_SIZE)
    struct.pack_into("<IIIII", pre, 0, MARKER, PREAMBLE_SIZE, version, svn,
                     FLAG_VENDOR_SIGNATURE_REQUIRED)
    # vendor manifest pubkeys, then vendor-signed range signed by vendor FW keys
    pre[O_VENDOR_KEYS:O_VENDOR_KEYS + 96] = swap4(k["vendor_man_ecc_pub"])
    pre[O_VENDOR_KEYS + 96:O_VENDOR_KEYS + 96 + 48] = k["vendor_man_lms_pub"]
    d = sha384(bytes(pre[R_VENDOR_SIGNED[0]:R_VENDOR_SIGNED[1]]))
    pre[O_VENDOR_SIGS:O_VENDOR_SIGS + 4724] = _sign_pair(
        d, k["vendor_fw_ecc_priv"], k["vendor_fw_lms_priv"])
    # owner pubkeys, signed by owner keys
    pre[O_OWNER_KEYS:O_OWNER_KEYS + 96] = swap4(k["owner_ecc_pub"])
    pre[O_OWNER_KEYS + 96:O_OWNER_KEYS + 96 + 48] = k["owner_lms_pub"]
    d = sha384(bytes(pre[O_OWNER_KEYS:O_OWNER_SIGS]))
    pre[O_OWNER_SIGS:O_OWNER_SIGS + 4724] = _sign_pair(
        d, k["owner_ecc_priv"], k["owner_lms_priv"])
    # IMC signed by vendor manifest keys (VENDOR_SIGNATURE_REQUIRED) + owner keys
    d = sha384(imc)
    pre[O_VENDOR_IMC_SIGS:O_VENDOR_IMC_SIGS + 4724] = _sign_pair(
        d, k["vendor_man_ecc_priv"], k["vendor_man_lms_priv"])
    pre[O_OWNER_IMC_SIGS:O_OWNER_IMC_SIGS + 4724] = _sign_pair(
        d, k["owner_ecc_priv"], k["owner_lms_priv"])
    return bytes(pre) + imc


def verify_manifest(data, keys=None):
    """Check all 8 signatures. Vendor-FW and owner sigs verify against the
    configured keys (the runtime checks them against the FW bundle / fuses);
    IMC sigs verify against the pubkeys embedded in the manifest itself,
    mirroring caliptra runtime SET_AUTH_MANIFEST."""
    k = DEFAULT_KEYS if keys is None else keys
    m = parse_manifest(data)             # raises on structural problems
    d_vendor = sha384(data[R_VENDOR_SIGNED[0]:R_VENDOR_SIGNED[1]])
    d_owner = sha384(data[O_OWNER_KEYS:O_OWNER_SIGS])
    d_imc = sha384(data[O_IMC:MANIFEST_SIZE])

    def block(off):
        return data[off:off + 96], data[off + 96:off + 96 + LMS_SIG_LEN]

    v_ecc, v_lms = block(O_VENDOR_SIGS)
    o_ecc, o_lms = block(O_OWNER_SIGS)
    vi_ecc, vi_lms = block(O_VENDOR_IMC_SIGS)
    oi_ecc, oi_lms = block(O_OWNER_IMC_SIGS)
    return {
        "vendor_pub_keys_ecc": ecdsa_verify(d_vendor, k["vendor_fw_ecc_pub"], v_ecc),
        "vendor_pub_keys_lms": lms_verify(d_vendor, k["vendor_fw_lms_pub"], v_lms),
        "owner_pub_keys_ecc": ecdsa_verify(d_owner, k["owner_ecc_pub"], o_ecc),
        "owner_pub_keys_lms": lms_verify(d_owner, k["owner_lms_pub"], o_lms),
        "vendor_imc_ecc": ecdsa_verify(d_imc, m["vendor_ecc_pub"], vi_ecc),
        "vendor_imc_lms": lms_verify(d_imc, m["vendor_lms_pub"], vi_lms),
        "owner_imc_ecc": ecdsa_verify(d_imc, m["owner_ecc_pub"], oi_ecc),
        "owner_imc_lms": lms_verify(d_imc, m["owner_lms_pub"], oi_lms),
    }


# ----------------------------------------------------------------------- CLI
def load_key_dir(path):
    """DEFAULT_KEYS overlaid with raw binary files named <key>.bin.

    LMS public keys are cross-checked against their private seeds when both
    are supplied; a lone private key regenerates its public automatically.
    """
    keys = dict(DEFAULT_KEYS)
    overridden = set()
    for name in DEFAULT_KEYS:
        f = Path(path) / (name + ".bin")
        if not f.is_file():
            continue
        data = f.read_bytes()
        want = KEY_SIZES["_".join(name.split("_")[-2:])]   # e.g. "ecc_pub"
        if len(data) != want:
            raise ValueError(f"{f}: expected {want} bytes, got {len(data)}")
        keys[name] = data
        overridden.add(name)
    for role in ("vendor_fw", "vendor_man", "owner"):
        priv, pub = role + "_lms_priv", role + "_lms_pub"
        if priv in overridden:
            derived = lms_pub_from_priv(keys[priv])
            if pub in overridden and keys[pub] != derived:
                raise ValueError(f"{pub} does not match {priv}")
            keys[pub] = derived
        epriv, epub = role + "_ecc_priv", role + "_ecc_pub"
        if epriv in overridden:
            derived = ecc_pub_from_priv(keys[epriv])
            if epub in overridden and keys[epub] != derived:
                raise ValueError(f"{epub} does not match {epriv}")
            keys[epub] = derived
    return keys


def parse_image_spec(spec, defaults=None):
    """--soc-image / --mcu-runtime entry spec: k=v pairs joined by commas.

    Keys: fw_id (required), path (required), load (required unless a default
    provides it), staging (default: =load), exec (default 0), comp (default:
    =fw_id), class (default 0), source (in_request|load, default load).
    """
    kv = dict(defaults or {})
    for part in spec.split(","):
        if "=" not in part:
            raise ValueError(f"malformed spec item {part!r} (want key=value)")
        key, val = part.split("=", 1)
        kv[key.strip()] = val.strip()
    for req in ("fw_id", "path"):
        if req not in kv:
            raise ValueError(f"image spec missing required {req}= : {spec!r}")
    num = lambda s: int(str(s), 0)
    if "load" not in kv and "staging" not in kv:
        raise ValueError(f"image spec missing load= (or staging=): {spec!r}")
    load = num(kv.get("load", 0))
    digest = sha384(Path(kv["path"]).read_bytes())
    return make_entry(
        num(kv["fw_id"]), digest,
        component_id=num(kv.get("comp", kv["fw_id"])),
        classification=num(kv.get("class", 0)),
        flags=entry_flags(kv.get("source", "load"), num(kv.get("exec", 0))),
        load=load, staging=num(kv.get("staging", load)))


def _mcu_defaults():
    d = MCU_ENTRY_DEFAULTS
    return {"fw_id": d["fw_id"], "comp": d["component_id"], "source": d["source"],
            "exec": d["exec_bit"], "load": d["load"], "staging": d["staging"]}


def cmd_generate(a):
    entries = []
    if a.mcu_runtime:
        entries.append(parse_image_spec(f"path={a.mcu_runtime}",
                                        defaults=_mcu_defaults()))
    for spec in a.soc_image or []:
        entries.append(parse_image_spec(spec))
    if not entries:
        sys.exit("error: nothing to sign (give --mcu-runtime and/or --soc-image)")
    keys = load_key_dir(a.key_dir) if a.key_dir else None
    data = generate(entries, svn=a.svn, version=a.version, keys=keys)
    checks = verify_manifest(data, keys=keys)
    if not all(checks.values()):
        bad = [n for n, ok in sorted(checks.items()) if not ok]
        sys.exit(f"error: generated manifest fails self-verification: {bad}\n"
                 "       a --key-dir public key does not match its private signing key")
    Path(a.out).write_bytes(data)
    print(f"Wrote {a.out}: {len(entries)} image(s), svn={a.svn}, "
          f"{len(data)} bytes, all signatures self-verified: True")


def cmd_verify(a):
    data = Path(a.file).read_bytes()
    keys = load_key_dir(a.key_dir) if a.key_dir else None
    m = parse_manifest(data)
    checks = verify_manifest(data, keys=keys)
    for e in m["entries"]:
        print(f"  fw_id={e['fw_id']} flags={e['flags']:#x} "
              f"load={e['load']:#x} staging={e['staging']:#x} "
              f"sha384={e['digest'].hex()[:32]}...")
    good = sum(checks.values())
    for name, ok in sorted(checks.items()):
        print(f"  {name}: {'OK' if ok else 'FAIL'}")
    print(f"{good}/{len(checks)} signatures OK")
    sys.exit(0 if good == len(checks) else 1)


def cmd_selftest(a):
    for role in ("vendor_fw", "vendor_man", "owner"):
        assert ecc_pub_from_priv(DEFAULT_KEYS[role + "_ecc_priv"]) == \
            DEFAULT_KEYS[role + "_ecc_pub"], f"ECC pub KAT failed: {role}"
        assert lms_pub_from_priv(DEFAULT_KEYS[role + "_lms_priv"]) == \
            DEFAULT_KEYS[role + "_lms_pub"], f"LMS pub KAT failed: {role}"
    entry = make_entry(2, sha384(b"fake runtime"), component_id=2,
                       flags=entry_flags("in_request", 2), staging=0xA8C00000)
    data = generate([entry])
    checks = verify_manifest(data)
    assert all(checks.values()), checks
    print("selftest OK")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("generate", help="build and sign a SoC manifest")
    g.add_argument("--mcu-runtime",
                   help="MCU runtime blob; adds the fw_id=2 in_request entry "
                        "with NPCM500 defaults (staging=0xA8C00000, exec=2)")
    g.add_argument("--soc-image", action="append", metavar="SPEC",
                   help="repeatable; fw_id=N,path=F,load=ADDR"
                        "[,staging=ADDR][,exec=N][,comp=N][,class=N]"
                        "[,source=load|in_request]")
    g.add_argument("--svn", type=lambda x: int(x, 0), default=0)
    g.add_argument("--version", type=lambda x: int(x, 0), default=1)
    g.add_argument("--key-dir", help="directory of <key>.bin overrides")
    g.add_argument("--out", default="soc_manifest.bin")
    g.set_defaults(func=cmd_generate)

    v = sub.add_parser("verify", help="parse a manifest and check all signatures")
    v.add_argument("file")
    v.add_argument("--key-dir")
    v.set_defaults(func=cmd_verify)

    sub.add_parser("selftest", help="key KATs + sign/verify round-trip") \
       .set_defaults(func=cmd_selftest)

    a = ap.parse_args(argv)
    try:
        a.func(a)
    except (ValueError, OSError, struct.error) as e:
        sys.exit(f"error: {e}")


if __name__ == "__main__":
    main()
