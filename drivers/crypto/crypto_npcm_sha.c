/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_sha

#include <zephyr/kernel.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/sys/byteorder.h>

#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sha_npcm, CONFIG_CRYPTO_LOG_LEVEL);

#define SHA_BASE_ADDR   DT_REG_ADDR(DT_NODELABEL(sha0))
#define SHA             ((struct sha_reg *) SHA_BASE_ADDR)

#define NPCM_HASH_CAPS_SUPPORT  (CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)
#define NPCM_SHA_MAX_SESSION    2

#define NPCM_IN_SWAP            (DT_INST_PROP_OR(0, in_swap, 0))
#define NPCM_OUT_SWAP           (DT_INST_PROP_OR(0, out_swap, 0))

#define SHA_BLOCK_LEN      (512/8)
#define SHA1_HASH_LEN      (160/8)
#define SHA256_HASH_LEN    (256/8)
#define SHA256_MSG_LEN     8

#define SHA512_BLOCK_LEN   (1024/8)
#define SHA384_HASH_LEN    (384/8)
#define SHA512_HASH_LEN    (512/8)
#define SHA512_MSG_LEN     16


static const uint32_t sha224_iv[8] = {
	0xd89e05c1UL, 0x07d57c36UL, 0x17dd7030UL, 0x39590ef7UL,
	0x310bc0ffUL, 0x11155868UL, 0xa78ff964UL, 0xa44ffabeUL
};

typedef struct
{
	uint32_t total[2];                        /* number of bytes processed  */
	uint32_t state[8];                        /* intermediate digest state  */
	unsigned char buffer[SHA_BLOCK_LEN];   /* data block being processed */
	bool in_swap;                             /* Input buffer endianness, 1 for little endian, 0 for big endian. */
	bool out_swap;                            /* Output buffer endianness, 1 for little endian, 0 for big endian. */
} npcm_sha256_context;

typedef struct
{
	uint64_t total[2];                         /* number of bytes processed  */
	uint64_t state[8];                         /* intermediate digest state  */
	unsigned char buffer[SHA512_BLOCK_LEN]; /* data block being processed */
	bool in_swap;                              /* Input buffer endianness, 1 for little endian, 0 for big endian. */
} npcm_sha512_context;

struct npcm_session {
	union {
		npcm_sha256_context npcm_sha256;
		npcm_sha512_context npcm_sha512;
	};
	bool in_use;
	union {
		enum hash_algo algo;
	};
};

struct npcm_session npcm_sessions[NPCM_SHA_MAX_SESSION];
/* Padding block (0x80 followed by zeros) for SHA* */
static const unsigned char sha_padding[SHA512_BLOCK_LEN] = {0x80};
/* The software context is currently loaded in HW engine */
static void *ctx_in_hwip = NULL;
static bool first_round; /* The first/next round of SHA2-384/512 */

static void update_reg_bit(volatile uint8_t *reg, uint8_t mask, bool set)
{
	if (set) {
		*reg |= mask;
	} else {
		*reg &= ~mask;
	}
}

static void npcm_sha_ctx_init(void *ctx, enum hash_algo algo)
{
	if (ctx_in_hwip == ctx) {
		ctx_in_hwip = NULL;
	}

	switch (algo) {
		case CRYPTO_HASH_ALGO_SHA1:
		case CRYPTO_HASH_ALGO_SHA224:
		case CRYPTO_HASH_ALGO_SHA256:
			memset(ctx, 0, sizeof(npcm_sha256_context));
			npcm_sha256_context *sha256_ctx = ctx;
			sha256_ctx->in_swap = NPCM_IN_SWAP;
			sha256_ctx->out_swap = NPCM_OUT_SWAP;
			break;
		case CRYPTO_HASH_ALGO_SHA384:
		case CRYPTO_HASH_ALGO_SHA512:
			memset(ctx, 0, sizeof(npcm_sha512_context));
			npcm_sha512_context *sha512_ctx = ctx;
			sha512_ctx->in_swap = NPCM_IN_SWAP;
			break;
		default:
			break;
	}
}

static int npcm_sha256_starts(npcm_sha256_context *ctx, enum hash_algo algo)
{
	uint8_t i = 0;

	/* Enable SHA module */
	SHA->CSTS |= HASH_CTR_STS_SHA_EN;

	/* Set SHA type (must set before reset) 1:SHA1 0:SHA2-256 */
	if (algo == CRYPTO_HASH_ALGO_SHA1) {
		SHA->CFG |= HASH_CFG_SHA1_SHA2;
	} else {
		SHA->CFG &= ~HASH_CFG_SHA1_SHA2;
	}

	/* HW reset */
	SHA->CSTS |= HASH_CTR_STS_SHA_RST;

	/* Replace the initial digest */
	if (algo == CRYPTO_HASH_ALGO_SHA224) {
		SHA->CFG &= ~HASH_INPUT_SWAP;

		/* Wait not busy. */
		while (SHA->CSTS & HASH_CTR_STS_SHA_BUSY);

		for (i = 0; i < HASH_DIG_H_NUM; i++) {
			SHA->DH[i] = sha224_iv[i];
		}
	}

	/* Set in/out swap bit */
	update_reg_bit(&SHA->CFG, HASH_INPUT_SWAP, ctx->in_swap);
	update_reg_bit(&SHA->CFG, HASH_OUTPUT_SWAP, ctx->out_swap);

	return 0;
}

static void npcm_backup_sha256_state(npcm_sha256_context *ctx, enum hash_algo algo)
{
	size_t size;
	uint8_t i;

	if (!ctx) {
		return;
	}

	if (algo == CRYPTO_HASH_ALGO_SHA1) {
		size = SHA1_HASH_LEN;
	} else if (algo == CRYPTO_HASH_ALGO_SHA224 ||
			   algo == CRYPTO_HASH_ALGO_SHA256) {
		size = SHA256_HASH_LEN;
	} else {
		return;
	}

	/* Wait not busy. */
	while (SHA->CSTS & HASH_CTR_STS_SHA_BUSY);

	memcpy(ctx->state, (void *)SHA->DH, size);

	if (ctx->out_swap != ctx->in_swap) {
		for (i = 0; i < HASH_DIG_H_NUM; i++) {
			/* Swap the output data */
			ctx->state[i] = sys_cpu_to_be32(ctx->state[i]);
		}
	}
}

static void npcm_sha256_process(npcm_sha256_context *ctx, enum hash_algo algo,
								uint32_t *input)
{
	uint8_t i;

	/* Switch HW context if needed */
	if (ctx_in_hwip != ctx || ctx->state[0] == 0U) {
		if (ctx_in_hwip) {
			/* Backup the digest value of the previous context from the hardware. */
			npcm_backup_sha256_state(ctx_in_hwip, CRYPTO_HASH_ALGO_SHA256);

			/* Restart HW for the new context */
			if (npcm_sha256_starts(ctx, algo) != 0) {
				LOG_ERR("SHA start failed (ctx switch)");
				return;
			}
		}

		/* Load the digest value of the current context into the hardware. */
		if (ctx->state[0] != 0U) {
			/* Wait not busy. */
			while (SHA->CSTS & HASH_CTR_STS_SHA_BUSY);

			for (i = 0; i < HASH_DIG_H_NUM; i++) {
				SHA->DH[i] = ctx->state[i];
			}
		}

		ctx_in_hwip = ctx; /* Set the context that the hardware is processing */
	}

	/* Wait till SHA is not busy */
	while (SHA->CSTS & HASH_CTR_STS_SHA_BUSY);

	for (i = 0; i < (SHA_BLOCK_LEN / sizeof(uint32_t)); i++) {
		SHA->DIN = input[i];
	}
}

static int npcm_sha256_update(npcm_sha256_context *ctx, enum hash_algo algo,
							  const unsigned char *input, size_t inlen)
{
	size_t fill;
	uint32_t left;
	uint32_t tmp_buf[SHA_BLOCK_LEN / sizeof(uint32_t)];
	uint8_t i;

	if(inlen == 0) {
		return 0;
	}

	left = ctx->total[0] % SHA_BLOCK_LEN; /* 0..63 */
	fill = SHA_BLOCK_LEN - left;

	ctx->total[0] += (uint32_t)inlen;
	if(ctx->total[0] < (uint32_t)inlen) {
		ctx->total[1]++;
	}

	/* If buffer has leftover data and input can fill a block, process it */
	if(left && inlen >= fill) {
		memcpy((void *)(ctx->buffer + left), input, fill);

		memcpy(tmp_buf, ctx->buffer, SHA_BLOCK_LEN);

		/* Swap the input data if the in_swap flag is set to 1(little endian) */
		if (ctx->in_swap) {
			for (i = 0; i < (SHA_BLOCK_LEN / sizeof(uint32_t)); i++) {
				tmp_buf[i] = sys_cpu_to_be32(tmp_buf[i]);
			}
		}
		npcm_sha256_process(ctx, algo, tmp_buf);

		input += fill;
		inlen -= fill;
		left = 0;
	}

	/* Process as many full blocks as possible directly from input */
	while(inlen >= SHA_BLOCK_LEN) {
		memcpy(tmp_buf, input, SHA_BLOCK_LEN);

		/* Swap the input data if the in_swap flag is set to 1(little endian) */
		if (ctx->in_swap) {
			for (i = 0; i < (SHA_BLOCK_LEN / sizeof(uint32_t)); i++) {
				tmp_buf[i] = sys_cpu_to_be32(tmp_buf[i]);
			}
		}
		npcm_sha256_process(ctx, algo, tmp_buf);
		input += SHA_BLOCK_LEN;
		inlen -= SHA_BLOCK_LEN;
	}

	/* Buffer any remaining input data for next update */
	if(inlen > 0) {
		memcpy((void *)(ctx->buffer + left), input, inlen);
	}

	return 0;
}

static int npcm_sha256_finish(npcm_sha256_context *ctx, enum hash_algo algo,
							  unsigned char *output)
{
	uint32_t last, padn;
	uint64_t bitlen, be_bitlen;
	unsigned char msglen[SHA256_MSG_LEN];
	uint8_t i;

	/* Calculate the total length of data in bits. */
	bitlen = ((uint64_t)ctx->total[1] << 32) | ctx->total[0];
	bitlen <<= 3; /* Set bytes to bits */
	be_bitlen = sys_cpu_to_be64(bitlen);
	memcpy(msglen, &be_bitlen, sizeof(be_bitlen));

	last = ctx->total[0] % SHA_BLOCK_LEN; /* 0..63 */
	padn = (last < 56U)? (56U - last) : ((SHA_BLOCK_LEN + 56U) - last);

	npcm_sha256_update(ctx, algo, sha_padding, padn);
	npcm_sha256_update(ctx, algo, msglen, sizeof(msglen));

	/* Wait till SHA is not busy */
	while (SHA->CSTS & HASH_CTR_STS_SHA_BUSY);

	/* Get digest values */
	memcpy(output, (void *)SHA->DH, SHA256_HASH_LEN);

	/* Swap the output data if the out_swap flag is set to 1(little endian) */
	if (ctx->out_swap) {
		uint32_t *out32 = (uint32_t *)output;
		for (i = 0; i < HASH_DIG_H_NUM; i++) {
			out32[i] = sys_cpu_to_be32(out32[i]);
		}
	}

	/* Disable SHA module */
	SHA->CSTS &= ~HASH_CTR_STS_SHA_EN;

	return 0;
}

static int npcm_sha256_compute(struct hash_ctx *ctx, struct hash_pkt *pkt,
							   bool finish)
{
	int ret = -EINVAL;
	struct npcm_session *session = ctx->drv_sessn_state;
	npcm_sha256_context *sha256_ctx = &session->npcm_sha256;
	uint8_t *process_buf = pkt->in_buf;

	if (!ctx->started) {
		if (!ctx_in_hwip) { /* Engine idle */
			ret = npcm_sha256_starts(sha256_ctx, session->algo);
			if (ret != 0) {
				LOG_ERR("Could not compute the hash");
				return ret;
			}
		}
		ctx->started = true;
	}

	if (pkt->in_len != 0) {
		ret = npcm_sha256_update(sha256_ctx, session->algo, process_buf, pkt->in_len);
		if (ret != 0) {
			LOG_ERR("Could not update the hash");
			ctx->started = false;
			return ret;
		}
	}

	if (finish) {
		ctx->started = false;
		ret = npcm_sha256_finish(sha256_ctx, session->algo, pkt->out_buf);
		if (ret != 0) {
			LOG_ERR("Could not compute the hash");
			return ret;
		}
	}

	return ret;
}

static int npcm_sha512_starts(npcm_sha512_context *ctx, enum hash_algo algo)
{
	/* Enable SHA module */
	SHA->SHA512_CSTS |= SHA512_CTR_STS_SHA512_EN;

	/* Set SHA type */
	if (algo == CRYPTO_HASH_ALGO_SHA384) {
		/* SHA512_LOAD:0, SHA512_384:0, SHA512_ROUND:0,SHA512_WR:1, SHA512_RD:0 */
		SHA->SHA512_CMD = SHA512_CMD_SHA512_WR;
	} else {
		/* SHA512_LOAD:0, SHA512_384:1, SHA512_ROUND:0,SHA512_WR:1, SHA512_RD:0 */
		SHA->SHA512_CMD = (SHA512_CMD_SHA512_WR | SHA512_CMD_SHA512_384); 
	}

	/* Reset SHA hardware */
	SHA->SHA512_CSTS |= SHA512_CTR_STS_SHA512_RST;

	/* Set input swap bit */
	update_reg_bit(&SHA->SHA512_CMD, SHA512_CMD_INPUT_SWAP, ctx->in_swap);

	return 0;
}

static void npcm_read_sha512_state(uint64_t *state)
{
	uint8_t i;
	uint32_t high, low;

	/* Set SHA512_RD bit */
	SHA->SHA512_CMD = (SHA->SHA512_CMD & ~SHA512_CMD_SHA512_WR) | SHA512_CMD_SHA512_RD;

	for (i = 0 ; i < (SHA512_HASH_LEN/sizeof(uint64_t)); i++) {
		// to be able to later use the SHA512_LOAD, all 512 bits must be saved regardless of SHA2-384/512 selection.
		low = SHA->SHA512_DOUT;
		high = SHA->SHA512_DOUT;

		/* Convert to big-endian format for consistency */
		low = sys_cpu_to_be32(low);
		high = sys_cpu_to_be32(high);

		/* Combine two 32-bit values into one 64-bit value */
		state[i] = ((uint64_t)high << 32) | low;
	}

	while(SHA->SHA512_CSTS & SHA512_CTR_STS_SHA512_BUSY);
}

static void npcm_write_sha512_state(npcm_sha512_context *ctx, uint64_t *state)
{
	uint8_t i;
	uint32_t high, low;

	/* Enable SHA512_LOAD bit */
	SHA->SHA512_CMD |= SHA512_CMD_SHA512_LOAD;

	/* Write digest to SHA512_DATA_IN */
	// Write the saved digest, all 512 bits must be written regardless of SHA2-384/512 bit selection.
	for (i = 0 ; i < (SHA512_HASH_LEN/sizeof(uint64_t)); i++) {
		high = (uint32_t)(state[i] >> 32);
		low  = (uint32_t)(state[i] & 0xFFFFFFFF);

		if (!ctx->in_swap) {
			/* Convert to big-endian format for consistency */
			low = sys_cpu_to_be32(low);
			high = sys_cpu_to_be32(high);
		}

		SHA->SHA512_DIN = low;
		SHA->SHA512_DIN = high;
	}

	/* Disable SHA512_LOAD bit */
	SHA->SHA512_CMD &= ~SHA512_CMD_SHA512_LOAD;
}

static void npcm_sha512_process(npcm_sha512_context *ctx, enum hash_algo algo,
								uint32_t *input)
{
	int i;

	/* Switch HW context if needed */
	if (ctx_in_hwip != ctx) {
		if (ctx_in_hwip) {
			/* Restart crypto module for the new context */
			if (npcm_sha512_starts(ctx, algo) != 0) {
				LOG_ERR("Could not restart the hash");
				return;
			}
		}

		/* If state[] is non-zero, restore into HW */
		if (ctx->state[0] || ctx->state[1] || ctx->state[2] || ctx->state[3]) {
			/* Load the digest value of the current context into the hardware. */
			npcm_write_sha512_state(ctx, ctx->state);

			/* Next update should be treated as internal round */
			first_round = false;
		} else {
			first_round = true; /* The first round */
		}

		ctx_in_hwip = ctx; /* Set the context that the hardware is processing */
	}

	/* Set SHA to write mode */
	SHA->SHA512_CMD = (SHA->SHA512_CMD & ~SHA512_CMD_SHA512_RD) | SHA512_CMD_SHA512_WR;

	if (first_round) { 
		SHA->SHA512_CMD &= ~SHA512_CMD_SHA512_ROUND;
		first_round = false;
	} else { /* Internal round */
		SHA->SHA512_CMD |= SHA512_CMD_SHA512_ROUND;
	}

	for (i = 0; i < (SHA512_BLOCK_LEN / sizeof(uint32_t)); i++) {
		SHA->SHA512_DIN = input[i];
	}

	while(SHA->SHA512_CSTS & SHA512_CTR_STS_SHA512_BUSY);
}

static int npcm_sha512_update(npcm_sha512_context *ctx, enum hash_algo algo,
							  const unsigned char *input, size_t inlen)
{
	size_t fill;
	unsigned int left;
	bool processed = false;
	uint32_t tmp_buf[SHA512_BLOCK_LEN / sizeof(uint32_t)];
	uint8_t i;

	if(inlen == 0) {
		return 0;
	}

	left = (unsigned int) (ctx->total[0] % SHA512_BLOCK_LEN); /* 0..127 */
	fill = SHA512_BLOCK_LEN - left;

	ctx->total[0] += (uint64_t)inlen;

	if(ctx->total[0] < (uint64_t)inlen) {
		ctx->total[1]++;
	}

	/* If buffer has leftover data and input can fill a block, process it */
	if(left && inlen >= fill) {
		memcpy((void *)(ctx->buffer + left), input, fill);

		memcpy(tmp_buf, ctx->buffer, SHA512_BLOCK_LEN);

		/* Swap the input data if the in_swap flag is set to 1(little endian) */
		if (!ctx->in_swap) {
			for (i = 0; i < (SHA512_BLOCK_LEN / sizeof(uint32_t)); i++) {
				tmp_buf[i] = sys_cpu_to_be32(tmp_buf[i]);
			}
		}
		npcm_sha512_process(ctx, algo, tmp_buf);
		input += fill;
		inlen -= fill;
		left = 0;
		processed = true;
	}

	/* Process as many full blocks as possible directly from input */
	while(inlen >= SHA512_BLOCK_LEN)
	{
		memcpy(tmp_buf, input, SHA512_BLOCK_LEN);

		/* Swap the input data if the in_swap flag is set to 1(little endian) */
		if (!ctx->in_swap) {
			for (i = 0; i < (SHA512_BLOCK_LEN / sizeof(uint32_t)); i++) {
				tmp_buf[i] = sys_cpu_to_be32(tmp_buf[i]);
			}
		}
		npcm_sha512_process(ctx, algo, tmp_buf);
		input += SHA512_BLOCK_LEN;
		inlen  -= SHA512_BLOCK_LEN;
		processed = true;
	}

	/* Buffer any remaining input data for next update */
	if(inlen > 0) {
		memcpy((void *)(ctx->buffer + left), input, inlen);
	}

	/* Back up hash values */
	if (processed) {
		npcm_sha512_context *curr_ctx = (npcm_sha512_context *)ctx_in_hwip;
		npcm_read_sha512_state(curr_ctx->state);
	}

	return 0;
}

static int npcm_sha512_finish(npcm_sha512_context *ctx, enum hash_algo algo,
							  unsigned char *output)
{
	size_t last, padn;
	uint64_t high, low, be_high, be_low;
	unsigned char msglen[SHA512_MSG_LEN];
	size_t out_len = (algo == CRYPTO_HASH_ALGO_SHA512)? SHA512_HASH_LEN : SHA384_HASH_LEN;

	high = (ctx->total[0] >> 61) | (ctx->total[1] << 3);
	low = (ctx->total[0] << 3);

	be_high = sys_cpu_to_be64(high);
	be_low  = sys_cpu_to_be64(low);
	memcpy(&msglen[0], &be_high, sizeof(be_high));
	memcpy(&msglen[8], &be_low,  sizeof(be_low));

	last = (size_t)(ctx->total[0] % SHA512_BLOCK_LEN); /* 0..127 */
	padn = (last < 112U)? (112U - last) : ((SHA512_BLOCK_LEN + 112U) - last);

	npcm_sha512_update(ctx, algo, sha_padding, padn);
	npcm_sha512_update(ctx, algo, msglen, sizeof(msglen));
	memcpy(output, ctx->state, out_len);

	/* Disable SHA module */
	SHA->SHA512_CSTS &= ~SHA512_CTR_STS_SHA512_EN;

	return 0;
}

static int npcm_sha512_compute(struct hash_ctx *ctx, struct hash_pkt *pkt,
							   bool finish)
{
	int ret = -EINVAL;
	struct npcm_session *session = ctx->drv_sessn_state;
	npcm_sha512_context *sha512_ctx = &session->npcm_sha512;

	if (!ctx->started) {
		if (!ctx_in_hwip) { /* Engine idle */
			ret = npcm_sha512_starts(sha512_ctx, session->algo);
			if (ret != 0) {
				LOG_ERR("Could not compute the hash");
				return ret;
			}
		}
		ctx->started = true;
	}

	if (pkt->in_len != 0U) {
		ret = npcm_sha512_update(sha512_ctx, session->algo, pkt->in_buf, pkt->in_len);
		if (ret != 0) {
			LOG_ERR("Could not update the hash");
			ctx->started = false;
			return ret;
		}
	}

	if (finish) {
		ctx->started = false;
		ret = npcm_sha512_finish(sha512_ctx, session->algo, pkt->out_buf);
		if (ret != 0) {
			LOG_ERR("Could not compute the hash");
			return ret;
		}
	}

	return ret;
}

static int npcm_get_unused_session_index(void)
{
	uint8_t i;

	for (i = 0; i < NPCM_SHA_MAX_SESSION; i++) {
		if (!npcm_sessions[i].in_use) {
			npcm_sessions[i].in_use = true;
			return i;
		}
	}

	return -1;
}

static int npcm_hash_session_setup(const struct device *dev,
				   struct hash_ctx *ctx,
				   enum hash_algo algo)
{
	int ctx_idx;

	if (ctx->flags & ~(NPCM_HASH_CAPS_SUPPORT)) {
		LOG_ERR("Unsupported flag");
		return -EINVAL;
	}

	if ((algo != CRYPTO_HASH_ALGO_SHA1) &&
	    (algo != CRYPTO_HASH_ALGO_SHA224) &&
	    (algo != CRYPTO_HASH_ALGO_SHA256) &&
	    (algo != CRYPTO_HASH_ALGO_SHA384) &&
	    (algo != CRYPTO_HASH_ALGO_SHA512)) {
		LOG_ERR("Unsupported algo: %d", algo);
		return -EINVAL;
	}

	ctx_idx = npcm_get_unused_session_index();
	if (ctx_idx < 0) {
		LOG_ERR("No free session for now");
		return -ENOSPC;
	}

	npcm_sessions[ctx_idx].algo = algo;
	ctx->drv_sessn_state = &npcm_sessions[ctx_idx];
	ctx->started = false;

	if ((algo == CRYPTO_HASH_ALGO_SHA1) ||
		(algo == CRYPTO_HASH_ALGO_SHA224) ||
		(algo == CRYPTO_HASH_ALGO_SHA256)) {
		npcm_sha256_context *sha256_ctx =
			&npcm_sessions[ctx_idx].npcm_sha256;
		npcm_sha_ctx_init(sha256_ctx, algo);
		ctx->hash_hndlr = npcm_sha256_compute;
	} else {
		npcm_sha512_context *sha512_ctx =
			&npcm_sessions[ctx_idx].npcm_sha512;
		npcm_sha_ctx_init(sha512_ctx, algo);
		ctx->hash_hndlr = npcm_sha512_compute;
	}

	return 0;
}

/* Implementation that should never be optimized out by the compiler */
static void npcm_zeroize(void *v, size_t n)
{
	if(v == NULL) {
		return;
	}

	volatile unsigned char *p = (volatile unsigned char *)v;
	while (n--) {
		*p++ = 0;
	}
}

static int npcm_hash_session_free(const struct device *dev, struct hash_ctx *ctx)
{
	ARG_UNUSED(dev);

	struct npcm_session *session =
		(struct npcm_session *)ctx->drv_sessn_state;

	switch (session->algo) {
		case CRYPTO_HASH_ALGO_SHA1:
		case CRYPTO_HASH_ALGO_SHA224:
		case CRYPTO_HASH_ALGO_SHA256:
			npcm_zeroize(&session->npcm_sha256, sizeof(npcm_sha256_context));
			if (ctx_in_hwip == &session->npcm_sha256) {
				ctx_in_hwip = NULL;
			}
			break;
		case CRYPTO_HASH_ALGO_SHA384:
		case CRYPTO_HASH_ALGO_SHA512:
			npcm_zeroize(&session->npcm_sha512, sizeof(npcm_sha512_context));
			if (ctx_in_hwip == &session->npcm_sha512) {
				ctx_in_hwip = NULL;
			}
			break;
		default:
			return -EINVAL;
	}
	session->in_use = false;
	session->algo = 0;

	return 0;
}

static int npcm_query_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return NPCM_HASH_CAPS_SUPPORT;
}

static struct crypto_driver_api npcm_crypto_funcs = {
	.hash_begin_session = npcm_hash_session_setup,
	.hash_free_session = npcm_hash_session_free,
	.query_hw_caps = npcm_query_caps,
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL,
			POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,
			&npcm_crypto_funcs);
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "only one 'nuvoton,npcm-sha' compatible node can be supported");