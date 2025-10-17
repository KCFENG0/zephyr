/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Sample to illustrate the usage of crypto APIs. The sample plaintext
 * and ciphertexts used for crosschecking are from TinyCrypt.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/crypto/crypto.h>

#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#ifdef CONFIG_CRYPTO_TINYCRYPT_SHIM
#define CRYPTO_DRV_NAME CONFIG_CRYPTO_TINYCRYPT_SHIM_DRV_NAME
#elif CONFIG_CRYPTO_MBEDTLS_SHIM
#define CRYPTO_DRV_NAME CONFIG_CRYPTO_MBEDTLS_SHIM_DRV_NAME
#elif DT_HAS_COMPAT_STATUS_OKAY(st_stm32_cryp)
#define CRYPTO_DEV_COMPAT st_stm32_cryp
#elif DT_HAS_COMPAT_STATUS_OKAY(st_stm32_aes)
#define CRYPTO_DEV_COMPAT st_stm32_aes
#elif DT_HAS_COMPAT_STATUS_OKAY(nxp_mcux_dcp)
#define CRYPTO_DEV_COMPAT nxp_mcux_dcp
#elif CONFIG_CRYPTO_NRF_ECB
#define CRYPTO_DEV_COMPAT nordic_nrf_ecb
#elif DT_HAS_COMPAT_STATUS_OKAY(renesas_smartbond_crypto)
#define CRYPTO_DEV_COMPAT renesas_smartbond_crypto
#elif CONFIG_CRYPTO_NPCM_SHA
#define CRYPTO_DEV_COMPAT nuvoton_npcm_sha
#else
#error "You need to enable one crypto device"
#endif

const static uint8_t key[16] __aligned(32) = {
	0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88,
	0x09, 0xcf, 0x4f, 0x3c
};

static uint8_t plaintext[64] = {
	0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11,
	0x73, 0x93, 0x17, 0x2a, 0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
	0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51, 0x30, 0xc8, 0x1c, 0x46,
	0xa3, 0x5c, 0xe4, 0x11, 0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
	0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b,
	0xe6, 0x6c, 0x37, 0x10
};

uint32_t cap_flags;

static void print_buffer_comparison(const uint8_t *wanted_result,
				    uint8_t *result, size_t length)
{
	int i, j;

	printk("Was waiting for: \n");

	for (i = 0, j = 1; i < length; i++, j++) {
		printk("0x%02x ", wanted_result[i]);

		if (j == 10) {
			printk("\n");
			j = 0;
		}
	}

	printk("\nBut got:\n");

	for (i = 0, j = 1; i < length; i++, j++) {
		printk("0x%02x ", result[i]);

		if (j == 10) {
			printk("\n");
			j = 0;
		}
	}

	printk("\n");
}

int validate_hw_compatibility(const struct device *dev)
{
	uint32_t flags = 0U;

	flags = crypto_query_hwcaps(dev);
	if ((flags & CAP_RAW_KEY) == 0U) {
		LOG_INF("Please provision the key separately "
			"as the module doesnt support a raw key");
		return -1;
	}

	if ((flags & CAP_SYNC_OPS) == 0U) {
		LOG_ERR("The app assumes sync semantics. "
		  "Please rewrite the app accordingly before proceeding");
		return -1;
	}

	if ((flags & CAP_SEPARATE_IO_BUFS) == 0U) {
		LOG_ERR("The app assumes distinct IO buffers. "
		"Please rewrite the app accordingly before proceeding");
		return -1;
	}

	cap_flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

	return 0;

}

void ecb_mode(const struct device *dev)
{
	/* from FIPS-197 test vectors */
	const uint8_t ecb_key[16] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
	};
	uint8_t ecb_plaintext[16] = {
		0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
		0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
	};
	uint8_t ecb_ciphertext[16] = {
		0x69, 0xC4, 0xE0, 0xD8, 0x6A, 0x7B, 0x04, 0x30,
		0xD8, 0xCD, 0xB7, 0x80, 0x70, 0xB4, 0xC5, 0x5A
	};

	uint8_t encrypted[16] = {0};
	uint8_t decrypted[16] = {0};
	struct cipher_ctx ini = {
		.keylen = sizeof(ecb_key),
		.key.bit_stream = ecb_key,
		.flags = cap_flags,
	};
	struct cipher_pkt encrypt = {
		.in_buf = ecb_plaintext,
		.in_len = sizeof(ecb_plaintext),
		.out_buf_max = sizeof(encrypted),
		.out_buf = encrypted,
	};
	struct cipher_pkt decrypt = {
		.in_buf = encrypt.out_buf,
		.in_len = sizeof(encrypted),
		.out_buf = decrypted,
		.out_buf_max = sizeof(decrypted),
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_ECB,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return;
	}

	if (cipher_block_op(&ini, &encrypt)) {
		LOG_ERR("ECB mode ENCRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	if (memcmp(encrypt.out_buf, ecb_ciphertext, sizeof(ecb_ciphertext))) {
		LOG_ERR("ECB mode ENCRYPT - Mismatch between expected and "
			    "returned cipher text");
		print_buffer_comparison(ecb_ciphertext, encrypt.out_buf,
					sizeof(ecb_ciphertext));
		goto out;
	}

	LOG_INF("ECB mode ENCRYPT - Match");
	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_ECB,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		return;
	}

	if (cipher_block_op(&ini, &decrypt)) {
		LOG_ERR("ECB mode DECRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, ecb_plaintext, sizeof(ecb_plaintext))) {
		LOG_ERR("ECB mode DECRYPT - Mismatch between plaintext and "
			    "decrypted cipher text");
		print_buffer_comparison(ecb_plaintext, decrypt.out_buf,
					sizeof(ecb_plaintext));
		goto out;
	}

	LOG_INF("ECB mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

static const uint8_t cbc_ciphertext[80] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
	0x0c, 0x0d, 0x0e, 0x0f, 0x76, 0x49, 0xab, 0xac, 0x81, 0x19, 0xb2, 0x46,
	0xce, 0xe9, 0x8e, 0x9b, 0x12, 0xe9, 0x19, 0x7d, 0x50, 0x86, 0xcb, 0x9b,
	0x50, 0x72, 0x19, 0xee, 0x95, 0xdb, 0x11, 0x3a, 0x91, 0x76, 0x78, 0xb2,
	0x73, 0xbe, 0xd6, 0xb8, 0xe3, 0xc1, 0x74, 0x3b, 0x71, 0x16, 0xe6, 0x9e,
	0x22, 0x22, 0x95, 0x16, 0x3f, 0xf1, 0xca, 0xa1, 0x68, 0x1f, 0xac, 0x09,
	0x12, 0x0e, 0xca, 0x30, 0x75, 0x86, 0xe1, 0xa7
};

void cbc_mode(const struct device *dev)
{
	uint8_t encrypted[80] = {0};
	uint8_t decrypted[64] = {0};
	struct cipher_ctx ini = {
		.keylen = sizeof(key),
		.key.bit_stream = key,
		.flags = cap_flags,
	};
	struct cipher_pkt encrypt = {
		.in_buf = plaintext,
		.in_len = sizeof(plaintext),
		.out_buf_max = sizeof(encrypted),
		.out_buf = encrypted,
	};
	struct cipher_pkt decrypt = {
		.in_buf = encrypt.out_buf,
		.in_len = sizeof(encrypted),
		.out_buf = decrypted,
		.out_buf_max = sizeof(decrypted),
	};

	static uint8_t iv[16] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CBC,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return;
	}

	if (cipher_cbc_op(&ini, &encrypt, iv)) {
		LOG_ERR("CBC mode ENCRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	if (memcmp(encrypt.out_buf, cbc_ciphertext, sizeof(cbc_ciphertext))) {
		LOG_ERR("CBC mode ENCRYPT - Mismatch between expected and "
			    "returned cipher text");
		print_buffer_comparison(cbc_ciphertext, encrypt.out_buf,
					sizeof(cbc_ciphertext));
		goto out;
	}

	LOG_INF("CBC mode ENCRYPT - Match");
	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CBC,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		return;
	}

	/* TinyCrypt keeps IV at the start of encrypted buffer */
	if (cipher_cbc_op(&ini, &decrypt, encrypted)) {
		LOG_ERR("CBC mode DECRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, plaintext, sizeof(plaintext))) {
		LOG_ERR("CBC mode DECRYPT - Mismatch between plaintext and "
			    "decrypted cipher text");
		print_buffer_comparison(plaintext, decrypt.out_buf,
					sizeof(plaintext));
		goto out;
	}

	LOG_INF("CBC mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

static const uint8_t ctr_ciphertext[64] = {
	0x22, 0xe5, 0x2f, 0xb1, 0x77, 0xd8, 0x65, 0xb2,
	0xf7, 0xc6, 0xb5, 0x12, 0x69, 0x2d, 0x11, 0x4d,
	0xed, 0x6c, 0x1c, 0x72, 0x25, 0xda, 0xf6, 0xa2,
	0xaa, 0xd9, 0xd3, 0xda, 0x2d, 0xba, 0x21, 0x68,
	0x35, 0xc0, 0xaf, 0x6b, 0x6f, 0x40, 0xc3, 0xc6,
	0xef, 0xc5, 0x85, 0xd0, 0x90, 0x2c, 0xc2, 0x63,
	0x12, 0x2b, 0xc5, 0x8e, 0x72, 0xde, 0x5c, 0xa2,
	0xa3, 0x5c, 0x85, 0x3a, 0xb9, 0x2c, 0x6, 0xbb
};

void ctr_mode(const struct device *dev)
{
	uint8_t encrypted[64] = {0};
	uint8_t decrypted[64] = {0};
	struct cipher_ctx ini = {
		.keylen = sizeof(key),
		.key.bit_stream = key,
		.flags = cap_flags,
		/*  ivlen + ctrlen = keylen , so ctrlen is 128 - 96 = 32 bits */
		.mode_params.ctr_info.ctr_len = 32,
	};
	struct cipher_pkt encrypt = {
		.in_buf = plaintext,
		.in_len = sizeof(plaintext),
		.out_buf_max = sizeof(encrypted),
		.out_buf = encrypted,
	};
	struct cipher_pkt decrypt = {
		.in_buf = encrypted,
		.in_len = sizeof(encrypted),
		.out_buf = decrypted,
		.out_buf_max = sizeof(decrypted),
	};
	uint8_t iv[12] = {
		0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,
		0xf8, 0xf9, 0xfa, 0xfb
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CTR,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return;
	}

	if (cipher_ctr_op(&ini, &encrypt, iv)) {
		LOG_ERR("CTR mode ENCRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	if (memcmp(encrypt.out_buf, ctr_ciphertext, sizeof(ctr_ciphertext))) {
		LOG_ERR("CTR mode ENCRYPT - Mismatch between expected "
			    "and returned cipher text");
		print_buffer_comparison(ctr_ciphertext, encrypt.out_buf,
					sizeof(ctr_ciphertext));
		goto out;
	}

	LOG_INF("CTR mode ENCRYPT - Match");
	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CTR,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		return;
	}

	if (cipher_ctr_op(&ini, &decrypt, iv)) {
		LOG_ERR("CTR mode DECRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, plaintext, sizeof(plaintext))) {
		LOG_ERR("CTR mode DECRYPT - Mismatch between plaintext "
			    "and decrypted cipher text");
		print_buffer_comparison(plaintext,
					decrypt.out_buf, sizeof(plaintext));
		goto out;
	}

	LOG_INF("CTR mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

/* RFC 3610 test vector #1 */
const static uint8_t ccm_key[16] = {
	0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb,
	0xcc, 0xcd, 0xce, 0xcf
};
static uint8_t ccm_nonce[13] = {
	0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x00, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4,
	0xa5
};
static uint8_t ccm_hdr[8] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
};
static uint8_t ccm_data[23] = {
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
	0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e
};
static const uint8_t ccm_expected[31] = {
	0x58, 0x8c, 0x97, 0x9a, 0x61, 0xc6, 0x63, 0xd2, 0xf0, 0x66, 0xd0, 0xc2,
	0xc0, 0xf9, 0x89, 0x80, 0x6d, 0x5f, 0x6b, 0x61, 0xda, 0xc3, 0x84, 0x17,
	0xe8, 0xd1, 0x2c, 0xfd, 0xf9, 0x26, 0xe0
};

void ccm_mode(const struct device *dev)
{
	uint8_t encrypted[50];
	uint8_t decrypted[25];
	struct cipher_ctx ini = {
		.keylen = sizeof(ccm_key),
		.key.bit_stream = ccm_key,
		.mode_params.ccm_info = {
			.nonce_len = sizeof(ccm_nonce),
			.tag_len = 8,
		},
		.flags = cap_flags,
	};
	struct cipher_pkt encrypt = {
		.in_buf = ccm_data,
		.in_len = sizeof(ccm_data),
		.out_buf_max = sizeof(encrypted),
		.out_buf = encrypted,
	};
	struct cipher_aead_pkt ccm_op = {
		.ad = ccm_hdr,
		.ad_len = sizeof(ccm_hdr),
		.pkt = &encrypt,
		/* TinyCrypt always puts the tag at the end of the ciphered
		 * text, but other library such as mbedtls might be more
		 * flexible and can take a different buffer for it.  So to
		 * make sure test passes on all backends: enforcing the tag
		 * buffer to be after the ciphered text.
		 */
		.tag = encrypted + sizeof(ccm_data),
	};
	struct cipher_pkt decrypt = {
		.in_buf = encrypted,
		.in_len = sizeof(ccm_data),
		.out_buf = decrypted,
		.out_buf_max = sizeof(decrypted),
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CCM,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return;
	}

	ccm_op.pkt = &encrypt;
	if (cipher_ccm_op(&ini, &ccm_op, ccm_nonce)) {
		LOG_ERR("CCM mode ENCRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	if (memcmp(encrypt.out_buf, ccm_expected, sizeof(ccm_expected))) {
		LOG_ERR("CCM mode ENCRYPT - Mismatch between expected "
			    "and returned cipher text");
		print_buffer_comparison(ccm_expected,
					encrypt.out_buf, sizeof(ccm_expected));
		goto out;
	}

	LOG_INF("CCM mode ENCRYPT - Match");
	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CCM,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		return;
	}

	ccm_op.pkt = &decrypt;
	if (cipher_ccm_op(&ini, &ccm_op, ccm_nonce)) {
		LOG_ERR("CCM mode DECRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, ccm_data, sizeof(ccm_data))) {
		LOG_ERR("CCM mode DECRYPT - Mismatch between plaintext "
			"and decrypted cipher text");
		print_buffer_comparison(ccm_data,
					decrypt.out_buf, sizeof(ccm_data));
		goto out;
	}

	LOG_INF("CCM mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

/*  MACsec GCM-AES test vector 2.4.1 */
const static uint8_t gcm_key[16] = {
	0x07, 0x1b, 0x11, 0x3b, 0x0c, 0xa7, 0x43, 0xfe, 0xcc, 0xcf, 0x3d, 0x05,
	0x1f, 0x73, 0x73, 0x82
};
static uint8_t gcm_nonce[12] = {
	0xf0, 0x76, 0x1e, 0x8d, 0xcd, 0x3d, 0x00, 0x01, 0x76, 0xd4, 0x57, 0xed
};
static uint8_t gcm_hdr[20] = {
	0xe2, 0x01, 0x06, 0xd7, 0xcd, 0x0d, 0xf0, 0x76, 0x1e, 0x8d, 0xcd, 0x3d,
	0x88, 0xe5, 0x4c, 0x2a, 0x76, 0xd4, 0x57, 0xed
};
static uint8_t gcm_data[42] = {
	0x08, 0x00, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
	0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24,
	0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30,
	0x31, 0x32, 0x33, 0x34, 0x00, 0x04
};
static const uint8_t gcm_expected[58] = {
	0x13, 0xb4, 0xc7, 0x2b, 0x38, 0x9d, 0xc5, 0x01, 0x8e, 0x72, 0xa1, 0x71,
	0xdd, 0x85, 0xa5, 0xd3, 0x75, 0x22, 0x74, 0xd3, 0xa0, 0x19, 0xfb, 0xca,
	0xed, 0x09, 0xa4, 0x25, 0xcd, 0x9b, 0x2e, 0x1c, 0x9b, 0x72, 0xee, 0xe7,
	0xc9, 0xde, 0x7d, 0x52, 0xb3, 0xf3,
	0xd6, 0xa5, 0x28, 0x4f, 0x4a, 0x6d, 0x3f, 0xe2, 0x2a, 0x5d, 0x6c, 0x2b,
	0x96, 0x04, 0x94, 0xc3
};

void gcm_mode(const struct device *dev)
{
	uint8_t encrypted[60] = {0};
	uint8_t decrypted[44] = {0};
	struct cipher_ctx ini = {
		.keylen = sizeof(gcm_key),
		.key.bit_stream = gcm_key,
		.mode_params.gcm_info = {
			.nonce_len = sizeof(gcm_nonce),
			.tag_len = 16,
		},
		.flags = cap_flags,
	};
	struct cipher_pkt encrypt = {
		.in_buf = gcm_data,
		.in_len = sizeof(gcm_data),
		.out_buf_max = sizeof(encrypted),
		.out_buf = encrypted,
	};
	struct cipher_aead_pkt gcm_op = {
		.ad = gcm_hdr,
		.ad_len = sizeof(gcm_hdr),
		.pkt = &encrypt,
		/* TinyCrypt always puts the tag at the end of the ciphered
		 * text, but other library such as mbedtls might be more
		 * flexible and can take a different buffer for it.  So to
		 * make sure test passes on all backends: enforcing the tag
		 * buffer to be after the ciphered text.
		 */
		.tag = encrypted + sizeof(gcm_data),
	};
	struct cipher_pkt decrypt = {
		.in_buf = encrypted,
		.in_len = sizeof(gcm_data),
		.out_buf = decrypted,
		.out_buf_max = sizeof(decrypted),
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_GCM,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return;
	}

	gcm_op.pkt = &encrypt;
	if (cipher_gcm_op(&ini, &gcm_op, gcm_nonce)) {
		LOG_ERR("GCM mode ENCRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	if (memcmp(encrypt.out_buf, gcm_expected, sizeof(gcm_expected))) {
		LOG_ERR("GCM mode ENCRYPT - Mismatch between expected "
			    "and returned cipher text");
		print_buffer_comparison(gcm_expected,
					encrypt.out_buf, sizeof(gcm_expected));
		goto out;
	}

	LOG_INF("GCM mode ENCRYPT - Match");
	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_GCM,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		return;
	}

	gcm_op.pkt = &decrypt;
	if (cipher_gcm_op(&ini, &gcm_op, gcm_nonce)) {
		LOG_ERR("GCM mode DECRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, gcm_data, sizeof(gcm_data))) {
		LOG_ERR("GCM mode DECRYPT - Mismatch between plaintext "
			"and decrypted cipher text");
		print_buffer_comparison(gcm_data,
					decrypt.out_buf, sizeof(gcm_data));
		goto out;
	}

	LOG_INF("GCM mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

struct mode_test {
	const char *mode;
	void (*mode_func)(const struct device *dev);
};

#if CONFIG_CRYPTO_NPCM_SHA
/* Following test are part of mbedTLS */
#define SHA160_HASH_LENGTH      (160/8)
#define SHA224_HASH_LENGTH      (224/8)
#define SHA256_HASH_LENGTH      (256/8)
#define SHA384_HASH_LENGTH      (384/8)
#define SHA512_HASH_LENGTH      (512/8)

uint8_t sha1_test1[] = {'a', 'b', 'c'};
uint8_t sha1_test2[] = { //size is 56
		'a', 'b', 'c', 'd', 'b', 'c', 'd', 'e',
		'c', 'd', 'e', 'f', 'd', 'e', 'f', 'g',
		'e', 'f', 'g', 'h', 'f', 'g', 'h', 'i',
		'g', 'h', 'i', 'j', 'h', 'i', 'j', 'k',
		'i', 'j', 'k', 'l', 'j', 'k', 'l', 'm',
		'k', 'l', 'm', 'n', 'l', 'm', 'n', 'o',
		'm', 'n', 'o', 'p', 'n', 'o', 'p', 'q'};
uint8_t sha1_test3[] = { //size is 64
		'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
		'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
		'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
		'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
		'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
		'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
		'0', '1', '2', '3'};
uint8_t sha1_test4[] = { //size is 94
		0x30, 0x5c, 0x30, 0x2c, 0x02, 0x01, 0x00, 0x30, 0x09, 0x06, 0x05, 0x2b,
		0x0e, 0x03, 0x02, 0x1a, 0x05, 0x00, 0x30, 0x06, 0x06, 0x04, 0x67, 0x2a,
		0x01, 0x0c, 0x04, 0x14, 0xe1, 0xb6, 0x93, 0xfe, 0x33, 0x43, 0xc1, 0x20,
		0x5d, 0x4b, 0xaa, 0xb8, 0x63, 0xfb, 0xcf, 0x6c, 0x46, 0x1e, 0x88, 0x04,
		0x30, 0x2c, 0x02, 0x01, 0x00, 0x30, 0x09, 0x06, 0x05, 0x2b, 0x0e, 0x03,
		0x02, 0x1a, 0x05, 0x00, 0x30, 0x06, 0x06, 0x04, 0x67, 0x2a, 0x01, 0x0c,
		0x04, 0x14, 0x13, 0xc1, 0x0c, 0xfc, 0xc8, 0x92, 0xd7, 0xde, 0x07, 0x1c,
		0x40, 0xde, 0x4f, 0xcd, 0x07, 0x5b, 0x68, 0x20, 0x5a, 0x6c};

uint8_t sha1_results[4][SHA160_HASH_LENGTH] = {
		{0xA9, 0x99, 0x3E, 0x36,
		 0x47, 0x06, 0x81, 0x6A,
		 0xBA, 0x3E, 0x25, 0x71,
		 0x78, 0x50, 0xC2, 0x6C,
		 0x9C, 0xD0, 0xD8, 0x9D},
		{0x84, 0x98, 0x3E, 0x44,
		 0x1C, 0x3B, 0xD2, 0x6E,
		 0xBA, 0xAE, 0x4A, 0xA1,
		 0xF9, 0x51, 0x29, 0xE5,
		 0xE5, 0x46, 0x70, 0xF1},
		{0xCF, 0x08, 0x00, 0xF7,
		 0x64, 0x4A, 0xCE, 0x3C,
		 0xB4, 0xC3, 0xFA, 0x33,
		 0x38, 0x8D, 0x3B, 0xA0,
		 0xEA, 0x3C, 0x8B, 0x6E},
		{0xc9, 0x84, 0x45, 0xc8,
		 0x64, 0x04, 0xb1, 0xe3,
		 0x3c, 0x6b, 0x0a, 0x8c,
		 0x8b, 0x80, 0x94, 0xfc,
		 0xf3, 0xc9, 0x98, 0xab}
};

uint8_t sha224_test1[] = {'a', 'b', 'c'};
uint8_t sha224_test2[] = { //size is 56
		'a', 'b', 'c', 'd', 'b', 'c', 'd', 'e',
		'c', 'd', 'e', 'f', 'd', 'e', 'f', 'g',
		'e', 'f', 'g', 'h', 'f', 'g', 'h', 'i',
		'g', 'h', 'i', 'j', 'h', 'i', 'j', 'k',
		'i', 'j', 'k', 'l', 'j', 'k', 'l', 'm',
		'k', 'l', 'm', 'n', 'l', 'm', 'n', 'o',
		'm', 'n', 'o', 'p', 'n', 'o', 'p', 'q'};
uint8_t sha224_test3[] = { //size is 64
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a'};

uint8_t sha224_results[3][SHA224_HASH_LENGTH]__attribute__ ((aligned (4))) =
{
	{0x23, 0x09, 0x7D, 0x22, 0x34, 0x05, 0xD8, 0x22,
	 0x86, 0x42, 0xA4, 0x77, 0xBD, 0xA2, 0x55, 0xB3,
	 0x2A, 0xAD, 0xBC, 0xE4, 0xBD, 0xA0, 0xB3, 0xF7,
	 0xE3, 0x6C, 0x9D, 0xA7},
	{0x75, 0x38, 0x8B, 0x16,0x51, 0x27, 0x76, 0xCC,
	 0x5D, 0xBA, 0x5D, 0xA1,0xFD, 0x89, 0x01, 0x50,
	 0xB0, 0xC6, 0x45, 0x5C,0xB4, 0xF5, 0x8B, 0x19,
	 0x52, 0x52, 0x25, 0x25},
	{0xA8, 0x8C, 0xD5, 0xCD, 0xE6, 0xD6, 0xFE, 0x91,
	 0x36, 0xA4, 0xE5, 0x8B, 0x49, 0x16, 0x74, 0x61,
	 0xEA, 0x95, 0xD3, 0x88, 0xCA, 0x2B, 0xDB, 0x7A,
	 0xFD, 0xC3, 0xCB, 0xF4}
};

uint8_t sha256_test1[] = {};
uint8_t sha256_test2[] = {0xbd};
uint8_t sha256_test3[] = {0x5f, 0xd4};
uint8_t sha256_test4[] = {0xb0, 0xbd, 0x69};
uint8_t sha256_test5[] = {0xc9, 0x8c, 0x8e, 0x55};
uint8_t sha256_test6[] = {0x81, 0xa7, 0x23, 0xd9, 0x66};
uint8_t sha256_test7[] = { //size is 56
		'a', 'b', 'c', 'd', 'b', 'c', 'd', 'e',
		'c', 'd', 'e', 'f', 'd', 'e', 'f', 'g',
		'e', 'f', 'g', 'h', 'f', 'g', 'h', 'i',
		'g', 'h', 'i', 'j', 'h', 'i', 'j', 'k',
		'i', 'j', 'k', 'l', 'j', 'k', 'l', 'm',
		'k', 'l', 'm', 'n', 'l', 'm', 'n', 'o',
		'm', 'n', 'o', 'p', 'n', 'o', 'p', 'q'};
uint8_t sha256_test8[] = { //size is 64
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a'};
uint8_t sha256_test9[] = { //size is 955
		0x83, 0x90, 0xcf, 0x0b, 0xe0, 0x76, 0x61, 0xcc, 0x76, 0x69, 0xaa, 0xc5,
		0x4c, 0xe0, 0x9a, 0x37, 0x73, 0x3a, 0x62, 0x9d, 0x45, 0xf5, 0xd9, 0x83,
		0xef, 0x20, 0x1f, 0x9b, 0x2d, 0x13, 0x80, 0x0e, 0x55, 0x5d, 0x9b, 0x10,
		0x97, 0xfe, 0xc3, 0xb7, 0x83, 0xd7, 0xa5, 0x0d, 0xcb, 0x5e, 0x2b, 0x64,
		0x4b, 0x96, 0xa1, 0xe9, 0x46, 0x3f, 0x17, 0x7c, 0xf3, 0x49, 0x06, 0xbf,
		0x38, 0x8f, 0x36, 0x6d, 0xb5, 0xc2, 0xde, 0xee, 0x04, 0xa3, 0x0e, 0x28,
		0x3f, 0x76, 0x4a, 0x97, 0xc3, 0xb3, 0x77, 0xa0, 0x34, 0xfe, 0xfc, 0x22,
		0xc2, 0x59, 0x21, 0x4f, 0xaa, 0x99, 0xba, 0xba, 0xff, 0x16, 0x0a, 0xb0,
		0xaa, 0xa7, 0xe2, 0xcc, 0xb0, 0xce, 0x09, 0xc6, 0xb3, 0x2f, 0xe0, 0x8c,
		0xbc, 0x47, 0x46, 0x94, 0x37, 0x5a, 0xba, 0x70, 0x3f, 0xad, 0xbf, 0xa3,
		0x1c, 0xf6, 0x85, 0xb3, 0x0a, 0x11, 0xc5, 0x7f, 0x3c, 0xf4, 0xed, 0xd3,
		0x21, 0xe5, 0x7d, 0x3a, 0xe6, 0xeb, 0xb1, 0x13, 0x3c, 0x82, 0x60, 0xe7,
		0x5b, 0x92, 0x24, 0xfa, 0x47, 0xa2, 0xbb, 0x20, 0x52, 0x49, 0xad, 0xd2,
		0xe2, 0xe6, 0x2f, 0x81, 0x74, 0x91, 0x48, 0x2a, 0xe1, 0x52, 0x32, 0x2b,
		0xe0, 0x90, 0x03, 0x55, 0xcd, 0xcc, 0x8d, 0x42, 0xa9, 0x8f, 0x82, 0xe9,
		0x61, 0xa0, 0xdc, 0x6f, 0x53, 0x7b, 0x7b, 0x41, 0x0e, 0xff, 0x10, 0x5f,
		0x59, 0x67, 0x3b, 0xfb, 0x78, 0x7b, 0xf0, 0x42, 0xaa, 0x07, 0x1f, 0x7a,
		0xf6, 0x8d, 0x94, 0x4d, 0x27, 0x37, 0x1c, 0x64, 0x16, 0x0f, 0xe9, 0x38,
		0x27, 0x72, 0x37, 0x25, 0x16, 0xc2, 0x30, 0xc1, 0xf4, 0x5c, 0x0d, 0x6b,
		0x6c, 0xca, 0x7f, 0x27, 0x4b, 0x39, 0x4d, 0xa9, 0x40, 0x2d, 0x3e, 0xaf,
		0xdf, 0x73, 0x39, 0x94, 0xec, 0x58, 0xab, 0x22, 0xd7, 0x18, 0x29, 0xa9,
		0x83, 0x99, 0x57, 0x4d, 0x4b, 0x59, 0x08, 0xa4, 0x47, 0xa5, 0xa6, 0x81,
		0xcb, 0x0d, 0xd5, 0x0a, 0x31, 0x14, 0x53, 0x11, 0xd9, 0x2c, 0x22, 0xa1,
		0x6d, 0xe1, 0xea, 0xd6, 0x6a, 0x54, 0x99, 0xf2, 0xdc, 0xeb, 0x4c, 0xae,
		0x69, 0x47, 0x72, 0xce, 0x90, 0x76, 0x2e, 0xf8, 0x33, 0x6a, 0xfe, 0xc6,
		0x53, 0xaa, 0x9b, 0x1a, 0x1c, 0x48, 0x20, 0xb2, 0x21, 0x13, 0x6d, 0xfc,
		0xe8, 0x0d, 0xce, 0x2b, 0xa9, 0x20, 0xd8, 0x8a, 0x53, 0x0c, 0x94, 0x10,
		0xd0, 0xa4, 0xe0, 0x35, 0x8a, 0x3a, 0x11, 0x05, 0x2e, 0x58, 0xdd, 0x73,
		0xb0, 0xb1, 0x79, 0xef, 0x8f, 0x56, 0xfe, 0x3b, 0x5a, 0x2d, 0x11, 0x7a,
		0x73, 0xa0, 0xc3, 0x8a, 0x13, 0x92, 0xb6, 0x93, 0x8e, 0x97, 0x82, 0xe0,
		0xd8, 0x64, 0x56, 0xee, 0x48, 0x84, 0xe3, 0xc3, 0x9d, 0x4d, 0x75, 0x81,
		0x3f, 0x13, 0x63, 0x3b, 0xc7, 0x9b, 0xaa, 0x07, 0xc0, 0xd2, 0xd5, 0x55,
		0xaf, 0xbf, 0x20, 0x7f, 0x52, 0xb7, 0xdc, 0xa1, 0x26, 0xd0, 0x15, 0xaa,
		0x2b, 0x98, 0x73, 0xb3, 0xeb, 0x06, 0x5e, 0x90, 0xb9, 0xb0, 0x65, 0xa5,
		0x37, 0x3f, 0xe1, 0xfb, 0x1b, 0x20, 0xd5, 0x94, 0x32, 0x7d, 0x19, 0xfb,
		0xa5, 0x6c, 0xb8, 0x1e, 0x7b, 0x66, 0x96, 0x60, 0x5f, 0xfa, 0x56, 0xeb,
		0xa3, 0xc2, 0x7a, 0x43, 0x86, 0x97, 0xcc, 0x21, 0xb2, 0x01, 0xfd, 0x7e,
		0x09, 0xf1, 0x8d, 0xee, 0xa1, 0xb3, 0xea, 0x2f, 0x0d, 0x1e, 0xdc, 0x02,
		0xdf, 0x0e, 0x20, 0x39, 0x6a, 0x14, 0x54, 0x12, 0xcd, 0x6b, 0x13, 0xc3,
		0x2d, 0x2e, 0x60, 0x56, 0x41, 0xc9, 0x48, 0xb7, 0x14, 0xae, 0xc3, 0x0c,
		0x06, 0x49, 0xdc, 0x44, 0x14, 0x35, 0x11, 0xf3, 0x5a, 0xb0, 0xfd, 0x5d,
		0xd6, 0x4c, 0x34, 0xd0, 0x6f, 0xe8, 0x6f, 0x38, 0x36, 0xdf, 0xe9, 0xed,
		0xeb, 0x7f, 0x08, 0xcf, 0xc3, 0xbd, 0x40, 0x95, 0x68, 0x26, 0x35, 0x62,
		0x42, 0x19, 0x1f, 0x99, 0xf5, 0x34, 0x73, 0xf3, 0x2b, 0x0c, 0xc0, 0xcf,
		0x93, 0x21, 0xd6, 0xc9, 0x2a, 0x11, 0x2e, 0x8d, 0xb9, 0x0b, 0x86, 0xee,
		0x9e, 0x87, 0xcc, 0x32, 0xd0, 0x34, 0x3d, 0xb0, 0x1e, 0x32, 0xce, 0x9e,
		0xb7, 0x82, 0xcb, 0x24, 0xef, 0xbb, 0xbe, 0xb4, 0x40, 0xfe, 0x92, 0x9e,
		0x8f, 0x2b, 0xf8, 0xdf, 0xb1, 0x55, 0x0a, 0x3a, 0x2e, 0x74, 0x2e, 0x8b,
		0x45, 0x5a, 0x3e, 0x57, 0x30, 0xe9, 0xe6, 0xa7, 0xa9, 0x82, 0x4d, 0x17,
		0xac, 0xc0, 0xf7, 0x2a, 0x7f, 0x67, 0xea, 0xe0, 0xf0, 0x97, 0x0f, 0x8b,
		0xde, 0x46, 0xdc, 0xde, 0xfa, 0xed, 0x30, 0x47, 0xcf, 0x80, 0x7e, 0x7f,
		0x00, 0xa4, 0x2e, 0x5f, 0xd1, 0x1d, 0x40, 0xf5, 0xe9, 0x85, 0x33, 0xd7,
		0x57, 0x44, 0x25, 0xb7, 0xd2, 0xbc, 0x3b, 0x38, 0x45, 0xc4, 0x43, 0x00,
		0x8b, 0x58, 0x98, 0x0e, 0x76, 0x8e, 0x46, 0x4e, 0x17, 0xcc, 0x6f, 0x6b,
		0x39, 0x39, 0xee, 0xe5, 0x2f, 0x71, 0x39, 0x63, 0xd0, 0x7d, 0x8c, 0x4a,
		0xbf, 0x02, 0x44, 0x8e, 0xf0, 0xb8, 0x89, 0xc9, 0x67, 0x1e, 0x2f, 0x8a,
		0x43, 0x6d, 0xde, 0xef, 0xfc, 0xca, 0x71, 0x76, 0xe9, 0xbf, 0x9d, 0x10,
		0x05, 0xec, 0xd3, 0x77, 0xf2, 0xfa, 0x67, 0xc2, 0x3e, 0xd1, 0xf1, 0x37,
		0xe6, 0x0b, 0xf4, 0x60, 0x18, 0xa8, 0xbd, 0x61, 0x3d, 0x03, 0x8e, 0x88,
		0x37, 0x04, 0xfc, 0x26, 0xe7, 0x98, 0x96, 0x9d, 0xf3, 0x5e, 0xc7, 0xbb,
		0xc6, 0xa4, 0xfe, 0x46, 0xd8, 0x91, 0x0b, 0xd8, 0x2f, 0xa3, 0xcd, 0xed,
		0x26, 0x5d, 0x0a, 0x3b, 0x6d, 0x39, 0x9e, 0x42, 0x51, 0xe4, 0xd8, 0x23,
		0x3d, 0xaa, 0x21, 0xb5, 0x81, 0x2f, 0xde, 0xd6, 0x53, 0x61, 0x98, 0xff,
		0x13, 0xaa, 0x5a, 0x1c, 0xd4, 0x6a, 0x5b, 0x9a, 0x17, 0xa4, 0xdd, 0xc1,
		0xd9, 0xf8, 0x55, 0x44, 0xd1, 0xd1, 0xcc, 0x16, 0xf3, 0xdf, 0x85, 0x80,
		0x38, 0xc8, 0xe0, 0x71, 0xa1, 0x1a, 0x7e, 0x15, 0x7a, 0x85, 0xa6, 0xa8,
		0xdc, 0x47, 0xe8, 0x8d, 0x75, 0xe7, 0x00, 0x9a, 0x8b, 0x26, 0xfd, 0xb7,
		0x3f, 0x33, 0xa2, 0xa7, 0x0f, 0x1e, 0x0c, 0x25, 0x9f, 0x8f, 0x95, 0x33,
		0xb9, 0xb8, 0xf9, 0xaf, 0x92, 0x88, 0xb7, 0x27, 0x4f, 0x21, 0xba, 0xee,
		0xc7, 0x8d, 0x39, 0x6f, 0x8b, 0xac, 0xdc, 0xc2, 0x24, 0x71, 0x20, 0x7d,
		0x9b, 0x4e, 0xfc, 0xcd, 0x3f, 0xed, 0xc5, 0xc5, 0xa2, 0x21, 0x4f, 0xf5,
		0xe5, 0x1c, 0x55, 0x3f, 0x35, 0xe2, 0x1a, 0xe6, 0x96, 0xfe, 0x51, 0xe8,
		0xdf, 0x73, 0x3a, 0x8e, 0x06, 0xf5, 0x0f, 0x41, 0x9e, 0x59, 0x9e, 0x9f,
		0x9e, 0x4b, 0x37, 0xce, 0x64, 0x3f, 0xc8, 0x10, 0xfa, 0xaa, 0x47, 0x98,
		0x97, 0x71, 0x50, 0x9d, 0x69, 0xa1, 0x10, 0xac, 0x91, 0x62, 0x61, 0x42,
		0x70, 0x26, 0x36, 0x9a, 0x21, 0x26, 0x3a, 0xc4, 0x46, 0x0f, 0xb4, 0xf7,
		0x08, 0xf8, 0xae, 0x28, 0x59, 0x98, 0x56, 0xdb, 0x7c, 0xb6, 0xa4, 0x3a,
		0xc8, 0xe0, 0x3d, 0x64, 0xa9, 0x60, 0x98, 0x07, 0xe7, 0x6c, 0x5f, 0x31,
		0x2b, 0x9d, 0x18, 0x63, 0xbf, 0xa3, 0x04, 0xe8, 0x95, 0x36, 0x47, 0x64,
		0x8b, 0x4f, 0x4a, 0xb0, 0xed, 0x99, 0x5e};

uint8_t sha256_results[9][SHA256_HASH_LENGTH] = {
		{0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14, 0x9a, 0xfb, 0xf4, 0xc8,
		 0x99, 0x6f, 0xb9, 0x24, 0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c,
		 0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55},
		{0x68, 0x32, 0x57, 0x20, 0xaa, 0xbd, 0x7c, 0x82, 0xf3, 0x0f, 0x55, 0x4b,
		 0x31, 0x3d, 0x05, 0x70, 0xc9, 0x5a, 0xcc, 0xbb, 0x7d, 0xc4, 0xb5, 0xaa,
		 0xe1, 0x12, 0x04, 0xc0, 0x8f, 0xfe, 0x73, 0x2b},
		{0x7c, 0x4f, 0xbf, 0x48, 0x44, 0x98, 0xd2, 0x1b, 0x48, 0x7b, 0x9d, 0x61,
		 0xde, 0x89, 0x14, 0xb2, 0xea, 0xda, 0xf2, 0x69, 0x87, 0x12, 0x93, 0x6d,
		 0x47, 0xc3, 0xad, 0xa2, 0x55, 0x8f, 0x67, 0x88},
		{0x40, 0x96, 0x80, 0x42, 0x21, 0x09, 0x3d, 0xdc, 0xcf, 0xbf, 0x46, 0x83,
		 0x14, 0x90, 0xea, 0x63, 0xe9, 0xe9, 0x94, 0x14, 0x85, 0x8f, 0x8d, 0x75,
		 0xff, 0x7f, 0x64, 0x2c, 0x7c, 0xa6, 0x18, 0x03},
		{0x7a, 0xbc, 0x22, 0xc0, 0xae, 0x5a, 0xf2, 0x6c, 0xe9, 0x3d, 0xbb, 0x94,
		 0x43, 0x3a, 0x0e, 0x0b, 0x2e, 0x11, 0x9d, 0x01, 0x4f, 0x8e, 0x7f, 0x65,
		 0xbd, 0x56, 0xc6, 0x1c, 0xcc, 0xcd, 0x95, 0x04},
		{0x75, 0x16, 0xfb, 0x8b, 0xb1, 0x13, 0x50, 0xdf, 0x2b, 0xf3, 0x86, 0xbc,
		 0x3c, 0x33, 0xbd, 0x0f, 0x52, 0xcb, 0x4c, 0x67, 0xc6, 0xe4, 0x74, 0x5e,
		 0x04, 0x88, 0xe6, 0x2c, 0x2a, 0xea, 0x26, 0x05},
		{0x24, 0x8D, 0x6A, 0x61, 0xD2, 0x06, 0x38, 0xB8,
		 0xE5, 0xC0, 0x26, 0x93, 0x0C, 0x3E, 0x60, 0x39,
		 0xA3, 0x3C, 0xE4, 0x59, 0x64, 0xFF, 0x21, 0x67,
		 0xF6, 0xEC, 0xED, 0xD4, 0x19, 0xDB, 0x06, 0xC1},
		{0xFF, 0xE0, 0x54, 0xFE, 0x7A, 0xE0, 0xCB, 0x6D,
		 0xC6, 0x5C, 0x3A, 0xF9, 0xB6, 0x1D, 0x52, 0x09,
		 0xF4, 0x39, 0x85, 0x1D, 0xB4, 0x3D, 0x0B, 0xA5,
		 0x99, 0x73, 0x37, 0xDF, 0x15, 0x46, 0x68, 0xEB},
		{0x41, 0x09, 0xcd, 0xbe, 0xc3, 0x24, 0x0a, 0xd7, 0x4c, 0xc6, 0xc3, 0x7f,
		 0x39, 0x30, 0x0f, 0x70, 0xfe, 0xde, 0x16, 0xe2, 0x1e, 0xfc, 0x77, 0xf7,
		 0x86, 0x59, 0x98, 0x71, 0x4a, 0xad, 0x0b, 0x5e}
};

uint8_t sha384_test1[] = {'a', 'b', 'c'};
uint8_t sha384_test2[] = { //size is 120
		'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i',
		'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
		'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
		'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o',
		'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q',
		'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's',
		'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a'};
uint8_t sha384_test3[] = { //size is 128*5
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',

		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',

		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',

		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',

		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a'};

static const uint8_t sha384_results[3][SHA384_HASH_LENGTH]__attribute__ ((aligned (4))) =
{
	/* SHA384(abc)=> (big endian)
	 * CB00753F 45A35E8B B5A03D69 9AC65007 272C32AB 0EDED163
	 * 1A8B605A 43FF5BED 8086072B A1E7CC23 58BAECA1 34C825A7 */
	{0xCB, 0x00, 0x75, 0x3F, 0x45, 0xA3, 0x5E, 0x8B,
	 0xB5, 0xA0, 0x3D, 0x69, 0x9A, 0xC6, 0x50, 0x07,
	 0x27, 0x2C, 0x32, 0xAB, 0x0E, 0xDE, 0xD1, 0x63,
	 0x1A, 0x8B, 0x60, 0x5A, 0x43, 0xFF, 0x5B, 0xED,
	 0x80, 0x86, 0x07, 0x2B, 0xA1, 0xE7, 0xCC, 0x23,
	 0x58, 0xBA, 0xEC, 0xA1, 0x34, 0xC8, 0x25, 0xA7},

	/* FAD1D20B 140A7438 FC501214 3D38BE9C F86BA566 CBAD1C87 004DD4B8 5E2F0334
	 * FA368D18 8B3F7E7D 84208B48 304F7F2B */
	{0xFA, 0xD1, 0xD2, 0x0B, 0x14, 0x0A, 0x74, 0x38,
	 0xFC, 0x50, 0x12, 0x14, 0x3D, 0x38, 0xBE, 0x9C,
	 0xF8, 0x6B, 0xA5, 0x66, 0xCB, 0xAD, 0x1C, 0x87,
	 0x00, 0x4D, 0xD4, 0xB8, 0x5E, 0x2F, 0x03, 0x34,
	 0xFA, 0x36, 0x8D, 0x18, 0x8B, 0x3F, 0x7E, 0x7D,
	 0x84, 0x20, 0x8B, 0x48, 0x30, 0x4F, 0x7F, 0x2B},

	/* EB16D0EC 106CD00A 51F85C32 0D48C121 AA3296A1 E9EAFC85 4A6B6E8F 9A860BB5 115BB160
	 * 3F0ECEA8 79C54D48 7000A78C */
	{0xEB, 0x16, 0xD0, 0xEC, 0x10, 0x6C, 0xD0, 0x0A,
	 0x51, 0xF8, 0x5C, 0x32, 0x0D, 0x48, 0xC1, 0x21,
	 0xAA, 0x32, 0x96, 0xA1, 0xE9, 0xEA, 0xFC, 0x85,
	 0x4A, 0x6B, 0x6E, 0x8F, 0x9A, 0x86, 0x0B, 0xB5,
	 0x11, 0x5B, 0xB1, 0x60, 0x3F, 0x0E, 0xCE, 0xA8,
	 0x79, 0xC5, 0x4D, 0x48, 0x70, 0x00, 0xA7, 0x8C}
};

uint8_t sha512_test1[] = {'a', 'b', 'c'};
uint8_t sha512_test2[] = { //size is 120
		'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i',
		'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
		'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
		'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o',
		'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q',
		'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's',
		'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a'};
uint8_t sha512_test3[] = { //size is 128*5
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',

		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',

		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',

		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',

		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a',
		'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a', 'a'};

static const uint8_t sha512_results[3][SHA512_HASH_LENGTH]__attribute__ ((aligned (4))) =
{
	/* SHA512(abc)=> (big endian)
	 * DDAF35A1 93617ABA CC417349 AE204131 12E6FA4E 89A97EA2 0A9EEEE6 4B55D39A
	 * 2192992A 274FC1A8 36BA3C23 A3FEEBBD 454D4423 643CE80E 2A9AC94F A54CA49F */
	{0xDD, 0xAF, 0x35, 0xA1, 0x93, 0x61, 0x7A, 0xBA,
	 0xCC, 0x41, 0x73, 0x49, 0xAE, 0x20, 0x41, 0x31,
	 0x12, 0xE6, 0xFA, 0x4E, 0x89, 0xA9, 0x7E, 0xA2,
	 0x0A, 0x9E, 0xEE, 0xE6, 0x4B, 0x55, 0xD3, 0x9A,
	 0x21, 0x92, 0x99, 0x2A, 0x27, 0x4F, 0xC1, 0xA8,
	 0x36, 0xBA, 0x3C, 0x23, 0xA3, 0xFE, 0xEB, 0xBD,
	 0x45, 0x4D, 0x44, 0x23, 0x64, 0x3C, 0xE8, 0x0E,
	 0x2A, 0x9A, 0xC9, 0x4F, 0xA5, 0x4C, 0xA4, 0x9F},

	/* 50650CF1 E7BC2B30 3C1469AD 80493065 1D7A7196 703B4658 2DE396C7 7B09F554
	 * 069640A0 32F690B0 98E9D9D2 97AD7CD8 803D9402 628D1CF6 16DBF08E BEF1420C */
	{0x50, 0x65, 0x0C, 0xF1, 0xE7, 0xBC, 0x2B, 0x30,
	 0x3C, 0x14, 0x69, 0xAD, 0x80, 0x49, 0x30, 0x65,
	 0x1D, 0x7A, 0x71, 0x96, 0x70, 0x3B, 0x46, 0x58,
	 0x2D, 0xE3, 0x96, 0xC7, 0x7B, 0x09, 0xF5, 0x54,
	 0x06, 0x96, 0x40, 0xA0, 0x32, 0xF6, 0x90, 0xB0,
	 0x98, 0xE9, 0xD9, 0xD2, 0x97, 0xAD, 0x7C, 0xD8,
	 0x80, 0x3D, 0x94, 0x02, 0x62, 0x8D, 0x1C, 0xF6,
	 0x16, 0xDB, 0xF0, 0x8E, 0xBE, 0xF1, 0x42, 0x0C},

	/* F2947A92 F28EAD25 15AB9945 7930FED9 31107B9A E89DD95E 2D328F95 EFAFAA97
	 * EE686EA5 17990218 8A1EA34F A90C4852 49F1C8F4 0C3A0022 E0556B83 28AA10DC */
	{0xF2, 0x94, 0x7A, 0x92, 0xF2, 0x8E, 0xAD, 0x25,
	 0x15, 0xAB, 0x99, 0x45, 0x79, 0x30, 0xFE, 0xD9,
	 0x31, 0x10, 0x7B, 0x9A, 0xE8, 0x9D, 0xD9, 0x5E,
	 0x2D, 0x32, 0x8F, 0x95, 0xEF, 0xAF, 0xAA, 0x97,
	 0xEE, 0x68, 0x6E, 0xA5, 0x17, 0x99, 0x02, 0x18,
	 0x8A, 0x1E, 0xA3, 0x4F, 0xA9, 0x0C, 0x48, 0x52,
	 0x49, 0xF1, 0xC8, 0xF4, 0x0C, 0x3A, 0x00, 0x22,
	 0xE0, 0x55, 0x6B, 0x83, 0x28, 0xAA, 0x10, 0xDC}
};

static int sha_hash_tests(const struct device *dev,
				uint8_t *in, size_t in_len,
				const uint8_t *expected, enum hash_algo algo)
{
	int ret;
	struct hash_ctx ctx;
	uint8_t out_buf[SHA512_HASH_LENGTH] = {0};
	size_t out_len = 0;

	switch (algo) {
		case CRYPTO_HASH_ALGO_SHA1:
			out_len = SHA160_HASH_LENGTH;
			break;
		case CRYPTO_HASH_ALGO_SHA224:
			out_len = SHA224_HASH_LENGTH;
			break;
		case CRYPTO_HASH_ALGO_SHA256:
			out_len = SHA256_HASH_LENGTH;
			break;
		case CRYPTO_HASH_ALGO_SHA384:
			out_len = SHA384_HASH_LENGTH;
			break;
		case CRYPTO_HASH_ALGO_SHA512:
			out_len = SHA512_HASH_LENGTH;
			break;
		default:
			break;
	}

	ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

	ret = hash_begin_session(dev, &ctx, algo);
	if (ret != 0) {
		LOG_ERR("Failed to init sha session (%d)", ret);
		return ret;
	}

	struct hash_pkt pkt = {
		.in_buf = in,
		.in_len = in_len,
		.out_buf = out_buf,
	};

	ret = hash_compute(&ctx, &pkt);
	if (ret != 0) {
		LOG_ERR("hash_compute failed (%d)", ret);
		hash_free_session(dev, &ctx);
		return ret;
	}

	ret = memcmp(out_buf, expected, out_len);
	hash_free_session(dev, &ctx);
	return ret;
}

static void crypto_hash(const struct device *dev)
{
	int ret;
#define TEST_HASH(_i, _algo) \
	do { \
		ret = sha_hash_tests(dev, sha##_algo##_test##_i, \
				sizeof(sha##_algo##_test##_i), sha##_algo##_results[(_i)-1], \
				CRYPTO_HASH_ALGO_SHA##_algo); \
		if (ret == 0) { \
			LOG_INF("SHA-%s test %d (%d bytes): PASS", #_algo, (_i), sizeof(sha##_algo##_test##_i)); \
		} else { \
			LOG_ERR("SHA-%s test %d (%d bytes): FAIL", #_algo, (_i), sizeof(sha##_algo##_test##_i)); \
		} \
	} while (0)

	/* SHA1 */
	TEST_HASH(1, 1);
	TEST_HASH(2, 1);
	TEST_HASH(3, 1);
	TEST_HASH(4, 1);

	/* SHA224 */
	TEST_HASH(1, 224);
	TEST_HASH(2, 224);
	TEST_HASH(3, 224);

	/* SHA256 */
	TEST_HASH(1, 256);
	TEST_HASH(2, 256);
	TEST_HASH(3, 256);
	TEST_HASH(4, 256);
	TEST_HASH(5, 256);
	TEST_HASH(6, 256);
	TEST_HASH(7, 256);
	TEST_HASH(8, 256);
	TEST_HASH(9, 256);

	/* SHA384 */
	TEST_HASH(1, 384);
	TEST_HASH(2, 384);
	TEST_HASH(3, 384);

	/* SHA512 */
	TEST_HASH(1, 512);
	TEST_HASH(2, 512);
	TEST_HASH(3, 512);
}

static int sha256_hash_stop_resume_test(const struct device *dev,
				uint8_t *in, size_t in_len,
				const uint8_t *expected, enum hash_algo algo)
{
	uint8_t sha256_in_data[64];
	uint8_t sha256_expected_result[SHA256_HASH_LENGTH] = {
			0x9b, 0x6c, 0xe5, 0x5f, 0x37, 0x9e, 0x97, 0x71,
			0x55, 0x1d, 0xe6, 0x93, 0x95, 0x56, 0xa7, 0xe6,
			0xb9, 0x49, 0x81, 0x4a, 0xe2, 0x7c, 0x2f, 0x5c,
			0xfd, 0x5d, 0xbe, 0xb3, 0x78, 0xce, 0x7c, 0x2a
	};
	struct hash_ctx ctx, ctx2;
	uint8_t result[SHA256_HASH_LENGTH] = {0};
	uint8_t result2[SHA256_HASH_LENGTH] = {0};
	size_t result2_len = (algo == CRYPTO_HASH_ALGO_SHA1)? SHA160_HASH_LENGTH : SHA256_HASH_LENGTH;
	int ret, i;

	LOG_INF("%s inserts SHA-%d START", __func__, (algo == CRYPTO_HASH_ALGO_SHA1)? 1 : 256);
	memset(sha256_in_data, 'B', sizeof(sha256_in_data));

	ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	ret = hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA256);
	if (ret != 0) {
		LOG_ERR("Failed to init sha session (%d)", ret);
		return ret;
	}

	struct hash_pkt pkt = {
		.in_buf = sha256_in_data,
		.in_len = sizeof(sha256_in_data),
		.out_buf = result,
	};

	/* Write 1K bytes data to register. (64 bytes x 16)*/
	/* When i as 7, backup hashs and do another process. */
	/* Then reload hashs and resume not finish process. */
	for(i = 0; i < 8; i++) {
		ret = hash_update(&ctx, &pkt);
		if (ret != 0) {
			LOG_ERR("hash_compute failed (%d)", ret);
			hash_free_session(dev, &ctx);
			return ret;
		}
	}

	/* Insert another context process */
	ctx2.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	ret = hash_begin_session(dev, &ctx2, algo);
	if (ret != 0) {
		LOG_ERR("Failed to init sha session2 (%d)", ret);
		return ret;
	}

	struct hash_pkt pkt2 = {
		.in_buf = in,
		.in_len = in_len,
		.out_buf = result2,
	};

	ret = hash_compute(&ctx2, &pkt2);
	if (ret != 0) {
		LOG_ERR("hash_compute failed (%d)", ret);
		hash_free_session(dev, &ctx2);
		return ret;
	}

	ret = memcmp(result2, expected, result2_len);
	if (ret == 0) {
		LOG_INF("Insert SHA-%d : PASS", (algo == CRYPTO_HASH_ALGO_SHA1)? 1 : 256);
	} else {
		LOG_INF("Insert SHA-%d: FAIL", (algo == CRYPTO_HASH_ALGO_SHA1)? 1 : 256);
	}
	/* End of inserting another context process */

	for(i = 0; i < 7; i++) {
		ret = hash_update(&ctx, &pkt);
		if (ret != 0) {
			LOG_ERR("hash_compute failed (%d)", ret);
			hash_free_session(dev, &ctx);
			return ret;
		}
	}

	ret = hash_compute(&ctx, &pkt);
	if (ret != 0) {
		LOG_ERR("hash_compute failed (%d)", ret);
		hash_free_session(dev, &ctx);
		return ret;
	}

	ret = memcmp(result, sha256_expected_result, SHA256_HASH_LENGTH);
	if (ret == 0) {
		LOG_INF("%s inserts SHA-%d: PASS", __func__, (algo == CRYPTO_HASH_ALGO_SHA1)? 1 : 256);
	} else {
		LOG_INF("%s inserts SHA-%d: FAIL", __func__, (algo == CRYPTO_HASH_ALGO_SHA1)? 1 : 256);
	}

	hash_free_session(dev, &ctx);
	hash_free_session(dev, &ctx2);
	return ret;
}

static int sha512_hash_stop_resume_test(const struct device *dev,
				uint8_t *in, size_t in_len,
				const uint8_t *expected, enum hash_algo algo)
{
	uint8_t sha512_in_data[128];
	struct hash_ctx ctx, ctx2;
	uint8_t result[SHA512_HASH_LENGTH] = {0};
	uint8_t result2[SHA512_HASH_LENGTH] = {0};
	size_t result2_len = (CRYPTO_HASH_ALGO_SHA384)? SHA384_HASH_LENGTH : SHA512_HASH_LENGTH;
	int ret, i;

	LOG_INF("%s inserts SHA-%d START", __func__, (algo == CRYPTO_HASH_ALGO_SHA384)? 384 : 512);
	memset(sha512_in_data, 'a', sizeof(sha512_in_data));

	ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	ret = hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA512);
	if (ret != 0) {
		LOG_ERR("Failed to init sha session (%d)", ret);
		return ret;
	}

	struct hash_pkt pkt = {
		.in_buf = sha512_in_data,
		.in_len = sizeof(sha512_in_data),
		.out_buf = result,
	};

	/* Write 640 bytes data to register. (128 bytes x 5)*/
	/* When i as 4, backup hashs and do another process. */
	/* Then reload hashs and resume not finish process. */
	for(i = 0; i < 4; i++) {
		ret = hash_update(&ctx, &pkt);
		if (ret != 0) {
			LOG_ERR("hash_compute failed (%d)", ret);
			hash_free_session(dev, &ctx);
			return ret;
		}
	}

	/* Insert another context process */
	ctx2.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	ret = hash_begin_session(dev, &ctx2, algo);
	if (ret != 0) {
		LOG_ERR("Failed to init sha session2 (%d)", ret);
		return ret;
	}

	struct hash_pkt pkt2 = {
		.in_buf = in,
		.in_len = in_len,
		.out_buf = result2,
	};

	ret = hash_compute(&ctx2, &pkt2);
	if (ret != 0) {
		LOG_ERR("hash_compute failed (%d)", ret);
		hash_free_session(dev, &ctx2);
		return ret;
	}

	ret = memcmp(result2, expected, result2_len);
	if (ret == 0) {
		LOG_INF("Insert SHA-%d : PASS", (algo == CRYPTO_HASH_ALGO_SHA384)? 384 : 512);
	} else {
		LOG_INF("Insert SHA-%d: FAIL", (algo == CRYPTO_HASH_ALGO_SHA384)? 384 : 512);
	}
	/* End of inserting another context process */

	ret = hash_compute(&ctx, &pkt);
	if (ret != 0) {
		LOG_ERR("hash_compute failed (%d)", ret);
		hash_free_session(dev, &ctx);
		return ret;
	}

	ret = memcmp(result, sha512_results[2], SHA512_HASH_LENGTH);
	if (ret == 0) {
		LOG_INF("%s inserts SHA-%d: PASS", __func__, (algo == CRYPTO_HASH_ALGO_SHA384)? 384 : 512);
	} else {
		LOG_INF("%s inserts SHA-%d: FAIL", __func__, (algo == CRYPTO_HASH_ALGO_SHA384)? 384 : 512);
	}

	hash_free_session(dev, &ctx);
	hash_free_session(dev, &ctx2);
	return ret;
}

static void crypto_hash_stop_resume(const struct device *dev)
{
	sha256_hash_stop_resume_test(dev, sha256_test1, sizeof(sha256_test1),
								 sha256_results[0], CRYPTO_HASH_ALGO_SHA256);
	sha256_hash_stop_resume_test(dev, sha1_test1, sizeof(sha1_test1),
								 sha1_results[0], CRYPTO_HASH_ALGO_SHA1);
	sha512_hash_stop_resume_test(dev, sha512_test1, sizeof(sha512_test1),
								 sha512_results[0], CRYPTO_HASH_ALGO_SHA512);
	sha512_hash_stop_resume_test(dev, sha384_test1, sizeof(sha384_test1),
								 sha384_results[0], CRYPTO_HASH_ALGO_SHA384);
}
#endif //CONFIG_CRYPTO_NPCM_SHA

int main(void)
{
#ifdef CRYPTO_DRV_NAME
	const struct device *dev = device_get_binding(CRYPTO_DRV_NAME);

	if (!dev) {
		LOG_ERR("%s pseudo device not found", CRYPTO_DRV_NAME);
		return 0;
	}
#else
	const struct device *const dev = DEVICE_DT_GET_ONE(CRYPTO_DEV_COMPAT);

	if (!device_is_ready(dev)) {
		LOG_ERR("Crypto device is not ready\n");
		return 0;
	}
#endif

#if CONFIG_CRYPTO_NPCM_SHA
	LOG_INF("Crypto Hash");
	crypto_hash(dev);
	crypto_hash_stop_resume(dev);
#else
	const struct mode_test modes[] = {
		{ .mode = "ECB Mode", .mode_func = ecb_mode },
		{ .mode = "CBC Mode", .mode_func = cbc_mode },
		{ .mode = "CTR Mode", .mode_func = ctr_mode },
		{ .mode = "CCM Mode", .mode_func = ccm_mode },
		{ .mode = "GCM Mode", .mode_func = gcm_mode },
		{ },
	};
	int i;

	if (validate_hw_compatibility(dev)) {
		LOG_ERR("Incompatible h/w");
		return 0;
	}

	LOG_INF("Cipher Sample");

	for (i = 0; modes[i].mode; i++) {
		LOG_INF("%s", modes[i].mode);
		modes[i].mode_func(dev);
	}
#endif //CONFIG_CRYPTO_NPCM_SHA

	return 0;
}
