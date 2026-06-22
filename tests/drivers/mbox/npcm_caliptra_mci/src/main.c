/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/ztest.h>
#include <zephyr/drivers/mbox/mbox_npcm_caliptra_mci_proto.h>

ZTEST(npcm_caliptra_mci_proto, test_decode_status)
{
	zassert_equal(caliptra_mci_mbox_decode_status(0x00), CALIPTRA_MCI_MBOX_BUSY);
	zassert_equal(caliptra_mci_mbox_decode_status(0x02), CALIPTRA_MCI_MBOX_COMPLETE);
	/* DONE bit (bit4) must not leak into the status code. */
	zassert_equal(caliptra_mci_mbox_decode_status(0x12), CALIPTRA_MCI_MBOX_COMPLETE);
	zassert_false(caliptra_mci_mbox_status_done(0x02));
	zassert_true(caliptra_mci_mbox_status_done(0x12));
}

ZTEST(npcm_caliptra_mci_proto, test_check_len)
{
	zassert_equal(caliptra_mci_mbox_check_len(0, 0x1000), 0);
	zassert_equal(caliptra_mci_mbox_check_len(0x1000, 0x1000), 0);
	zassert_equal(caliptra_mci_mbox_check_len(0x1004, 0x1000), -ENOSPC);
	zassert_equal(caliptra_mci_mbox_check_len(3, 0x1000), -EINVAL);
}

ZTEST(npcm_caliptra_mci_proto, test_state_machine_happy_path)
{
	enum caliptra_mci_mbox_state s = CALIPTRA_MCI_MBOX_STATE_IDLE;

	zassert_equal(caliptra_mci_mbox_state_next(s, CALIPTRA_MCI_MBOX_EV_ACQUIRE, &s), 0);
	zassert_equal(s, CALIPTRA_MCI_MBOX_STATE_LOCKED);
	zassert_equal(caliptra_mci_mbox_state_next(s, CALIPTRA_MCI_MBOX_EV_STAGE, &s), 0);
	zassert_equal(s, CALIPTRA_MCI_MBOX_STATE_STAGED);
	zassert_equal(caliptra_mci_mbox_state_next(s, CALIPTRA_MCI_MBOX_EV_SEND, &s), 0);
	zassert_equal(s, CALIPTRA_MCI_MBOX_STATE_BUSY);
	zassert_equal(caliptra_mci_mbox_state_next(s, CALIPTRA_MCI_MBOX_EV_COMPLETE, &s), 0);
	zassert_equal(s, CALIPTRA_MCI_MBOX_STATE_LOCKED);
	zassert_equal(caliptra_mci_mbox_state_next(s, CALIPTRA_MCI_MBOX_EV_RELEASE, &s), 0);
	zassert_equal(s, CALIPTRA_MCI_MBOX_STATE_IDLE);
}

ZTEST(npcm_caliptra_mci_proto, test_state_machine_rejects_out_of_order)
{
	enum caliptra_mci_mbox_state s = CALIPTRA_MCI_MBOX_STATE_IDLE;

	/* Cannot doorbell without staging. */
	zassert_equal(caliptra_mci_mbox_state_next(s, CALIPTRA_MCI_MBOX_EV_SEND, &s), -EPERM);
	zassert_equal(s, CALIPTRA_MCI_MBOX_STATE_IDLE);
	/* Cannot stage without the lock. */
	zassert_equal(caliptra_mci_mbox_state_next(s, CALIPTRA_MCI_MBOX_EV_STAGE, &s), -EPERM);
	zassert_equal(s, CALIPTRA_MCI_MBOX_STATE_IDLE);
}

ZTEST(npcm_caliptra_mci_proto, test_state_machine_responder_path)
{
	/* Responder path: dispatch puts IDLE -> LOCKED on EV_ACQUIRE
	 * (observing EXECUTE=1), write_response transitions LOCKED -> CLOSING,
	 * dispatch returns CLOSING -> IDLE on observing EXECUTE=0. */
	enum caliptra_mci_mbox_state s = CALIPTRA_MCI_MBOX_STATE_IDLE;

	zassert_equal(caliptra_mci_mbox_state_next(s, CALIPTRA_MCI_MBOX_EV_ACQUIRE, &s), 0);
	zassert_equal(s, CALIPTRA_MCI_MBOX_STATE_LOCKED);
	zassert_equal(caliptra_mci_mbox_state_next(s, CALIPTRA_MCI_MBOX_EV_RESPOND, &s), 0);
	zassert_equal(s, CALIPTRA_MCI_MBOX_STATE_CLOSING);
	zassert_equal(caliptra_mci_mbox_state_next(s, CALIPTRA_MCI_MBOX_EV_CLOSED, &s), 0);
	zassert_equal(s, CALIPTRA_MCI_MBOX_STATE_IDLE);
}

ZTEST(npcm_caliptra_mci_proto, test_state_machine_responder_rejects_misuse)
{
	enum caliptra_mci_mbox_state s = CALIPTRA_MCI_MBOX_STATE_IDLE;

	/* Cannot respond before the request has been latched. */
	zassert_equal(caliptra_mci_mbox_state_next(s, CALIPTRA_MCI_MBOX_EV_RESPOND, &s), -EPERM);
	/* Cannot signal CLOSED without first responding. */
	zassert_equal(caliptra_mci_mbox_state_next(s, CALIPTRA_MCI_MBOX_EV_CLOSED, &s), -EPERM);
}

ZTEST(npcm_caliptra_mci_proto, test_cmd_status_responded)
{
	/* Requester direction: CMD_STATUS has no DONE bit. Any non-BUSY code
	 * means the MCU has replied. */
	zassert_false(caliptra_mci_mbox_cmd_status_responded(CALIPTRA_MCI_MBOX_BUSY));
	zassert_true(caliptra_mci_mbox_cmd_status_responded(CALIPTRA_MCI_MBOX_DATA_READY));
	zassert_true(caliptra_mci_mbox_cmd_status_responded(CALIPTRA_MCI_MBOX_COMPLETE));
	zassert_true(caliptra_mci_mbox_cmd_status_responded(CALIPTRA_MCI_MBOX_FAILURE));
}

ZTEST_SUITE(npcm_caliptra_mci_proto, NULL, NULL, NULL, NULL, NULL);
