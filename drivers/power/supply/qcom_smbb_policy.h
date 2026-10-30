/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * LG thermistor data: Copyright (c) 2012-2014, The Linux Foundation.
 * LG V1_8 charging rules: Copyright (C) 2013 LG Electronics.
 * Vendor source 03aba4427a7666d57b5ae06d375e259ea337f351.
 * This model is authorized only by a runtime-selected TOCAD Madai profile.
 */
#ifndef _QCOM_SMBB_POLICY_H
#define _QCOM_SMBB_POLICY_H

struct smbb_ntc_point {
	int temperature; /* deci-Celsius */
	int millivolts;
};

static const struct smbb_ntc_point smbb_madai_ntc[] = {
	{ -300, 1627 },
	{ -200, 1507 },
	{ -100, 1345 },
	{ 0, 1159 },
	{ 10, 1139 },
	{ 20, 1120 },
	{ 30, 1101 },
	{ 40, 1082 },
	{ 50, 1064 },
	{ 60, 1045 },
	{ 70, 1026 },
	{ 80, 1008 },
	{ 90, 989 },
	{ 100, 972 },
	{ 110, 954 },
	{ 120, 936 },
	{ 130, 919 },
	{ 140, 902 },
	{ 150, 885 },
	{ 160, 869 },
	{ 170, 853 },
	{ 180, 837 },
	{ 190, 822 },
	{ 200, 807 },
	{ 210, 792 },
	{ 220, 777 },
	{ 230, 763 },
	{ 240, 750 },
	{ 250, 737 },
	{ 260, 724 },
	{ 270, 711 },
	{ 280, 699 },
	{ 290, 687 },
	{ 300, 675 },
	{ 310, 664 },
	{ 320, 653 },
	{ 330, 643 },
	{ 340, 633 },
	{ 350, 623 },
	{ 360, 613 },
	{ 370, 604 },
	{ 380, 595 },
	{ 390, 586 },
	{ 400, 578 },
	{ 410, 570 },
	{ 420, 562 },
	{ 430, 555 },
	{ 440, 547 },
	{ 450, 540 },
	{ 460, 533 },
	{ 470, 527 },
	{ 480, 521 },
	{ 490, 515 },
	{ 500, 509 },
	{ 510, 503 },
	{ 520, 498 },
	{ 530, 492 },
	{ 540, 487 },
	{ 550, 482 },
	{ 560, 478 },
	{ 570, 473 },
	{ 580, 469 },
	{ 590, 465 },
	{ 600, 461 },
	{ 610, 457 },
	{ 620, 453 },
	{ 630, 449 },
	{ 640, 446 },
	{ 650, 442 },
	{ 660, 439 },
	{ 670, 436 },
	{ 680, 433 },
	{ 690, 430 },
	{ 700, 427 },
	{ 710, 425 },
	{ 720, 422 },
	{ 730, 420 },
	{ 740, 417 },
	{ 750, 415 },
	{ 760, 413 },
	{ 770, 411 },
	{ 780, 409 },
	{ 790, 407 },
};

enum smbb_policy_state {
	SMBB_POLICY_NORMAL,
	SMBB_POLICY_REDUCED,
	SMBB_POLICY_STOP,
};

static int smbb_policy_temperature(int millivolts, int *temperature)
{
	const struct smbb_ntc_point *hi, *lo;
	int i;

	/* Reject open/short/clamped endpoints instead of reporting a valid temp. */
	if (millivolts >= smbb_madai_ntc[0].millivolts ||
	    millivolts <= smbb_madai_ntc[ARRAY_SIZE(smbb_madai_ntc) - 1].millivolts)
		return -ERANGE;
	for (i = 1; i < ARRAY_SIZE(smbb_madai_ntc); i++) {
		hi = &smbb_madai_ntc[i - 1];
		lo = &smbb_madai_ntc[i];
		if (millivolts < lo->millivolts)
			continue;
		*temperature = hi->temperature +
			(lo->temperature - hi->temperature) *
			(millivolts - hi->millivolts) /
			(lo->millivolts - hi->millivolts);
		return 0;
	}
	return -ERANGE;
}

static enum smbb_policy_state smbb_policy_next(enum smbb_policy_state previous,
					       int temperature, int microvolts)
{
	if (temperature < -100 || temperature > 550)
		return SMBB_POLICY_STOP;

	switch (previous) {
	case SMBB_POLICY_NORMAL:
		if (temperature <= 450)
			return SMBB_POLICY_NORMAL;
		return microvolts > 4000000 ? SMBB_POLICY_STOP : SMBB_POLICY_REDUCED;
	case SMBB_POLICY_REDUCED:
		/* Stock checks voltage before the cool-temperature recovery. */
		if (microvolts > 4000000)
			return SMBB_POLICY_STOP;
		return temperature <= 419 ? SMBB_POLICY_NORMAL : SMBB_POLICY_REDUCED;
	case SMBB_POLICY_STOP:
		if (temperature >= -49 && temperature <= 419)
			return SMBB_POLICY_NORMAL;
		if (temperature >= 420 && temperature <= 520 && microvolts <= 4000000)
			return SMBB_POLICY_REDUCED;
		return SMBB_POLICY_STOP;
	default:
		return SMBB_POLICY_STOP;
	}
}

#endif
