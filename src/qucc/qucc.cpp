#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "qucc.h"

#define SWAP_BYTE 1
#define SHIFT_BIT (SWAP_BYTE*8)

#define SWAP_2BYTE(x) ((x>>SHIFT_BIT)|(x<<SHIFT_BIT))

typedef struct _QuccData {
	uint16_t total_voltage_10mv;
	uint16_t current_10ma;
	uint16_t remaining_capacity_10mah;
	uint16_t norminal_capacity_10mah;
	uint16_t number_of_cycles;
	uint16_t production_date;
	uint16_t balanced_state;
	uint16_t balanced_state_high;
	uint16_t protection_status;
	uint8_t software_version;
	uint8_t rsoc;
	uint8_t fet_control_state;
	uint8_t number_of_battery_strings;
	uint8_t number_of_ntc;
	uint16_t ntc_1st;
	uint16_t ntc_2nd;
	uint16_t ntc_3rd;
	uint16_t ntc_4th;
} QuccData;

typedef struct _QuccInfo {
	double voltage_v;
	double current_a;
	double remaining_capacity_ah;
	double norminal_capacity_ah;
	uint16_t cycles;
	uint16_t production_date;
	uint16_t balanced_low;
	uint16_t balanced_high;
	uint16_t protection_status;
	uint8_t software_version;
	double remaining_capacity_percent;
	uint8_t fet_control_state;
	uint8_t number_of_battery_strings;
	uint8_t number_of_ntc;
	double celsius_1st;
	double celsius_2nd;
	double celsius_3rd;
	double celsius_4th;
} QuccInfo;

#define QUCC_INFO_STATUS

int parseRxData(QuccData quccData) {
	static QuccInfo quccInfo = {0, };

	// 총전압(raw unit: 10mV)
	quccData.total_voltage_10mv = SWAP_2BYTE(quccData.total_voltage_10mv);
	quccInfo.voltage_v = quccData.total_voltage_10mv / 100.0;
	printf("총전압(raw unit: 10mV) : %d\n", quccData.total_voltage_10mv);
	printf("총전압(V) : %.3lf\n", quccInfo.voltage_v);
	// 전류소비량(raw unit: 10mA)
	quccData.current_10ma = SWAP_2BYTE(quccData.current_10ma);
	#define MSB_DISCHARGE 15
	#define IS_DISCHARGE(x) ((x>>MSB_DISCHARGE) & 0x0001)
	#define CALC_CURRENT_A(x) ((65536.0-(double)x)/100.0)
	quccInfo.current_a = IS_DISCHARGE(quccData.current_10ma)?CALC_CURRENT_A(quccData.current_10ma)*-1.0:CALC_CURRENT_A(quccData.current_10ma);
	printf("전류소비량(raw unit: 10mA) : %d\n", quccData.current_10ma);
	printf("전류소비량(A) : %.3lf\n", quccInfo.current_a);
	// 잔여용량(raw unit: 10mAh)
	quccData.remaining_capacity_10mah = SWAP_2BYTE(quccData.remaining_capacity_10mah);
	quccInfo.remaining_capacity_ah = quccData.remaining_capacity_10mah / 100.0;
	printf("잔여용량(raw unit: 10mAh) : %d\n", quccData.remaining_capacity_10mah);
	printf("잔여용량(Ah) : %.3lf\n", quccInfo.remaining_capacity_ah);
	// 총용량(raw unit: 10mAh)
	quccData.norminal_capacity_10mah = SWAP_2BYTE(quccData.norminal_capacity_10mah);
	quccInfo.norminal_capacity_ah = quccData.norminal_capacity_10mah / 100.0;
	printf("총용량(raw unit: 10mAh) : %d\n", quccData.norminal_capacity_10mah);
	printf("총용량(Ah) : %.3lf\n", quccInfo.norminal_capacity_ah);
	// 사이클 횟수
	quccData.number_of_cycles = SWAP_2BYTE(quccData.number_of_cycles);
	quccInfo.cycles = quccData.number_of_cycles;
	printf("사이클 횟수 : %d\n", quccData.number_of_cycles);
	// 제조일
	quccData.production_date = SWAP_2BYTE(quccData.production_date);
	quccInfo.production_date = quccData.production_date;
	printf("제조일 : %d\n", quccData.production_date);
	// low balanced
	quccData.balanced_state = SWAP_2BYTE(quccData.balanced_state);
	quccInfo.balanced_low = quccData.balanced_state;
	printf("low balanced : %d\n", quccData.balanced_state);
	// high balanced
	quccData.balanced_state_high = SWAP_2BYTE(quccData.balanced_state_high);
	quccInfo.balanced_high = quccData.balanced_state_high;
	printf("high balanced : %d\n", quccData.balanced_state_high);
	// 보호 상태
	quccData.protection_status = SWAP_2BYTE(quccData.protection_status);
	quccInfo.protection_status = quccData.protection_status;
	printf("보호 상태 : %d\n", quccData.protection_status);
	// 소프트웨어버전
	quccInfo.software_version = quccData.software_version;
	printf("소프트웨어버전 : %d\n", quccData.software_version);
	// 잔여용량(%)
	quccInfo.remaining_capacity_percent = quccData.rsoc;
	printf("잔여용량(%%) : %d\n", quccData.rsoc);
	// MOS 상태
	quccInfo.fet_control_state = quccData.fet_control_state;
	printf("MOS 상태 : %d\n", quccData.fet_control_state);
	// 셀 수량
	quccInfo.number_of_battery_strings = quccData.number_of_battery_strings;
	printf("셀 수량 : %d\n", quccData.number_of_battery_strings);
	// 온도계 수량
	quccInfo.number_of_ntc = quccData.number_of_ntc;
	printf("온도계 수량 : %d\n", quccData.number_of_ntc);

	// 프로토콜과 상이함
#if 0
	#define ZERO_DEG_ADC 2731
	#define ADC_TO_DEG(x) ((x-ZERO_DEG_ADC)/10.0)
	// ntc1 온도
	quccData.ntc_1st = SWAP_2BYTE(quccData.ntc_1st);
	quccInfo.celsius_1st = ADC_TO_DEG(quccData.ntc_1st);
	printf("ntc1 : %d\n", quccData.ntc_1st);
	printf("온도1 : %.3lf\n", quccInfo.celsius_1st);
	// ntc2 온도
	quccData.ntc_2nd = SWAP_2BYTE(quccData.ntc_2nd);
	quccInfo.celsius_2nd = ADC_TO_DEG(quccData.ntc_2nd);
	printf("ntc2 : %d\n", quccData.ntc_2nd);
	printf("온도2 : %.3lf\n", quccInfo.celsius_2nd);
	// ntc3 온도
	quccData.ntc_3rd = SWAP_2BYTE(quccData.ntc_3rd);
	quccInfo.celsius_3rd = ADC_TO_DEG(quccData.ntc_3rd);
	printf("ntc3 : %d\n", quccData.ntc_3rd);
	printf("온도3 : %.3lf\n", quccInfo.celsius_3rd);
	// ntc4 온도
	quccData.ntc_4th = SWAP_2BYTE(quccData.ntc_4th);
	quccInfo.celsius_4th = ADC_TO_DEG(quccData.ntc_4th);
	printf("ntc4 : %d\n", quccData.ntc_4th);
	printf("온도4 : %.3lf\n", quccInfo.celsius_4th);
#endif

	return QUCC_RX_SUCCESS;
}

int isValidChecksum(uint8_t *recv) {
	// START 검사
	uint8_t qucc_rx_start_val;
	qucc_rx_start_val = recv[QUCC_RX_START_IDX];

	if (qucc_rx_start_val != QUCC_RX_START_VAL) {
		printf("invalid start: 0x%02x\n", qucc_rx_start_val);

		return QUCC_RX_INVALID;
	}

	// END 검사
	uint8_t qucc_rx_end_idx;
	qucc_rx_end_idx = recv[QUCC_RX_LENGTH_IDX] + (QUCC_RX_LENGTH_IDX+QUCC_RX_LENGTH_LEN) + QUCC_RX_CHECKSUM_LEN;
	uint8_t qucc_rx_end_val;
	qucc_rx_end_val = recv[qucc_rx_end_idx];

	if (qucc_rx_end_val != QUCC_RX_END_VAL) {
		printf("invalid end: 0x%02x\n", qucc_rx_end_val);

		return QUCC_RX_INVALID;
	}

	// CHECKSUM 검사
	uint16_t checksum;
	checksum = 0;
	uint16_t qucc_rx_checksum_byte_len;
	qucc_rx_checksum_byte_len = (QUCC_RX_STATUS_LEN+QUCC_RX_LENGTH_LEN+recv[QUCC_RX_LENGTH_IDX]);
	// printf("checksum len: 0x%x\n", qucc_rx_checksum_byte_len);
	for (int i=QUCC_RX_CHECKSUM_START_IDX; i<(QUCC_RX_CHECKSUM_START_IDX+qucc_rx_checksum_byte_len); i++) {
		checksum += recv[i];
	}
	checksum = ~checksum + 1;
	// printf("checksum: 0x%x\n", checksum);

	uint16_t qucc_rx_checksum_idx;
	qucc_rx_checksum_idx = recv[QUCC_RX_LENGTH_IDX] + (QUCC_RX_LENGTH_IDX+QUCC_RX_LENGTH_LEN);
	uint16_t qucc_rx_checksum_val;
	qucc_rx_checksum_val = *(uint16_t*)(recv + qucc_rx_checksum_idx);
	#if 0
	qucc_rx_checksum_val = (qucc_rx_checksum_val >> SHIFT_BIT) | (qucc_rx_checksum_val << SHIFT_BIT);
	#else
	qucc_rx_checksum_val = SWAP_2BYTE(qucc_rx_checksum_val);
	#endif

	if (checksum != qucc_rx_checksum_val) {
		printf("invalid checksum: 0x%x, qucc_rx_checksum_val: 0x%x\n", checksum, qucc_rx_checksum_val);
		return QUCC_RX_INVALID;
	}

	return QUCC_RX_VALID;
}

#if 1
int test() {
	uint8_t send[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
	// 프로토콜 예시데이터
	// uint8_t recv[] = {0xDD, 0x03, 0x00, 0x1B, 0x17, 0x00, 0x00, 0x00, 0x02, 0xD0, 0x03, 0xE8, 0x00, 0x00, 0x20, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x48, 0x03, 0x0F, 0x02, 0x0B, 0x76, 0x0B, 0x82, 0xFB, 0xFF, 0x77};
	// 실제데이터
	uint8_t recv[] = {0xDD, 0x03, 0x00, 0x1B, 0x0A, 0x69, 0x00, 0x00, 0x0A, 0xF0, 0x0C, 0xC9, 0x00, 0x02, 0x2C, 0xED, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x56, 0x01, 0x08, 0x02, 0x0B, 0xAC, 0x0B, 0xAF, 0xFA, 0x8E, 0x77};
	// 프로토콜 설명데이터
	// uint8_t recv[] = {0xDD, 0x03, 0x00, 0x1B, 0x19, 0xDF, 0xF8, 0x24, 0x0D, 0xA5, 0x0F, 0xA0, 0x00, 0x02, 0x24, 0x91, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x57, 0x11, 0x04, 0x02, 0x98, 0x0B, 0xA9, 0x0B, 0xF9, 0xE2, 0x77};

	if (isValidChecksum(recv)) {
		if (recv[QUCC_RX_STATUS_IDX] == 0x00) {
			QuccData quccData;
			memcpy((uint8_t*)(&quccData), recv+QUCC_RX_DATA_IDX, recv[QUCC_RX_LENGTH_IDX]);
			parseRxData(quccData);
		} else {
			printf("QUCC_RX_STATUS_VAL is not ZERO : 0x%x\n", recv[QUCC_RX_STATUS_IDX]);
		}
	}

	return 0;
}
#endif

#if 1
int main() {
	test();

	return 0;
}
#endif