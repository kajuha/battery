#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <string.h>

#include <iostream>
#include <queue>

#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>

#include "qucc.h"

Qucc::Qucc() {
}

Qucc::Qucc(rclcpp::Node::SharedPtr& node, std::string serialPort, int baudrate) {
	this->node = node;
	this->serialPort = serialPort;
	this->baudrate = baudrate;
	this->isParsed = false;
	this->ser = new serial::Serial();
}

bool Qucc::initSerial()	{
	const char* COMM_PORT = serialPort.c_str();

	ser->setPort(serialPort);
	ser->setBaudrate(baudrate);
	#define SERIAL_TIMEOUT_MS 3000
	serial::Timeout to = serial::Timeout::simpleTimeout(SERIAL_TIMEOUT_MS);
	ser->setTimeout(to);

	ser->open();

	if (!ser->isOpen()) {
		RCLCPP_INFO(node->get_logger(), "error opening port[%s] baudrate[%d]", COMM_PORT, baudrate);
		RCLCPP_INFO(node->get_logger(), "you may need to have ROOT access");
		return false;
	}

	ser->flush();

	RCLCPP_INFO(node->get_logger(), "qucc communication port is ready");

	return true;
}

void Qucc::closeSerial() {
	ser->close();
	
	RCLCPP_INFO(node->get_logger(), "closing qucc");
}

bool Qucc::sendQuccCmd() {
	// 송신 포맷 생성
	serialBufferTx[QUCC_TX_START_IDX] = QUCC_TX_START_VAL;
	serialBufferTx[QUCC_TX_STATUS_IDX] = QUCC_TX_STATUS_VAL;
	serialBufferTx[QUCC_TX_COMMAND_IDX] = QUCC_TX_COMMAND_VAL;
	serialBufferTx[QUCC_TX_DATA_IDX] = QUCC_TX_DATA_VAL;
	// CHECKSUM
	*(uint16_t*)(serialBufferTx+QUCC_TX_CHECKSUM_IDX) = QUCC_TX_CHECKSUM_VAL;
	serialBufferTx[QUCC_TX_END_IDX] = QUCC_TX_END_VAL;

	ser->write(serialBufferTx, QUCC_TX_LEN);

	return true;
}

bool Qucc::receiveQuccState(bool enableParsing) {
	static int rx_size;

	memset(serialBufferRx, '\0', sizeof(serialBufferRx));

	rx_size = ser->available();
	if (rx_size) {
		rx_size = ser->read(serialBufferRx, rx_size);
	}

	for (int i=0; i<rx_size; i++) {
		queSerialRx.push(serialBufferRx[i]);
	}

	if (enableParsing) {
		parseQuccState();
	} else {
		if (queSerialRx.size()) {
			static std::string tmpBufString;
			static char tmpBuffer[BUFSIZ];
			sprintf(tmpBuffer, "RxBuffer:");
			tmpBufString = tmpBuffer;
			for (int i=0; i<rx_size; i++) {
				sprintf(tmpBuffer, "[%02x]", queSerialRx.front());
				tmpBufString += tmpBuffer;
				queSerialRx.pop();
			}
			RCLCPP_INFO(node->get_logger(), "%s", tmpBufString.c_str());
		}
	}

	return true;
}

bool Qucc::parseQuccState() {
	static int state = FSM_QUCC_RX::START;
	static uint8_t recv[BUFSIZ] = {'\0', };
	static uint8_t qucc_rx_checksum_idx = 0;

	switch (state) {
		case FSM_QUCC_RX::START:
			if (queSerialRx.size() >= QUCC_RX_START_LEN) {
				recv[QUCC_RX_START_IDX] = queSerialRx.front();
				if (recv[QUCC_RX_START_IDX] == QUCC_RX_START_VAL) {
					state = FSM_QUCC_RX::COMMAND;
				} else {
					RCLCPP_INFO(node->get_logger(), "FSM_QUCC_RX::START not Match");
					state = FSM_QUCC_RX::START;
				}
				queSerialRx.pop();
			}
			break;
		case FSM_QUCC_RX::COMMAND:
			if (queSerialRx.size() >= QUCC_RX_COMMAND_LEN) {
				recv[QUCC_RX_COMMAND_IDX] = queSerialRx.front();
				if (recv[QUCC_RX_COMMAND_IDX] == QUCC_RX_COMMAND_VAL) {
					state = FSM_QUCC_RX::STATUS;
				} else {
					RCLCPP_INFO(node->get_logger(), "FSM_QUCC_RX::COMMAND not Match");
					state = FSM_QUCC_RX::COMMAND;
				}
				queSerialRx.pop();
			}
			break;
		case FSM_QUCC_RX::STATUS:
			if (queSerialRx.size() >= QUCC_RX_STATUS_LEN) {
				recv[QUCC_RX_STATUS_IDX] = queSerialRx.front();
				if (recv[QUCC_RX_STATUS_IDX] == QUCC_RX_STATUS_VAL) {
					state = FSM_QUCC_RX::LENGTH;
				} else {
					RCLCPP_INFO(node->get_logger(), "FSM_QUCC_RX::STATUS not Match");
					state = FSM_QUCC_RX::STATUS;
				}
				queSerialRx.pop();
			}
			break;
		case FSM_QUCC_RX::LENGTH:
			if (queSerialRx.size() >= QUCC_RX_LENGTH_LEN) {
				recv[QUCC_RX_LENGTH_IDX] = queSerialRx.front();
				#if 0
				if (recv[QUCC_RX_LENGTH_IDX] == QUCC_RX_LENGTH_VAL) {
					state = FSM_QUCC_RX::DATA;
				} else {
					RCLCPP_INFO(node->get_logger(), "FSM_QUCC_RX::LENGTH not Match");
					state = FSM_QUCC_RX::LENGTH;
				}
				#else
				QUCC_RX_LENGTH_VAL = recv[QUCC_RX_LENGTH_IDX];
				QUCC_RX_DATA_LEN = QUCC_RX_LENGTH_VAL;
				QUCC_RX_CHECKSUM_IDX = (QUCC_RX_DATA_IDX+QUCC_RX_DATA_LEN);
				QUCC_RX_CHECKSUM_BYTE_LEN = (QUCC_RX_STATUS_LEN+QUCC_RX_LENGTH_LEN+QUCC_RX_DATA_LEN);
				state = FSM_QUCC_RX::DATA;
				#endif
				queSerialRx.pop();
			}
			break;
		case FSM_QUCC_RX::DATA:
			if (queSerialRx.size() >= QUCC_RX_LENGTH_VAL) {
				for (int i=0; i<QUCC_RX_LENGTH_VAL; i++) {
					recv[QUCC_RX_DATA_IDX+i] = queSerialRx.front();
					queSerialRx.pop();
				}

				state = FSM_QUCC_RX::CHECKSUM;
			}
			break;
		case FSM_QUCC_RX::CHECKSUM:
			if (queSerialRx.size() >= QUCC_RX_CHECKSUM_LEN) {
				for (int i=0; i<QUCC_RX_CHECKSUM_LEN; i++) {
					recv[QUCC_RX_CHECKSUM_IDX+i] = queSerialRx.front();
					queSerialRx.pop();
				}

				state = FSM_QUCC_RX::END;
			}
			break;
		case FSM_QUCC_RX::END:
			if (queSerialRx.size() >= QUCC_RX_END_LEN) {
				recv[QUCC_RX_END_IDX] = queSerialRx.front();
				if (recv[QUCC_RX_END_IDX] == QUCC_RX_END_VAL) {
					state = FSM_QUCC_RX::OK;
				} else {
					RCLCPP_INFO(node->get_logger(), "FSM_QUCC_RX::END not Match");
					for (int i=0; i<QUCC_RX_LEN; i++) {
						RCLCPP_INFO(node->get_logger(), "[%02x]", recv[i]);
					}
					RCLCPP_INFO(node->get_logger(), "");
					state = FSM_QUCC_RX::START;
				}
				queSerialRx.pop();
			}
			break;
		case FSM_QUCC_RX::OK:
			if (isValidChecksum(recv)) {
				if (recv[QUCC_RX_STATUS_IDX] == 0x00) {
					#if 0
					memcpy((uint8_t*)(&_quccData), recv+QUCC_RX_DATA_IDX, recv[QUCC_RX_LENGTH_IDX]);
					#else
					memcpy((uint8_t*)(&_quccData), recv+QUCC_RX_DATA_IDX, sizeof(_quccData));
					#endif
					_quccInfo = parseRxData(_quccData);
				} else {
					RCLCPP_INFO(node->get_logger(), "QUCC_RX_STATUS_VAL is not ZERO : 0x%x", recv[QUCC_RX_STATUS_IDX]);
				}
			}

			memset(recv, '\0', QUCC_RX_LEN);
			
			state = FSM_QUCC_RX::START;

			break;
		default:
			state = FSM_QUCC_RX::START;

			break;
	}
	
	return false;
}

QuccInfo Qucc::parseRxData(QuccData quccData) {
	static QuccInfo quccInfo;

#if 1
	// 총전압(raw unit: 10mV)
	quccData.total_voltage_10mv = SWAP_2BYTE(quccData.total_voltage_10mv);
	quccInfo.voltage_v = quccData.total_voltage_10mv / 100.0;
	// 전류소비량(raw unit: 10mA)
	quccData.current_10ma = SWAP_2BYTE(quccData.current_10ma);
	#define MSB_DISCHARGE 15
	#define IS_DISCHARGE(x) ((x>>MSB_DISCHARGE) & 0x0001)
	#define CALC_CURRENT_A(x) ((65536.0-(double)x)/100.0)
	if (quccData.current_10ma) {
		quccInfo.current_a = IS_DISCHARGE(quccData.current_10ma)?CALC_CURRENT_A(quccData.current_10ma)*-1.0:(quccData.current_10ma*10.0)/1000.0;
	} else {
		quccInfo.current_a = 0.0;
	}
	// 잔여용량(raw unit: 10mAh)
	quccData.remaining_capacity_10mah = SWAP_2BYTE(quccData.remaining_capacity_10mah);
	quccInfo.remaining_capacity_ah = quccData.remaining_capacity_10mah / 100.0;
	// 총용량(raw unit: 10mAh)
	quccData.norminal_capacity_10mah = SWAP_2BYTE(quccData.norminal_capacity_10mah);
	quccInfo.norminal_capacity_ah = quccData.norminal_capacity_10mah / 100.0;
	// 사이클 횟수
	quccData.number_of_cycles = SWAP_2BYTE(quccData.number_of_cycles);
	quccInfo.cycles = quccData.number_of_cycles;
	// 제조일
	quccData.production_date = SWAP_2BYTE(quccData.production_date);
	quccInfo.production_date = quccData.production_date;
	// low balanced
	quccData.balanced_state = SWAP_2BYTE(quccData.balanced_state);
	quccInfo.balanced_low = quccData.balanced_state;
	// high balanced
	quccData.balanced_state_high = SWAP_2BYTE(quccData.balanced_state_high);
	quccInfo.balanced_high = quccData.balanced_state_high;
	// 보호 상태
	quccData.protection_status = SWAP_2BYTE(quccData.protection_status);
	quccInfo.protection_status = quccData.protection_status;
	// 소프트웨어버전
	quccInfo.software_version = quccData.software_version;
	// 잔여용량(%)
	quccInfo.remaining_capacity_percent = quccData.rsoc;
	// MOS 상태
	#define BIT_CHARGING 0
	#define BIT_DISCHARGING 1
	#define IS_CHARGING(x) ((x>>BIT_CHARGING) & 0x01)
	#define IS_DISCHARGING(x) ((x>>BIT_DISCHARGING) & 0x01)
	quccInfo.charging = IS_CHARGING(quccData.fet_control_state);
	quccInfo.discharging = IS_DISCHARGING(quccData.fet_control_state);
	// 셀 수량
	quccInfo.number_of_battery_strings = quccData.number_of_battery_strings;
	// 온도계 수량
	quccInfo.number_of_ntc = quccData.number_of_ntc;

	// 프로토콜과 상이함
#if 0
	#define ZERO_DEG_ADC 2731
	#define ADC_TO_DEG(x) ((x-ZERO_DEG_ADC)/10.0)
	// ntc1 온도
	quccData.ntc_1st = SWAP_2BYTE(quccData.ntc_1st);
	if (quccData.ntc_1st) {
		quccInfo.celsius_1st = ADC_TO_DEG(quccData.ntc_1st);
	} else {
		quccInfo.celsius_1st = 0.0;
	}
	// ntc2 온도
	quccData.ntc_2nd = SWAP_2BYTE(quccData.ntc_2nd);
	if (quccData.ntc_2nd) {
		quccInfo.celsius_2nd = ADC_TO_DEG(quccData.ntc_2nd);
	} else {
		quccInfo.celsius_2nd = 0.0;
	}
	// ntc3 온도
	quccData.ntc_3rd = SWAP_2BYTE(quccData.ntc_3rd);
	if (quccData.ntc_3rd) {
		quccInfo.celsius_3rd = ADC_TO_DEG(quccData.ntc_3rd);
	} else {
		quccInfo.celsius_3rd = 0.0;
	}
	// ntc4 온도
	quccData.ntc_4th = SWAP_2BYTE(quccData.ntc_4th);
	if (quccData.ntc_4th) {
		quccInfo.celsius_4th = ADC_TO_DEG(quccData.ntc_4th);
	} else {
		quccInfo.celsius_4th = 0.0;
	}
#endif
#endif

	#if 0
	RCLCPP_INFO(node->get_logger(), "총전압(raw unit: 10mV) : %d", quccData.total_voltage_10mv);
	RCLCPP_INFO(node->get_logger(), "총전압(V) : %.3lf", quccInfo.voltage_v);
	RCLCPP_INFO(node->get_logger(), "전류(raw unit: 10mA) : %d", quccData.current_10ma);
	RCLCPP_INFO(node->get_logger(), "전류(A) : %.3lf", quccInfo.current_a);
	RCLCPP_INFO(node->get_logger(), "잔여용량(raw unit: 10mAh) : %d", quccData.remaining_capacity_10mah);
	RCLCPP_INFO(node->get_logger(), "잔여용량(Ah) : %.3lf", quccInfo.remaining_capacity_ah);
	RCLCPP_INFO(node->get_logger(), "총용량(raw unit: 10mAh) : %d", quccData.norminal_capacity_10mah);
	RCLCPP_INFO(node->get_logger(), "총용량(Ah) : %.3lf", quccInfo.norminal_capacity_ah);
	RCLCPP_INFO(node->get_logger(), "사이클 횟수 : %d", quccData.number_of_cycles);
	RCLCPP_INFO(node->get_logger(), "제조일 : %d", quccData.production_date);
	RCLCPP_INFO(node->get_logger(), "low balanced : %d", quccData.balanced_state);
	RCLCPP_INFO(node->get_logger(), "high balanced : %d", quccData.balanced_state_high);
	RCLCPP_INFO(node->get_logger(), "보호 상태 : %d", quccData.protection_status);
	RCLCPP_INFO(node->get_logger(), "소프트웨어버전 : %d", quccData.software_version);
	RCLCPP_INFO(node->get_logger(), "잔여용량(%%) : %d", quccData.rsoc);
	RCLCPP_INFO(node->get_logger(), "MOS 상태 : %d", quccData.fet_control_state);
	RCLCPP_INFO(node->get_logger(), "충전 : %d, 방전 : %d", quccInfo.charging, quccInfo.discharging);
	RCLCPP_INFO(node->get_logger(), "셀 수량 : %d", quccData.number_of_battery_strings);
	RCLCPP_INFO(node->get_logger(), "온도계 수량 : %d", quccData.number_of_ntc);
	RCLCPP_INFO(node->get_logger(), "ntc1 : %d", quccData.ntc_1st);
	RCLCPP_INFO(node->get_logger(), "온도1 : %.3lf", quccInfo.celsius_1st);
	RCLCPP_INFO(node->get_logger(), "ntc2 : %d", quccData.ntc_2nd);
	RCLCPP_INFO(node->get_logger(), "온도2 : %.3lf", quccInfo.celsius_2nd);
	RCLCPP_INFO(node->get_logger(), "ntc3 : %d", quccData.ntc_3rd);
	RCLCPP_INFO(node->get_logger(), "온도3 : %.3lf", quccInfo.celsius_3rd);
	RCLCPP_INFO(node->get_logger(), "ntc4 : %d", quccData.ntc_4th);
	RCLCPP_INFO(node->get_logger(), "온도4 : %.3lf", quccInfo.celsius_4th);
	#endif

	isParsed = true;

	return quccInfo;
}

int Qucc::isValidChecksum(uint8_t* recv) {
	// START 검사
	uint8_t qucc_rx_start_val;
	qucc_rx_start_val = recv[QUCC_RX_START_IDX];

	if (qucc_rx_start_val != QUCC_RX_START_VAL) {
		RCLCPP_INFO(node->get_logger(), "invalid start: 0x%02x", qucc_rx_start_val);

		return QUCC_RX_INVALID;
	}

	// END 검사
	uint8_t qucc_rx_end_idx;
	qucc_rx_end_idx = recv[QUCC_RX_LENGTH_IDX] + (QUCC_RX_LENGTH_IDX+QUCC_RX_LENGTH_LEN) + QUCC_RX_CHECKSUM_LEN;
	uint8_t qucc_rx_end_val;
	qucc_rx_end_val = recv[qucc_rx_end_idx];

	if (qucc_rx_end_val != QUCC_RX_END_VAL) {
		RCLCPP_INFO(node->get_logger(), "invalid end: 0x%02x", qucc_rx_end_val);

		return QUCC_RX_INVALID;
	}

	// CHECKSUM 검사
	uint16_t checksum;
	checksum = 0;
	uint16_t qucc_rx_checksum_byte_len;
	#if 0
	qucc_rx_checksum_byte_len = (QUCC_RX_STATUS_LEN+QUCC_RX_LENGTH_LEN+recv[QUCC_RX_LENGTH_IDX]);
	#else
	qucc_rx_checksum_byte_len = (QUCC_RX_STATUS_LEN+QUCC_RX_LENGTH_LEN+QUCC_RX_LENGTH_VAL);
	#endif
	for (int i=QUCC_RX_CHECKSUM_START_IDX; i<(QUCC_RX_CHECKSUM_START_IDX+qucc_rx_checksum_byte_len); i++) {
		checksum += recv[i];
	}
	checksum = ~checksum + 1;

	uint16_t qucc_rx_checksum_idx;
	#if 0
	qucc_rx_checksum_idx = recv[QUCC_RX_LENGTH_IDX] + (QUCC_RX_LENGTH_IDX+QUCC_RX_LENGTH_LEN);
	#else
	qucc_rx_checksum_idx = QUCC_RX_LENGTH_VAL + QUCC_RX_LENGTH_IDX + QUCC_RX_LENGTH_LEN;
	#endif
	uint16_t qucc_rx_checksum_val;
	qucc_rx_checksum_val = *(uint16_t*)(recv + qucc_rx_checksum_idx);
	#if 0
	qucc_rx_checksum_val = (qucc_rx_checksum_val >> SHIFT_BIT) | (qucc_rx_checksum_val << SHIFT_BIT);
	#else
	qucc_rx_checksum_val = SWAP_2BYTE(qucc_rx_checksum_val);
	#endif

	if (checksum != qucc_rx_checksum_val) {
		RCLCPP_INFO(node->get_logger(), "invalid checksum: 0x%x, qucc_rx_checksum_val: 0x%x", checksum, qucc_rx_checksum_val);
		return QUCC_RX_INVALID;
	}

	return QUCC_RX_VALID;
}