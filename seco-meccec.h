/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
 * CEC driver for SECO MEC-based Boards
 *
 * Author:  Ettore Chimenti <ek5.chimenti@gmail.com>
 * Copyright (C) 2022, SECO SpA.
 */

/* Configuration access definitions */
/* Not useful now
 *#define EC_CONFIG_INDEX     MEC_CONFIG_BAR
 *#define EC_CONFIG_DATA      ( EC_CONFIG_INDEX + 1 )
 *#define EC_CONFIG_UNLOCK    MEC_CONFIG_UNLOCK
 *#define EC_CONFIG_LOCK      MEC_CONFIG_LOCK
 */
/*
 * /// ID
 *#define EC_CONFIG_ID        0x20
 *#define EC_CONFIG_REV       0x21
 *#define EC_CONFIG_BAD_ID1   0
 *#define EC_CONFIG_BAD_ID2   0xffff
 */
/*
 * /// MBX BAR
 *#define EC_CONFIG_MBX_LOW   ( EC_CONFIG_MBX_ADDR + 2 )
 *#define EC_CONFIG_MBX_HIGH  ( EC_CONFIG_MBX_LOW + 1 )
 */

/*
 * /// Resource BAR
 *#define EC_CONFIG_RES_LOW   ( EC_CONFIG_RES_ADDR + 2 )
 *#define EC_CONFIG_RES_HIGH  ( EC_CONFIG_RES_LOW + 1 )
 */

/* Mail Box  definitions */
#define EC_REGISTER_INDEX   MBX_ACCESS_BAR
#define EC_REGISTER_DATA    (EC_REGISTER_INDEX + 1)

/* ACPI compatible format (no enum, struct, union...) */

/* ESPI/LPC definitions */
#define MBX_RESERVED_SIZE        0x10
#define MBX_RESERVED_BASE        0x2b0
#define MBX_BASE_FROM_CFG_BAR(x) (x - CFG_BAR_OFFSET)

/* Mail Box  definitions */
#define RES_BAR_OFFSET           0
#define RES_BAR_FROM_MBX_BASE(x) (x + RES_BAR_OFFSET)
#define MBX_RESOURCE_REGISTER    RES_BAR_FROM_MBX_BASE(MBX_RESERVED_BASE)
#define MBX_BAR_OFFSET           0xc
#define MBX_BAR_FROM_MBX_BASE(x) (x + MBX_BAR_OFFSET)
#define MBX_ACCESS_BAR           MBX_BAR_FROM_MBX_BASE(MBX_RESERVED_BASE)
#define EC_MBX_SIZE              0x20

#define EC_COMMAND_REGISTER      0
#define EC_RESULT_REGISTER       1
#define EC_STATUS_REGISTER       2
#define EC_MASK_REGISTER         3
#define EC_MBX_REGISTER          0x10

/* Software definitions */
#define EC_CMD_TIMEOUT      0x30000 //  Maximum wait loop

/**
 * @brief GET_FIRMWARE_VERSION_CMD data struct and definitions
 */
#define FIRMWARE_TIME_STAMP_SIZE (EC_MBX_SIZE - sizeof(uint32_t))

struct version_t {
	u8 minor;
	u8 major;
};

struct version_msg_t {
	struct version_t fw;
	struct version_t lib;
	u8 firmware_ts[FIRMWARE_TIME_STAMP_SIZE];
};

/**
 * @brief CEC data struct and constant definitions
 */
#define MECCEC_MAX_MSG_SIZE 16

struct seco_meccec_tx_t {
	u8 bus;
	u8 send;
	u8 dest;
	u8 data[MECCEC_MAX_MSG_SIZE];
	u8 size;
};

struct seco_meccec_rx_t {
	u8 bus;
	u8 send;
	u8 dest;
	u8 data[MECCEC_MAX_MSG_SIZE];
	u8 size;
};

struct seco_meccec_logaddr_t {
	u8 bus;
	u8 addr;
};


struct seco_meccec_status_t {
	u8 status_ch0;
	u8 status_ch1;
	u8 status_ch2;
	u8 status_ch3;
};

/*
 * Status data
 */

#define SECOCEC_STATUS_MSG_RECEIVED_MASK	BIT(0)
#define SECOCEC_STATUS_RX_ERROR_MASK		BIT(1)
#define SECOCEC_STATUS_MSG_SENT_MASK		BIT(2)
#define SECOCEC_STATUS_TX_ERROR_MASK		BIT(3)

#define SECOCEC_STATUS_TX_NACK_ERROR		BIT(4)
#define SECOCEC_STATUS_RX_OVERFLOW_MASK		BIT(5)

/**
 * @brief   MBX Status bitmap values from E.C. to Host.
 * @note    Refer to Microchip MEC170x DS00002206D data-sheet for naming and
 *          description.
 * @details The bitmap refers to the data byte of the ACPI-ECI EC0 device, plus
 *          bit 0 for init/deinit. All the agents will have an QUEUED and an
 *          ACTIVE bit to register the activity; this allows at most 4 distinct
 *          agents: see the macros following this enum definition.
 */
enum MBX_STATUS {
	MBX_OFF     = 0,          /*!< Disable MBX Interface */
	MBX_ON      = 1,          /*!< Enable MBX Interface  */
	MBX_ACTIVE0 = (1 << 6), /*!< MBX AGENT 0 active    */
	MBX_QUEUED0 = (1 << 7), /*!< MBX AGENT 0 idle      */
};

#define AGENT_IDLE(x)      0
#define AGENT_QUEUED(x)    (MBX_QUEUED0 >> (2 * x))
#define AGENT_ACTIVE(x)    (MBX_ACTIVE0 >> (2 * x))
#define AGENT_MASK(x)      (AGENT_QUEUED(x) + AGENT_ACTIVE(x))
#define AGENT_DONE(x)      AGENT_MASK(x)
#define MBX_STATUS_DEFAULT 0

/**
 * @brief   MBX user IDs.
 * @note    Used to keep track of the current agent.
 * @details As explained in the definition of MBX_STATUS above, AGENT_NONE must
 *          be at most 4.
 */
enum AGENT_IDS {
	AGENT_BIOS, /*!< BIOS AGENT */
	AGENT_ACPI, /*!< ACPI AGENT */
	AGENT_EAPI, /*!< EAPI AGENT */
	AGENT_USER, /*!< USER AGENT */
	AGENT_NONE, /*!< No AGENT   */
};

/**
 * @brief MBX command return codes: 0 = success.
 */
enum CMD_RESULT {
	EC_NO_ERROR = 0,           /*!< Success          */
	EC_UNKNOWN_COMMAND_ERROR,  /*!< Unknown command  */
	EC_INVALID_ARGUMENT_ERROR, /*!< Invalid argument */
	EC_TIMEOUT_ERROR,          /*!< Waiting Time-out */
	EC_DEVICE_ERROR,           /*!< Device error     */
};

/**
 * @brief   MBX commands.
 * @note    Maintained accordingly to corresponding library modules.
 * @details The type is uint8_t, so at most 0x100 values starting from 0.
 *          REQUEST_MBX_ACCESS_CMD is the first not hookable command, so it is
 *          used also as size in the command handlers table; this is why all
 *          the other not hookable commands must be greater. There is one
 *          request and one release access command for each agent: see the
 *          macros following this enum definition; this is why there must be at
 *          least AGENT_NONE long holes after REQUEST_MBX_ACCESS_CMD and
 *          RELEASE_MBX_ACCESS_CMD.
 */
enum MBX_CMDS {
	REQUEST_MBX_ACCESS_CMD   = 0xf0, /*!< First request access command           */
	RELEASE_MBX_ACCESS_CMD   = 0xf8, /*!< First release access command           */
	GET_FIRMWARE_VERSION_CMD = 0,    /*!< Get firmware version record            */
	ASSERT_SYSTEM_RESET_CMD  = 1,    /*!< Reset the Embedded Controller          */
	RSTBTN_RESET_CMD         = 2,    /*!< Reset the Host with Reset Button       */
	PWRBTN_SHUTDOWN_CMD      = 3,    /*!< Shutdown the Host with Power Button 4s */
	GET_STATUS_REGISTER      = 4,    /*!< Read custom status register            */
	CLR_STATUS_REGISTER      = 5,    /*!< Write/Clear custom status register     */
	GET_ENABLE_REGISTER      = 6,    /*!< Read custom enable register            */
	SET_ENABLE_REGISTER      = 7,    /*!< Write/Set custom enable register       */
	CLR_ENABLE_REGISTER      = 8,    /*!< Write/Clear custom enable register     */
	SET_FEATURE_VARIABLE     = 9,    /*!< Write flavor feature variable          */
	GET_FEATURE_VARIABLE     = 0xa,  /*!< Read flavor feature variable           */
	GET_UART_CONFIG          = 0xb,  /*!< Read current UART configuration        */
	SET_UART_CONFIG          = 0xc,  /*!< Write UART configuration               */
	ACCESS_I2C_CMD           = 0x10, /*!< Access an I2C channel                  */
	GET_EEPROM_CAPACITY_CMD  = 0x20, /*!< Get EEPROM total capacity in bytes     */
	GET_EEPROM_RESERVED_CMD  = 0x21, /*!< Get EEPROM reserved bytes              */
	EEPROM_READ_CMD          = 0x22, /*!< Read bytes from EEPROM                 */
	EEPROM_WRITE_CMD         = 0x23, /*!< Write bytes to EEPROM                  */
	UTC_READ_CMD             = 0x24, /*!< Read Up Time Counter                   */
	UTC_WRITE_CMD            = 0x25, /*!< Write Up Time Counter                  */
	GET_AFTER_G3_CACHE       = 0x26, /*!< Get previous after G3 state            */
	GET_AFTER_G3_STATE       = 0x27, /*!< Get current after G3 configuration     */
	GET_LAST_STATE           = 0x28, /*!< Get last power state detected          */
	SET_AFTER_G3_STATE       = 0x29, /*!< Set after G3 configuration             */
	GET_RESET_CAUSES_CMD     = 0x2a, /*!< Get reset causes since last clearing   */
	CLEAR_RESET_CAUSES_CMD   = 0x2b, /*!< Clear reset causes                     */
	SET_PORT_80_DEBUG_CMD    = 0x2c, /*!< Set Port 80 on serial debug features   */
	FAN_RPM_READ_CMD         = 0x30, /*!< Read FAN RPM speed                     */
	FAN_DC_READ_CMD          = 0x31, /*!< Read FAN Duty Cycle                    */
	FAN_DC_WRITE_CMD         = 0x32, /*!< Write FAN Duty Cycle                   */
	FAN_FREQ_READ_CMD        = 0x33, /*!< Read FAN Duty Frequency                */
	FAN_FREQ_WRITE_CMD       = 0x34, /*!< Write FAN Duty Frequency               */
	FAN_TYPE_WRITE_CMD       = 0x35, /*!< Write FAN type                         */
	FAN_TEMP_READ_CMD        = 0x36, /*!< Read FAN Regulating Temperature        */
	FAN_TEMP_WRITE_CMD       = 0x37, /*!< Write FAN Regulating Temperature       */
	FAN_THERM_MGMT_CFG_CMD   = 0x38, /*!< Write FAN Regulating Parameters        */
	FAN_RAMP_READ_CMD        = 0x39, /*!< Read FAN speed change duration         */
	FAN_RAMP_WRITE_CMD       = 0x3a, /*!< Write FAN speed change duration        */
	FAN_ENABLE_CFG_CMD       = 0x3b, /*!< Enable and configure FAN               */
	FAN_DISABLE_CMD          = 0x3c, /*!< Full FAN Disabling                     */
	SPI_ACCESS_CMD           = 0x40, /*!< Access of SPI Channel                  */
	GPIO_ACCESS_CMD          = 0x50, /*!< Access of GPIO PIN                     */
	WDT_CONFIG_CMD           = 0x60, /*!< WDT configuration                      */
	WDT_START_CMD            = 0x61, /*!< WDT start                              */
	WDT_REFRESH_CMD          = 0x62, /*!< WDT software refresh                   */
	WDT_STOP_CMD             = 0x63, /*!< WDT stop                               */
	ACCESS_ADC_CMD           = 0x70, /*!< Access an ADC channel                  */
	ACCESS_RCID_CMD          = 0x71, /*!< Access RC_ID                           */
	CEC_WRITE_CMD		 = 0x72, /*!< Write CEC command                      */
	CEC_READ_CMD		 = 0x73, /*!< Read CEC command                       */
	GET_CEC_STATUS_CMD	 = 0x74, /*!< Get CEC status regisers                */
	SET_CEC_LOGADDR_CMD	 = 0x75, /*!< Set CEC Logical Address                */
};

#define REQUEST_MBX_ACCESS(x) (REQUEST_MBX_ACCESS_CMD + x)
#define RELEASE_MBX_ACCESS(x) (RELEASE_MBX_ACCESS_CMD + x)
