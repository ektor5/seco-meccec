// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * CEC driver for SECO MEC-based Boards
 *
 * Author:  Ettore Chimenti <ek5.chimenti@gmail.com>
 * Copyright (C) 2022, SECO SpA.
 */

#pragma pack(push, 1)
/* Configuration access definitions */
#define EC_CONFIG_INDEX     MEC_CONFIG_BAR
#define EC_CONFIG_DATA      ( EC_CONFIG_INDEX + 1 )
#define EC_CONFIG_UNLOCK    MEC_CONFIG_UNLOCK
#define EC_CONFIG_LOCK      MEC_CONFIG_LOCK
/// Logical Device Number
#define EC_CONFIG_LDN       7
/// ID
#define EC_CONFIG_ID        0x20
#define EC_CONFIG_REV       0x21
#define EC_CONFIG_BAD_ID1   0
#define EC_CONFIG_BAD_ID2   0xffff
/// MBX BAR
#define EC_CONFIG_MBX_LOW   ( EC_CONFIG_MBX_ADDR + 2 )
#define EC_CONFIG_MBX_HIGH  ( EC_CONFIG_MBX_LOW + 1 )
/// Resource BAR
#define EC_CONFIG_RES_LOW   ( EC_CONFIG_RES_ADDR + 2 )
#define EC_CONFIG_RES_HIGH  ( EC_CONFIG_RES_LOW + 1 )

/* Mail Box  definitions */
#define EC_REGISTER_INDEX   MBX_ACCESS_BAR
#define EC_REGISTER_DATA    ( EC_REGISTER_INDEX + 1 )

/* ACPI compatible format (no enum, struct, union...) */

/* ESPI/LPC definitions */
#define MBX_RESERVED_SIZE        0x10
#define MBX_RESERVED_BASE        0x2b0
#define CFG_BAR_OFFSET           0x0e
#define CFG_BAR_FROM_MBX_BASE(x) ( x + CFG_BAR_OFFSET )
#define MBX_BASE_FROM_CFG_BAR(x) ( x - CFG_BAR_OFFSET )
#define MEC_CONFIG_BAR           CFG_BAR_FROM_MBX_BASE(MBX_RESERVED_BASE)
#define MEC_CONFIG_UNLOCK        0x55
#define MEC_CONFIG_LOCK          0xAA

#define MEC_REG_LDEV             0x07    // Logical Device Register
#define MEC_DID_COM1           	 0x09    // Device ID of Com1
#define MEC_DID_COM2           	 0x0A    // Device ID of Com2
#define MEC_DID_LPC            	 0x0C    // Device ID of LPC Interface
#define MEC_REG_UART_EN          0x30    // UART Enable Register
#define MEC_REG_LPC_SIRQ         0x40    // LPC Serial IRQ configuration register base
#define MEC_REG_IOBAR_UART0      0x88    // I/O BASE ADDRESS REGISTER for UART0
#define MEC_REG_IOBAR_UART1      0x8C    // I/O BASE ADDRESS REGISTER for UART1


/* Mail Box  definitions */
#define RES_BAR_OFFSET           0
#define RES_BAR_FROM_MBX_BASE(x) ( x + RES_BAR_OFFSET )
#define MBX_RESOURCE_REGISTER    RES_BAR_FROM_MBX_BASE(MBX_RESERVED_BASE)
#define MBX_BAR_OFFSET           0xc
#define MBX_BAR_FROM_MBX_BASE(x) ( x + MBX_BAR_OFFSET )
#define MBX_ACCESS_BAR           MBX_BAR_FROM_MBX_BASE(MBX_RESERVED_BASE)
#define EC_MBX_SIZE              0x20

#define EC_COMMAND_REGISTER      0
#define EC_RESULT_REGISTER       1
#define EC_STATUS_REGISTER       2
#define EC_MASK_REGISTER         3
#define EC_MBX_REGISTER          0x10

/* Setup values passed directly to MEC*/
// Watchdog
#define WDT_EVENT_NONE           0
#define WDT_EVENT_PWRBTN_PULSE   1
#define WDT_EVENT_PWRBTN_FORCE   2
#define WDT_EVENT_RESET          3

// After G3 management
/// EEPROM area bitmap
#define AFTER_G3_SHIFT    0
#define AFTER_G3_MASK     (0xF << AFTER_G3_SHIFT )
#define LAST_STATE_SHIFT  4
#define LAST_STATE_MASK   (0xF << LAST_STATE_SHIFT)
/// Possible values for after G3 behaviour
#define AFTER_G3_ON            1
#define AFTER_G3_OFF           2
#define AFTER_G3_LAST_STATE    3
/// Possible last state values
#define LAST_STATE_OFF  0x5
#define LAST_STATE_ON   0xA

// Status/Enable registers common bitmap
/**
 * @details Two considerations are in order here: bit assignment criteria and
 *          constant definition format. Both status and enable registers have
 *          kinds of bits: those shared, i.e. status bits that can be enabled
 *          to interrupt the host, and the remaining, i.e. status bit that can
 *          not interrupt the host and enable bit for events not related to any
 *          status bit. This is why we assign the former starting from the less
 *          significant and the latter from the most significant: in particular,
 *          the shared ones are defined based ob the selected bits, so as to
 *          maintain them aligned. Furthermore, these definitions must work
 *          also on ACPI ASL files, so they can not be defined with expressions
 *          containing C operators: this is why they are done with the direct
 *          constant definition, leaving indicated their origin in comments.
 */
#define BIT_LID_TX_HIGH               0
#define BIT_LID_TX_LOW                1
#define BIT_SLEEP_BUTTON              2
#define BIT_WAKE_BUTTON               3
#define BIT_SMB_ALERT                 4
#define BIT_BATLOW                    5
#define BIT_POWER_BUTTON              6

#define STATUS_BIT_LID_TX_HIGH        1    /// ( 1 << BIT_LID_TX_HIGH )
#define STATUS_BIT_LID_TX_LOW         2    /// ( 1 << BIT_LID_TX_LOW )
#define STATUS_BIT_SLEEP_BUTTON       4    /// ( 1 << BIT_SLEEP_BUTTON )
#define STATUS_BIT_WAKE_BUTTON        8    /// ( 1 << BIT_WAKE_BUTTON )
#define STATUS_BIT_SMB_ALERT          0x10 /// ( 1 << BIT_SMB_ALERT )
#define STATUS_BIT_BATLOW             0x20 /// ( 1 << BIT_BATLOW )
#define STATUS_BIT_POWER_BUTTON       0x40 /// ( 1 << BIT_POWER_BUTTON )

#define STATUS_BIT_LID_STATE          0x40000000 /// ( 1 << 30 )
#define STATUS_BIT_POWERFAIL          0x80000000 /// ( 1 << 31 )

#define ENABLE_BIT_LID_TX_HIGH        1    /// ( 1 << BIT_LID_TX_HIGH )
#define ENABLE_BIT_LID_TX_LOW         2    /// ( 1 << BIT_LID_TX_LOW )
#define ENABLE_BIT_SLEEP_BUTTON       4    /// ( 1 << BIT_SLEEP_BUTTON )
#define ENABLE_BIT_WAKE_BUTTON        8    /// ( 1 << BIT_WAKE_BUTTON )
#define ENABLE_BIT_SMB_ALERT          0x10 /// ( 1 << BIT_SMB_ALERT )
#define ENABLE_BIT_BATLOW             0x20 /// ( 1 << BIT_BATLOW )
#define ENABLE_BIT_POWER_BUTTON       0x40 /// ( 1 << BIT_POWER_BUTTON )

// ACPI Notify events management
// List of status bit that can assert the uC wake signal and must be cleared before deasserting it
#define MICRO_NOTIFY_ENABLED_MASK ( STATUS_BIT_LID_TX_HIGH \
                                  | STATUS_BIT_LID_TX_LOW \
                                  | STATUS_BIT_SLEEP_BUTTON \
                                  | STATUS_BIT_SMB_ALERT \
                                  | STATUS_BIT_BATLOW \
                                  | STATUS_BIT_POWER_BUTTON )

// Status bit not clearable from the outside
#define MICRO_READONLY_STATUS_MASK ( STATUS_BIT_LID_STATE )

/**
 * @brief reset causes and features constants, always in ACPI file compatible format
 */
#define BIT_RESET_CAUSE_WATCHDOG  0
#define BIT_RESET_CAUSE_POWERFAIL 1
#define BIT_RESET_CAUSE_SYSRESET  2
#define BIT_RESET_CAUSE_SOFTWARE  3
#define BIT_CHECK_RESET_CAUSE     4

#define BIT_UART0_PORT_80         5
#define BIT_UART1_PORT_80         6
#define BIT_CHECK_PORT_80         7

#define MEC170_RESET_CAUSE_WATCHDOG      1    /// ( 1 << BIT_RESET_CAUSE_WATCHDOG )
#define MEC170_RESET_CAUSE_POWERFAIL     2    /// ( 1 << BIT_RESET_CAUSE_POWERFAIL )
#define MEC170_RESET_CAUSE_SYSRESET      4    /// ( 1 << BIT_RESET_CAUSE_SYSRESET )
#define MEC170_RESET_CAUSE_SOFTWARE      8    /// ( 1 << BIT_RESET_CAUSE_SOFTWARE )
#define MEC170_RESET_CAUSE_MASK          0xf  /// OR of all the previous
#define MEC170_RESET_CAUSE_CHECK         0x10 /// ( 1 << BIT_RESET_CAUSE_SOFTWARE )
#define MEC170_TOTAL_RESET_CAUSE_MASK    0x1f /// OR of the previous two

#define ENABLE_UART0_PORT_80      0x20 /// ( 1 << BIT_UART0_PORT_80 )
#define ENABLE_UART1_PORT_80      0x40 /// ( 1 << BIT_UART1_PORT_80 )
#define SET_PORT_80_MASK          0x60 /// OR of all the previous
#define CHECK_PORT_80             0x80 /// ( 1 << BIT_CHECK_PORT_80 )
#define UART_PORT_80_MASK         0xe0 /// OR of the previous two

#pragma pack(pop)

/* Software definitions */
#define NO_PIN 0xFF


/* Portable 16 bit I/O data struct definition */
#pragma pack(push, 1)
typedef union{
    struct{
      unsigned char Index;
      unsigned char Data;
    };
    unsigned short Value;
}IO16BIT;
#pragma pack(pop)

/* Software definitions */
#define EC_CMD_TIMEOUT      0x30000 //  Maximum wait loop

/**
 * @brief GET_FIRMWARE_VERSION_CMD data struct and definitions
 */
#define FIRMWARE_TIME_STAMP_SIZE ( EC_MBX_SIZE - sizeof( uint32_t ) )

typedef struct{
  union{
    uint32_t versions;
    struct{
      union{
        uint16_t firmwareVersion;
        struct{
          uint8_t firmwareMinorVersion;
          uint8_t firmwareMajorVersion;
        };
      };
      union{
        uint16_t libraryVersion;
        struct{
          uint8_t libraryMinorVersion;
          uint8_t libraryMajorVersion;
        };
      };
    };
  }VERSION;
  uint8_t firmwareTimeStamp[FIRMWARE_TIME_STAMP_SIZE];
}GET_FIRMWARE_VERSION_STRUCT;

/**
 * @brief CEC data struct and constant definitions
 */
#define MECCEC_MAX_MSG_SIZE 16

struct seco_meccec_tx_t{
	u8 bus;
	u8 send;
	u8 dest;
	u8 data[MECCEC_MAX_MSG_SIZE];
	u8 size;	
};

struct seco_meccec_rx_t{
	u8 bus;
	u8 send;
	u8 dest;
	u8 data[MECCEC_MAX_MSG_SIZE];
	u8 size;	
};

struct seco_meccec_logaddr_t{
	u8 bus;
	u8 addr;	
};

struct seco_meccec_status_t{
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
typedef enum{
	MBX_OFF     = 0,          /*!< Disable MBX Interface */
	MBX_ON      = 1,          /*!< Enable MBX Interface  */
	MBX_ACTIVE0 = ( 1 << 6 ), /*!< MBX AGENT 0 active    */
	MBX_QUEUED0 = ( 1 << 7 ), /*!< MBX AGENT 0 idle      */
}MBX_STATUS;

#define AGENT_IDLE(x)      0
#define AGENT_QUEUED(x)    ( MBX_QUEUED0 >> ( 2 * x ) )
#define AGENT_ACTIVE(x)    ( MBX_ACTIVE0 >> ( 2 * x ) )
#define AGENT_MASK(x)      ( AGENT_QUEUED(x) + AGENT_ACTIVE(x) )
#define AGENT_DONE(x)      AGENT_MASK(x)
#define MBX_STATUS_DEFAULT 0

/**
 * @brief   MBX user IDs.
 * @note    Used to keep track of the current agent.
 * @details As explained in the definition of MBX_STATUS above, AGENT_NONE must
 *          be at most 4.
 */
typedef enum{
	AGENT_BIOS, /*!< BIOS AGENT */
	AGENT_ACPI, /*!< ACPI AGENT */
	AGENT_EAPI, /*!< EAPI AGENT */
	AGENT_USER, /*!< USER AGENT */
	AGENT_NONE, /*!< No AGENT   */
}AGENT_IDS;

/**
 * @brief MBX command return codes: 0 = success.
 */
typedef enum{
	EC_NO_ERROR = 0,           /*!< Success          */
	EC_UNKNOWN_COMMAND_ERROR,  /*!< Unknown command  */
	EC_INVALID_ARGUMENT_ERROR, /*!< Invalid argument */
	EC_TIMEOUT_ERROR,          /*!< Waiting Time-out */
	EC_DEVICE_ERROR,           /*!< Device error     */
}CMD_RESULT;

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
typedef enum{
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
}MBX_CMDS;

#define REQUEST_MBX_ACCESS(x) ( REQUEST_MBX_ACCESS_CMD + x )
#define RELEASE_MBX_ACCESS(x) ( RELEASE_MBX_ACCESS_CMD + x )

