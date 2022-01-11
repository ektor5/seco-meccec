// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * CEC driver for SECO MEC-based Boards
 *
 * Author:  Ettore Chimenti <ek5.chimenti@gmail.com>
 * Copyright (C) 2022, SECO SpA.
 */

#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/dmi.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/version.h>

/* CEC Framework */
#include <media/cec-notifier.h>

#include "seco-meccec.h"

#define DEBUG
#define SECO_MECCEC_DEV_NAME "seco_meccec"
#define MECCEC_MAX_CEC_ADAP 4
#define MECCEC_MAX_ADDRS 1

static DEFINE_MUTEX(ec_mutex);

struct seco_meccec_data {
	struct device *dev;
	struct platform_device *pdev;
	struct cec_adapter *cec_adap[MECCEC_MAX_CEC_ADAP];
	struct cec_notifier *notifier[MECCEC_MAX_CEC_ADAP];
	u8 channels;	/* bitmask */
	int irq;
};

/*
 *
 *static uint8_t hwio_config_op(
 *                uint8_t reg,
 *                uint8_t operation,
 *                uint8_t val,
 *                uint8_t* result)
 *{
 *        /// Unlock the configuration
 *        outb(EC_CONFIG_UNLOCK, EC_CONFIG_INDEX);
 *        /// Set the register index
 *        outb(reg, EC_CONFIG_INDEX);
 *
 *        if (operation == READ) {
 *                /// Read the register value
 *                *result = inb(EC_CONFIG_DATA);
 *
 *        }
 *        if (operation == WRITE) {
 *                /// Write the register value
 *                outb(val, EC_CONFIG_DATA);
 *        }
 *
 *        /// Lock the configuration
 *        outb(EC_CONFIG_LOCK, EC_CONFIG_INDEX);
 *
 *        return 0;
 *}
 *
 *#define hwio_config_rd(reg, res) hwio_config_op(reg, READ , 0, result)
 *#define hwio_config_wr(reg, val) hwio_config_op(reg, WRITE, val, NULL)
 *
 */

static int ec_reg_byte_op(u8 reg, u8 operation, u8 data, u8 *result)
{
	// Check still active
	if (!(inb(MBX_RESOURCE_REGISTER) & AGENT_ACTIVE(AGENT_USER)))
		return -EBUSY;

	// Set the register index
	outb(reg, EC_REGISTER_INDEX);

	// Check still active
	if (!(inb(MBX_RESOURCE_REGISTER) & AGENT_ACTIVE(AGENT_USER)))
		return -EBUSY;

	if (operation == READ) {
		if (!result)
			return -EINVAL;

		// Read the data value
		*result = inb(EC_REGISTER_DATA);
	} else if (operation == WRITE) {
		// Write the data value
		outb(data, EC_REGISTER_DATA);
	}

	// Check still active
	if (!(inb(MBX_RESOURCE_REGISTER) & AGENT_ACTIVE(AGENT_USER)))
		return -EBUSY;

	return 0;
}

#define ec_reg_byte_rd(reg, res) ec_reg_byte_op(reg, READ, 0, res)
#define ec_reg_byte_wr(reg, val) ec_reg_byte_op(reg, WRITE, val, NULL)

/// @brief  E.C. status wait
/// @note   No check is done on the argument; see OemEcSendCommand for details
/// @param  ui8Status   status to wait for
/// @param  cmd  command to trigger status change if needed (0 if none)
/// @return EFI_TIMEOUT if not status in EC_CMD_TIMEOUT attempts
///         EAPI_STATUS_SUCCESS if successful
static int ec_waitstatus(u8 status, u8 cmd)
{
	int idx;

	// Loop until time-out or Status
	for (idx = 0; idx < EC_CMD_TIMEOUT; idx++) {
		// If status, done
		if ((inb(MBX_RESOURCE_REGISTER) & AGENT_MASK(AGENT_USER)) == status)
			return 0;

		/// Send command if needed
		if (cmd)
			outb_p(cmd, MBX_RESOURCE_REGISTER);
	}

	/// Time-out expired
	return -EAGAIN;
}

/// @brief   Send command to E.C.
/// @note    No check is done on the arguments validity
/// @details To guarantee atomic command execution, any agent will acquire an
///          hardware semaphore; these semaphores are implemented via the data
///          I/O of an ACPI-ECI interface, which will give on reads the
///          combined status of the agents and will trigger on writes the
///          actions of acquiring/releasing resources: due to the hardware
///          mechanism, the commands will be repeatedly written until the
///          desired status comes or a time-out expires. The true commands will
///          be sent instead on the Mail Box interface, that can not be
///          accessed atomically because of the Index/Data access method, so
///          are sequentially used by only one agent at a time. These is why
///          the algorithm is the following:
///          1) wait for the idle state, continuously sending
///             RELEASE_MBX_ACCESS_CMD: if the E.C. is working, this is fast;
///          2) wait for queued state, continuously sending REQUEST_MBX_ACCESS;
///          3) wait for active state, with a sufficiently long time-out;
///          4) fill MBX with parameters, send command and wait for done state;
///          5) read result and if success read data if any;
///          6) in any case, repeat pass 1 to release resources.
///          This last pass must be done even in case of failure of any of the
///          preceding, to make the E.C. return to the idle state as soon as
///          possible.
///          The E.C. is responsible for maintaining only one active agent at a
///          time, with a round robin algorithm to avoid infinite waiting, and
///          to switch to the next agent when a request is pending, either at
///          the completion of a command or when in IDLE phase after reception
///          of REQUEST_MBX_ACCESS command.
///          When an agent is active, i.e. it is sure to have exclusive access
///          to all the MailBox registers, it can fill the MailBox with the
///          command parameters, send the command, wait for completion, save
///          the completion code and the command output data before releasing
///          the exclusive access.
///          The only registers shared by all the agents at any time are those
///          related to the ACPI-ECI interface.
/// @param   cmd       Command to be sent, of type MBX_CMDS
/// @param   pui8InputBuffer  E.C. input data buffer, NULL if not required
/// @param   ui8InputLength   E.C. input buffer length, limited by EC_MBX_SIZE
/// @param   pui8OutputBuffer E.C. output data buffer, NULL if not required
/// @param   tx_size  E.C. output buffer length, limited by EC_MBX_SIZE
/// @return  EAPI_STATUS_TIMEOUT       if E.C. not present or malfunctioning
///          EAPI_STATUS_TIMEOUT           if any phase lasts too much
///          EFI_UNSUPPORTED       if EC_UNKNOWN_COMMAND_ERROR
///          EFI_INVALID_PARAMETER if EC_INVALID_ARGUMENT_ERROR
///          EFI_DEVICE_ERROR      if unknown error
///          EAPI_STATUS_SUCCESS           if successful
static int ec_send_command(const struct platform_device *pdev, u8 cmd,
			   void *rx_buf, u8 rx_size,
			   void *tx_buf, u8 tx_size)
{
	struct seco_meccec_data *meccec = platform_get_drvdata(pdev);
	const struct device *dev = meccec->dev;

	int status;
	u8 *buf;
	u8 idx;
	u8 res;

	mutex_lock(&ec_mutex);

	// Wait for BIOS agent idle (should be already if all works)
	status = ec_waitstatus(AGENT_IDLE(AGENT_USER), 0);
	if (status) {
		dev_err(dev, "Mailbox agent not available");
		goto err;
	}

	// BIOS agent is idle: we can request access
	status = ec_waitstatus(AGENT_ACTIVE(AGENT_USER),
			       REQUEST_MBX_ACCESS(AGENT_USER));
	if (status) {
		dev_err(dev, "Request mailbox agent failed");
		goto err;
	}

	// We now prepare MBX data if we can
	for (buf = (uint8_t *)rx_buf, idx = 0; (!status) && idx < rx_size; idx++)
		status = ec_reg_byte_wr(EC_MBX_REGISTER + idx, buf[idx]);

	if (status) {
		dev_err(dev, "Mailbox buffer write failed");
		goto err;
	}

	// Send command
	status = ec_reg_byte_wr(EC_COMMAND_REGISTER, cmd);
	if (status) {
		dev_err(dev, "Command write failed");
		goto err;
	}

	// Wait for completion
	status = ec_waitstatus(AGENT_DONE(AGENT_USER), 0);
	if (status) {
		dev_err(dev, "Mailbox did not complete after command write");
		goto err;
	}

	// Get result code
	status = ec_reg_byte_rd(EC_RESULT_REGISTER, &res);
	if (status) {
		dev_err(dev, "Result read failed");
		goto err;
	}

	// Get result code and translate it
	switch (res) {
	case EC_NO_ERROR:
		status = 0;
		break;

	case EC_UNKNOWN_COMMAND_ERROR:
		status = -EPERM;
		break;

	case EC_INVALID_ARGUMENT_ERROR:
		status = -EINVAL;
		break;

	case EC_TIMEOUT_ERROR:
		status = -EAGAIN;
		break;

	default:
		status = -EIO;
		break;
	}
	if (status) {
		dev_err(dev, "Command failed");
		goto err;
	}

	// We now have to read return data if we can
	for (buf = (uint8_t *)tx_buf, idx = 0; !status && idx < tx_size; idx++)
		status = ec_reg_byte_rd(EC_MBX_REGISTER + idx, &buf[idx]);

	if (status) {
		dev_err(dev, "Mailbox read failed");
		goto err;
	}

	// Release access, ignoring eventual time-out
	ec_waitstatus(AGENT_IDLE(AGENT_USER), RELEASE_MBX_ACCESS(AGENT_USER));

err:
	mutex_unlock(&ec_mutex);
	return status;
}

static int find_adap_idx(struct cec_adapter *adap)
{
	struct seco_meccec_data *cec = cec_get_drvdata(adap);
	int idx;

	if (!adap)
		return -EINVAL;

	for (idx = 0; idx < MECCEC_MAX_CEC_ADAP; idx++) {
		if (cec->cec_adap[idx] == adap)
			return idx;
	}
	return -ENODEV;
}

static int ec_get_version(struct seco_meccec_data *cec)
{
	const struct device *dev = cec->dev;
	const struct platform_device *pdev = cec->pdev;
	struct version_msg_t version;
	int status;

	status = ec_send_command(pdev, GET_FIRMWARE_VERSION_CMD,
				 NULL, 0,
				 &version, sizeof(struct version_msg_t));

	if (status)
		return status;

	dev_dbg(dev, "Firmware version %X.%02X / %X.%02X",
		version.fw.major,
		version.fw.minor,
		version.lib.major,
		version.lib.minor);

	return 0;
}

static int ec_cec_status(struct seco_meccec_data *cec,
			 struct seco_meccec_status_t *result)
{
	const struct device *dev = cec->dev;
	const struct platform_device *pdev = cec->pdev;
	struct seco_meccec_status_t buf = { 0 };
	int ret;

	ret = ec_send_command(pdev, GET_CEC_STATUS_CMD,
			      //NULL, 0,
			      &buf, sizeof(struct seco_meccec_status_t),
			      &buf, sizeof(struct seco_meccec_status_t));
	if (ret)
		return ret;

	dev_dbg(dev, "CEC Status:");
	dev_dbg(dev, "ch0: 0x%02x", buf.status_ch0);
	dev_dbg(dev, "ch1: 0x%02x", buf.status_ch1);
	dev_dbg(dev, "ch2: 0x%02x", buf.status_ch2);
	dev_dbg(dev, "ch3: 0x%02x", buf.status_ch3);

	if (result)
		*result = buf;

	return 0;
}

static int meccec_adap_log_addr(struct cec_adapter *adap, u8 logical_addr)
{
	struct seco_meccec_data *cec = cec_get_drvdata(adap);
	struct platform_device *pdev = cec->pdev;
	const struct device *dev = cec->dev;
	struct seco_meccec_logaddr_t buf = { };
	int status;

	buf.bus = find_adap_idx(adap);
	buf.addr = logical_addr & 0x0f;

	status = ec_send_command(pdev, SET_CEC_LOGADDR_CMD,
				 &buf, sizeof(struct seco_meccec_logaddr_t),
				 NULL, 0);

	dev_dbg(dev, "Log address 0x%02x", logical_addr);

	return status;
}

static int meccec_adap_phys_addr(struct cec_adapter *adap)
{
	struct seco_meccec_data *cec = cec_get_drvdata(adap);
	struct platform_device *pdev = cec->pdev;
	struct seco_meccec_phyaddr_t buf = { };
	int status;

	buf.bus = find_adap_idx(adap);
	buf.addr = cpu_to_be32(adap->phys_addr);

	status = ec_send_command(pdev, SET_CEC_PHYADDR_CMD,
				 &buf, sizeof(struct seco_meccec_phyaddr_t),
				 NULL, 0);

	return status;
}

static int meccec_adap_enable(struct cec_adapter *adap, bool enable)
{
	struct seco_meccec_data *cec = cec_get_drvdata(adap);
	const struct device *dev = cec->dev;
	int ret;

	/* reset status register */
	ret = ec_cec_status(cec, NULL);
	if (ret)
		dev_err(dev, "enable: status operation failed (%d)", ret);

	if (enable)
		dev_dbg(dev, "Device enabled");
	else
		dev_dbg(dev, "Device disabled");

	ret = meccec_adap_phys_addr(adap);
	if (ret) {
		dev_err(dev, "enable: set physical address failed (%d)", ret);
		return ret;
	}

	return 0;
}

static int meccec_adap_transmit(struct cec_adapter *adap, u8 attempts,
				u32 signal_free_time, struct cec_msg *msg)
{
	struct seco_meccec_data *cec = cec_get_drvdata(adap);
	struct platform_device *pdev = cec->pdev;
	const struct device *dev = cec->dev;
	struct seco_meccec_tx_t buf = { };
	int status, idx, i;

	dev_dbg(dev, "Device transmitting");

	idx = find_adap_idx(adap);
	if (idx < 0)
		return idx;

	buf.bus = idx;
	buf.send = (msg->msg[0] & 0xf0) >> 4;
	buf.dest = msg->msg[0] & 0x0f;
	buf.size = msg->len - 1;
	memcpy(buf.data, msg->msg + 1, buf.size);

	/* TODO: debug, remove */
	dev_dbg(dev, "tx_buf:");
	dev_dbg(dev, "send: 0x%0x", buf.send);
	dev_dbg(dev, "dest: 0x%0x", buf.dest);
	for (i = 0; i < buf.size; i++) {
		dev_dbg(dev, "%02d: %02x", i, buf.data[i]);
	}
	dev_dbg(dev, "size: %d", buf.size);

	dev_dbg(dev, "tx_msg:");
	dev_dbg(dev, "size: %d", msg->len);
	for (i = 0; i < msg->len; i++) {
		dev_dbg(dev, "%02d: %02x", i, msg->msg[i]);
	}

	status = ec_send_command(pdev, CEC_WRITE_CMD,
				 &buf, sizeof(struct seco_meccec_tx_t),
				 NULL, 0);

	return status;
}

static void meccec_tx_done(struct seco_meccec_data *cec, int adap_idx, u8 status_val)
{
	struct cec_adapter *adap = cec->cec_adap[adap_idx];

	if (status_val & SECOCEC_STATUS_TX_ERROR_MASK) {
		if (status_val & SECOCEC_STATUS_TX_NACK_ERROR)
			cec_transmit_attempt_done(adap, CEC_TX_STATUS_NACK);
		else
			cec_transmit_attempt_done(adap, CEC_TX_STATUS_ERROR);
	} else {
		cec_transmit_attempt_done(adap, CEC_TX_STATUS_OK);
	}
}

static void meccec_rx_done(struct seco_meccec_data *cec, int adap_idx, u8 status_val)
{
	struct device *dev = cec->dev;
	struct platform_device *pdev = cec->pdev;
	struct cec_adapter *adap = cec->cec_adap[adap_idx];
	struct seco_meccec_rx_t buf = { .bus = adap_idx };
	struct cec_msg msg = { };
	int status;
	int i = 0;

	if (status_val & SECOCEC_STATUS_RX_OVERFLOW_MASK) {
		/* NOTE: Untested, it also might not be necessary */
		dev_warn(dev, "Received more than 16 bytes. Discarding");
	}

	if (status_val & SECOCEC_STATUS_RX_ERROR_MASK) {
		dev_warn(dev, "Message received with errors. Discarding");
		status = -EIO;
		goto rxerr;
	}
	/* Read message buffer */
	status = ec_send_command(pdev, CEC_READ_CMD,
				 &buf, sizeof(struct seco_meccec_rx_t),
				 &buf, sizeof(struct seco_meccec_rx_t));
	if (status)
		return;

	/* Device msg len already accounts for the header */
	msg.len = min(buf.size + 1, CEC_MAX_MSG_SIZE);

	/* Read logical address */
	msg.msg[0]  = buf.dest & 0x0f;
	msg.msg[0] |= (buf.send & 0x0f) << 4;

	memcpy(msg.msg + 1, buf.data, buf.size);

	/* TODO: debug, remove */
	dev_dbg(dev, "rx_buf:");
	dev_dbg(dev, "bus: %d", buf.bus);
	dev_dbg(dev, "send: 0x%0x", buf.send);
	dev_dbg(dev, "dest: 0x%0x", buf.dest);
	for (i = 0; i < buf.bus; i++) {
		dev_dbg(dev, "%02d: %02x", i, buf.data[i]);
	}
	dev_dbg(dev, "size: %d", buf.size);

	dev_dbg(dev, "rx_msg:");
	dev_dbg(dev, "size: %d", msg.len);
	for (i = 0; i < msg.len; i++) {
		dev_dbg(dev, "%02d: %02x", i, msg.msg[i]);
	}

	cec_received_msg(adap, &msg);
	dev_dbg(dev, "Message received successfully");

rxerr:
	return;
}

static int get_status_ch(struct seco_meccec_status_t *s,
			 int ch)
{
	if (!s)
		return -1;

	switch (ch) {
	case 0: return s->status_ch0;
	case 1: return s->status_ch1;
	case 2: return s->status_ch2;
	case 3: return s->status_ch3;
	default: return -1;
	}
}

static irqreturn_t seco_meccec_irq_handler(int irq, void *priv)
{
	struct seco_meccec_data *cec = priv;
	struct device *dev = cec->dev;
	/*u16 status_val, cec_val, val = 0;*/
	int ret, idx;
	struct seco_meccec_status_t status;

	dev_dbg(dev, "Interrupt Called!");

	ret = ec_cec_status(cec, &status);
	if (ret)
		dev_err_once(dev, "IRQ: status operation failed (%d)", ret);

	for (idx = 0; idx < MECCEC_MAX_CEC_ADAP; idx++) {
		if (cec->channels & BIT_MASK(idx)) {
			int cec_val = get_status_ch(&status, idx);

			if (cec_val < 0)
				continue;

			if (cec_val & SECOCEC_STATUS_MSG_RECEIVED_MASK)
				meccec_rx_done(cec, idx, cec_val);
			if (cec_val & SECOCEC_STATUS_MSG_SENT_MASK)
				meccec_tx_done(cec, idx, cec_val);

			/* TODO: improve this or remove, it should take care of
			 * other channels */

			if ((~cec_val & SECOCEC_STATUS_MSG_SENT_MASK) &&
					(~cec_val & SECOCEC_STATUS_MSG_RECEIVED_MASK))
				dev_warn_once(dev,
					      "Message not received or sent, but interrupt fired");
		}
	}

	return IRQ_HANDLED;
}

struct cec_dmi_match {
	const char *sys_vendor;
	const char *product_name;
	const char *devname;
	const char *conn[MECCEC_MAX_CEC_ADAP];
};

static const struct cec_dmi_match secocec_dmi_match_table[] = {
	/* UDOO BOLT */
	{ "SECO", "UDOO BOLT", "0000:00:02.0", {"Port B"} },
	/* SECO SBC-D61 */
	{ "Seco", "0D61", "0000:00:02.0", {"Port B", "Port C", "Port D", "Port E"} },
};

static struct device *seco_meccec_find_hdmi_dev(struct device *dev,
						const char * const **conn_ptr)
{
	int i;

	for (i = 0 ; i < ARRAY_SIZE(secocec_dmi_match_table) ; ++i) {
		const struct cec_dmi_match *m = &secocec_dmi_match_table[i];

		if (dmi_match(DMI_SYS_VENDOR, m->sys_vendor) &&
		    dmi_match(DMI_PRODUCT_NAME, m->product_name)) {
			struct device *d;

			/* Find the device, bail out if not yet registered */
			d = bus_find_device_by_name(&pci_bus_type, NULL,
						    m->devname);
			if (!d)
				return ERR_PTR(-EPROBE_DEFER);

			put_device(d);

			if (!conn_ptr)
				return ERR_PTR(-EFAULT);

			*conn_ptr = m->conn;

			return d;
		}
	}

	return ERR_PTR(-EINVAL);
}

static int seco_meccec_acpi_probe(struct seco_meccec_data *sdev)
{
	struct device *dev = sdev->dev;
	const struct acpi_device *adev = ACPI_COMPANION(dev);
	const union acpi_object *obj;
	struct gpio_desc *gpio;
	int irq = 0;
	int ret;

	gpio = devm_gpiod_get(dev, "notify", GPIOF_IN);
	if (IS_ERR(gpio)) {
		dev_err(dev, "Cannot request interrupt gpio");
		return PTR_ERR(gpio);
	}

	irq = gpiod_to_irq(gpio);
	if (irq < 0) {
		dev_err(dev, "Cannot find valid irq");
		return -ENODEV;
	}
	dev_dbg(dev, "irq-gpio is bound to IRQ %d", irq);
	sdev->irq = irq;

	/* get info from ACPI about channels capabilities */
	ret = acpi_dev_get_property(adev, "av-channels",  ACPI_TYPE_INTEGER, &obj);
	if (ret < 0) {
		dev_err(dev, "Cannot retrieve channel properties");
		return ret;
	}
	dev_dbg(dev, "ACPI property: av-channels -> %x", (int)obj->integer.value);
	sdev->channels = (int)obj->integer.value;

	return 0;
}

static const struct cec_adap_ops meccec_cec_adap_ops = {
	/* Low-level callbacks */
	.adap_enable = meccec_adap_enable,
	.adap_log_addr = meccec_adap_log_addr,
	.adap_transmit = meccec_adap_transmit,
};

static int seco_meccec_probe(struct platform_device *pdev)
{
	struct seco_meccec_data *meccec;
	struct device *dev = &pdev->dev;
	struct device *hdmi_dev;
	const char * const *conn;
	int ret, idx;
	int adaps, notifs = 0;

	meccec = devm_kzalloc(dev, sizeof(*meccec), GFP_KERNEL);
	if (!meccec)
		return -ENOMEM;

	dev_set_drvdata(dev, meccec);

	meccec->pdev = pdev;
	meccec->dev = dev;

	ret = ec_get_version(meccec);
	if (ret) {
		dev_err(dev, "Get version failed");
		goto err;
	}

	if (!has_acpi_companion(dev)) {
		dev_err(dev, "Cannot find any ACPI companion");
		ret = -ENODEV;
		goto err;
	}

	ret = seco_meccec_acpi_probe(meccec);
	if (ret) {
		dev_err(dev, "ACPI probe failed");
		goto err;
	}

	ret = devm_request_threaded_irq(dev,
					meccec->irq,
					NULL,
					seco_meccec_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					dev_name(&pdev->dev), meccec);

	if (ret) {
		dev_err(dev, "Cannot request IRQ %d", meccec->irq);
		ret = -EIO;
		goto err;
	}

	/* for (each enabled cec)
	 *	do probe
	 */

	hdmi_dev = seco_meccec_find_hdmi_dev(&pdev->dev, &conn);
	if (IS_ERR(hdmi_dev)) {
		dev_err(dev, "Cannot find HDMI Device");
		return PTR_ERR(hdmi_dev);
	}
	dev_dbg(dev, "HDMI device found");

	for (idx = 0; idx < MECCEC_MAX_CEC_ADAP; idx++) {
		if (meccec->channels & BIT_MASK(idx)) {
			struct cec_adapter *acec;

			/* Allocate CEC adapter */
			acec = cec_allocate_adapter(&meccec_cec_adap_ops,
						    meccec,
						    dev_name(dev),
						    CEC_CAP_DEFAULTS |
						    CEC_CAP_CONNECTOR_INFO,
						    MECCEC_MAX_ADDRS);

			if (IS_ERR(acec)) {
				ret = PTR_ERR(acec);
				goto err_delete_adapter;
			}
			dev_dbg(dev, "CEC adapter #%d allocated", idx);

			meccec->cec_adap[idx] = acec;
			adaps++;
		}
	}

	for (idx = 0; idx < MECCEC_MAX_CEC_ADAP; idx++) {
		if (meccec->channels & BIT_MASK(idx)) {
			struct cec_adapter *acec = meccec->cec_adap[idx];
			struct cec_notifier *ncec;

			if (!acec) {
				ret = -EINVAL;
				goto err_notifier;
			}

			ncec = cec_notifier_cec_adap_register(hdmi_dev,
							      conn[idx], acec);

			dev_dbg(dev, "CEC notifier #%d allocated %s", idx, conn[idx]);

			if (IS_ERR(ncec)) {
				ret = PTR_ERR(ncec);
				goto err_notifier;
			}

			meccec->notifier[idx] = ncec;
			notifs++;
		}
	}

	for (idx = 0; idx < MECCEC_MAX_CEC_ADAP; idx++) {
		if (meccec->channels & BIT_MASK(idx)) {
			ret = cec_register_adapter(meccec->cec_adap[idx], dev);
			if (ret)
				goto err_notifier;

			dev_dbg(dev, "CEC adapter #%d registered", idx);
		}
	}

	platform_set_drvdata(pdev, meccec);
	dev_dbg(dev, "Device registered");

	return ret;

err_notifier:
	for (idx = 0; idx < MECCEC_MAX_CEC_ADAP; idx++) {
		if (meccec->channels & BIT_MASK(idx)) {
			if (adaps--)
				return ret;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0))
			cec_notifier_cec_adap_unregister(meccec->notifier[idx],
							 meccec->cec_adap[idx]);
#else
			cec_notifier_cec_adap_unregister(meccec->notifier[idx]);
#endif
		}
	}
err_delete_adapter:
	for (idx = 0; idx < MECCEC_MAX_CEC_ADAP; idx++) {
		if (meccec->channels & BIT_MASK(idx)) {
			if (notifs--)
				return ret;

			cec_delete_adapter(meccec->cec_adap[idx]);
		}
	}
err:
	dev_err(dev, "%s device probe failed: %d", dev_name(dev), ret);

	return ret;
}

static int seco_meccec_remove(struct platform_device *pdev)
{
	struct seco_meccec_data *meccec = platform_get_drvdata(pdev);
	int idx;

	for (idx = 0; idx < MECCEC_MAX_CEC_ADAP; idx++) {
		if (meccec->channels && BIT_MASK(idx)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0))
			cec_notifier_cec_adap_unregister(meccec->notifier[idx],
							 meccec->cec_adap[idx]);
#else
			cec_notifier_cec_adap_unregister(meccec->notifier[idx]);
#endif

			cec_unregister_adapter(meccec->cec_adap[idx]);
		}
	}

	dev_dbg(&pdev->dev, "CEC device removed");

	return 0;
}

#define SECO_MECCEC_PM_OPS NULL

#ifdef CONFIG_ACPI
static const struct acpi_device_id seco_meccec_acpi_match[] = {
	{"CEC00002", 0},
	{},
};

MODULE_DEVICE_TABLE(acpi, seco_meccec_acpi_match);
#endif

static struct platform_driver seco_meccec_driver = {
	.driver = {
		.name = SECO_MECCEC_DEV_NAME,
		.acpi_match_table = ACPI_PTR(seco_meccec_acpi_match),
		.pm = SECO_MECCEC_PM_OPS,
	},
	.probe = seco_meccec_probe,
	.remove = seco_meccec_remove,
};

module_platform_driver(seco_meccec_driver);

MODULE_DESCRIPTION("SECO MEC CEC Driver");
MODULE_AUTHOR("Ettore Chimenti <ek5.chimenti@gmail.com>");
MODULE_LICENSE("Dual BSD/GPL");
