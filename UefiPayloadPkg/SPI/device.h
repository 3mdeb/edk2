#ifndef DEVICE_H
#define DEVICE_H

#include <Include/PiDxe.h>
#include "path.h"
#include "kconfig.h"

struct bus {

	struct device *dev;	/* This bridge device */
	struct device *children;	/* devices behind this bridge */
	struct bus *next;    /* The next bridge on this device */
	unsigned int	bridge_ctrl;	/* Bridge control register */
	UINT16	bridge_cmd;		/* Bridge command register */
	unsigned char	link_num;	/* The index of this link */
	UINT16	secondary;	/* secondary bus number */
	UINT16	subordinate;	/* max subordinate bus number */
	unsigned char   cap;		/* PCi capability offset */
	UINT32	hcdn_reg;		/* For HyperTransport link  */

	unsigned int	reset_needed : 1;
	unsigned int	disable_relaxed_ordering : 1;
	unsigned int	ht_link_up : 1;
	unsigned int	no_vga16 : 1;	/* No support for 16-bit VGA decoding */
};

/*
 * There is one device structure for each slot-number/function-number
 * combination:
 */

struct pci_irq_info {
	unsigned int	ioapic_irq_pin;
	unsigned int	ioapic_src_pin;
	unsigned int	ioapic_dst_id;
	unsigned int    ioapic_flags;
};

struct device {
	struct bus *bus;	/* bus this device is on, for bridge
					 * devices, it is the up stream bus */

	struct device *sibling;	/* next device on this bus */

	struct device *next;	/* chain of all devices */

	struct device_path path;
	unsigned int	vendor;
	unsigned int	device;
	UINT16	subsystem_vendor;
	UINT16	subsystem_device;
	unsigned int	class;		/* 3 bytes: (base, sub, prog-if) */
	unsigned int	hdr_type;	/* PCI header type */
	unsigned int  enabled : 1;	/* set if we should enable the device */
	unsigned int  initialized : 1; /* 1 if we have initialized the device */
	unsigned int    on_mainboard : 1;
	unsigned int    disable_pcie_aspm : 1;
	/* set if we should hide from UI */
	unsigned int    hidden : 1;
	/* set if this device is used even in minimum PCI cases */
	unsigned int    mandatory : 1;
	UINT8 command;
	UINT16 hotplug_buses; /* Number of hotplug buses to allocate */

	/* Base registers for this device. I/O, MEM and Expansion ROM */
	struct resource *resource_list;

	/* links are (downstream) buses attached to the device, usually a leaf
	 * device with no children has 0 buses attached and a bridge has 1 bus
	 */
	struct bus *link_list;

#if !DEVTREE_EARLY
	struct pci_irq_info pci_irq_info[4];
	struct device_operations *ops;
	struct chip_operations *chip_ops;
	const char *name;
#if CONFIG(GENERATE_SMBIOS_TABLES)
	UINT8 smbios_slot_type;
	UINT8 smbios_slot_data_width;
	UINT8 smbios_slot_length;
	const char *smbios_slot_designation;
#endif
#endif
	void *chip_info;

	/* Zero-terminated array of fields and options to probe. */
	struct fw_config *probe_list;
};

#endif /* DEVICE_H */
