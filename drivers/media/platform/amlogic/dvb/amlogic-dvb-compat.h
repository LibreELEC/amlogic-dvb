/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __AML_DVB_COMPAT_H
#define __AML_DVB_COMPAT_H

#include <media/dvbdev.h>

static inline int aml_dvb_register_adapter(struct dvb_adapter *adap,
					   const char *name,
					   struct module *module,
					   struct device *dev,
					   short *adapter_nr)
{
	return dvb_register_adapter(adap, name, module, dev, adapter_nr);
}

#endif