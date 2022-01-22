/*
    gl_dashboard.c - C/C++ GL and QEMU interface wrapper for
    Mini404.

	Copyright 2021 VintagePC <https://github.com/vintagepc/>

 	This file is part of Mini404.

	Mini404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Mini404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Mini404.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "../utility/macros.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qom/object.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "dashboard_types.h"
#include "../opengl/GLDashboardMgr.h"

#define TYPE_GLDASHBOARD "gl-dashboard"

OBJECT_DECLARE_SIMPLE_TYPE(GLDashboadState, GLDASHBOARD)

struct GLDashboadState {
    SysBusDevice parent_obj;
    /*< private >*/
    /*< public >*/
    void *dashboardmgr;

    uint8_t dashboard_type;

};

OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(GLDashboadState, gldashboard, GLDASHBOARD, SYS_BUS_DEVICE, {NULL})


static void gldashboard_realize(DeviceState *dev, Error **errp)
{
#ifdef BUDDY_HAS_GL
    GLDashboadState *s = GLDASHBOARD(dev);
    if (s->dashboard_type) {
        s->dashboardmgr = gl_dashboard_start(s->dashboard_type);
        if (s->dashboardmgr) {
            gl_dashboard_run(s->dashboardmgr);
        }
    }
#endif
}

static void gldashboard_motor_in(void *opaque, int n, int level) {
#ifdef BUDDY_HAS_GL
    GLDashboadState *s = GLDASHBOARD(opaque);
    if (s->dashboard_type) {
        gl_dashboard_update_motor(s->dashboardmgr,n,level);
    }
#endif
}

static void gldashboard_indicator_in(void *opaque, int n, int level) {
#ifdef BUDDY_HAS_GL    
    GLDashboadState *s = GLDASHBOARD(opaque);
    if (s->dashboard_type) {
        gl_dashboard_update_indicator(s->dashboardmgr,n,level);
    }
#endif
}

static void gldashboard_indicator_logic_in(void *opaque, int n, int level) {
#ifdef BUDDY_HAS_GL
    GLDashboadState *s = GLDASHBOARD(opaque);
    if (s->dashboard_type) {
        gl_dashboard_update_indicator(s->dashboardmgr,n,level ? 255 : 0);
    }
#endif
}

static void gldashboard_finalize(Object *obj)
{
}

static void gldashboard_reset(DeviceState *dev) {
#ifdef BUDDY_HAS_GL
    GLDashboadState *s = GLDASHBOARD(dev);
    if (s->dashboard_type) {
        gl_dashboard_reset(s->dashboardmgr);
    }
#endif
}

static void gldashboard_init(Object *obj)
{
    qdev_init_gpio_in_named(DEVICE(obj), gldashboard_motor_in, "motor-step",DB_MOTOR_COUNT);
    qdev_init_gpio_in_named(DEVICE(obj), gldashboard_indicator_in, "indicator-analog",DB_IND_COUNT);
    qdev_init_gpio_in_named(DEVICE(obj), gldashboard_indicator_logic_in, "indicator-logic",DB_IND_COUNT);

}

static Property gldashboard_properties[] = {
    DEFINE_PROP_UINT8("dashboard_type",GLDashboadState, dashboard_type, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void gldashboard_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->reset = gldashboard_reset;
    dc->realize = gldashboard_realize;
    device_class_set_props(dc, gldashboard_properties);
}
