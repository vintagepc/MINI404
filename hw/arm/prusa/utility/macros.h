/*
    macros.h - Stupid fix for the fact you can't use DEFINE
    with simple type objects and have to resort to copypasta
    or tons of boilerplate.

    Copyright 2021 VintagePC <https://github.com/vintagepc/>
    Based off the related macros in qom/object.h

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

#define OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(ModuleObjName, module_obj_name, \
                                    MODULE_OBJ_NAME, PARENT_MODULE_OBJ_NAME, \
                                    ...) \
    static void \
    module_obj_name##_finalize(Object *obj); \
    static void \
    module_obj_name##_class_init(ObjectClass *oc, void *data); \
    static void \
    module_obj_name##_init(Object *obj); \
    \
    static const TypeInfo module_obj_name##_info = { \
        .parent = TYPE_##PARENT_MODULE_OBJ_NAME, \
        .name = TYPE_##MODULE_OBJ_NAME, \
        .instance_size = sizeof(ModuleObjName), \
        .instance_align = __alignof__(ModuleObjName), \
        .instance_init = module_obj_name##_init, \
        .instance_finalize = module_obj_name##_finalize, \
        .class_init = module_obj_name##_class_init, \
        .abstract = false, \
        .interfaces = (InterfaceInfo[]) { __VA_ARGS__ } , \
    }; \
    \
    static void \
    module_obj_name##_register_types(void) \
    { \
        type_register_static(&module_obj_name##_info); \
    } \
    type_init(module_obj_name##_register_types);


#define OBJECT_DEFINE_SIMPLE_TYPE (ModuleObjName, module_obj_name, \
                        MODULE_OBJ_NAME, PARENT_MODULE_OBJ_NAME) \
                        OBJECT_DEFINE_TYPE_SIMPLE_WITH_INTERFACES(ModuleObjName, module_obj_name, \
                            MODULE_OBJ_NAME, PARENT_MODULE_OBJ_NAME, {NULL})

#define _JOIN2R(a,b) a##b
#define _JOIN3(a,b,c) a#b#c
#define _JOIN3R(a,b,c) a##b##c
#define _JOIN2(a,b) a#b

#define CHECK_PRI(x,y)  #x" != "#y
#define CHECK_ALIGN(x,y, name) QEMU_BUILD_BUG_MSG(x != y, "ERROR - " name " register definition misaligned! - " CHECK_PRI(x,y))
#define CHECK_REG_u32(reg) CHECK_ALIGN(sizeof(reg),sizeof(uint32_t),#reg " size incorrect!")
#define CHECK_TYPEDEF_u32(type,reg) CHECK_ALIGN(sizeof(((type*)0)->reg),sizeof(uint32_t),#reg " size incorrect!")
#define CHECK_REGDEF_u32(type,reg) CHECK_ALIGN(sizeof(type),sizeof(uint32_t),#reg " size incorrect!")

// 32-bit register with a single field of a lesser size.
#define REG_S32(name,used) struct{ uint32_t name :used; uint32_t :32-used; } QEMU_PACKED
// Single-bit field in a 32bit register
#define REG_B32(name) uint32_t name :1
// multi-bit bloc_K_ in a 32 bit register
#define REG_K32(name, len) uint32_t name : len

#define REGDEF_R(addr) uint32_t _JOIN2R(_reserved, addr)
// Block of reserved 32 bit registers:




#define REGDEF_NAME(part, x) _JOIN3R(stm32reg_,part##_##x,_t)

#define REGDEF_BLOCK_BEGIN() typedef union { \
	struct {

#define REGDEF_BLOCK_END(part, x) } QEMU_PACKED; \
	uint32_t raw; \
} REGDEF_NAME(part, x);

// Helper for auto-naming reserved blocks.
#define _REG_R32(name, count, size) uint32_t _JOIN2R(name,count) : size
// Define reserved bits in register.
#define REG_R(size) _REG_R32(_reserved, __LINE__, size)
// Single reserved bit.
#define REG_RB() _REG_R32(_reserved, __LINE__, 1)

// Missing int32 array macro:
#define VMSTATE_INT32_2DARRAY_V(_f, _s, _n1, _n2, _v)                \
    VMSTATE_2DARRAY(_f, _s, _n1, _n2, _v, vmstate_info_int32, int32_t)

#define VMSTATE_INT32_2DARRAY(_f, _s, _n1, _n2)                      \
    VMSTATE_INT32_2DARRAY_V(_f, _s, _n1, _n2, 0)

// Some rather ugly convenience macros for
// more easily debugging save state symmetry. See the RCC implementation for
// an example how this works. (STATE_DEBUG_VAR must be defined for it to work.)
#ifdef STATE_DEBUG_VAR
#define DEBUG_COPY(type, size) static type STATE_DEBUG_VAR[size]

#define DEBUG_INDEX(value) uint8_t index = value

#define DEBUG_TAKE(src,index) memcpy(&STATE_DEBUG_VAR[index],src, sizeof(STATE_DEBUG_VAR[index]))
#define DEBUG_CHECK(field) assert(s->field == STATE_DEBUG_VAR[index].field)
#define DEBUG_CHECKP(field) assert(s->field == STATE_DEBUG_VAR[index]->field)
#define DEBUG_VERIFY DEBUG_LIST
#define DEBUG_CAST_ONLY(cast) cast
#undef STATE_DEBUG_VAR
#else

#define DEBUG_COPY(type, size)
#define DEBUG_TAKE(src,index)

#define DEBUG_INDEX(value)
#define DEBUG_CAST_ONLY(cast)
#define DEBUG_CHECK(field)
#define DEBUG_VERIFY

#endif
