#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <dlfcn.h>
#include <stdbool.h>
#include <stdint.h>

#include "opendbc/safety/can.h"

typedef struct {
  PyObject_HEAD
  CANPacket_t packet;
} CANPacketObject;

typedef struct {
  PyObject_HEAD
} LibSafetyObject;

static PyTypeObject CANPacketType;
static PyTypeObject LibSafetyType;
static LibSafetyObject *g_libsafety = NULL;
static void *g_handle = NULL;

typedef bool (*bool_packet_fn)(CANPacket_t *msg);
typedef int (*int_int_int_fn)(int, int);
typedef int (*int_u16_u16_fn)(uint16_t, uint16_t);
typedef void (*void_bool_fn)(bool);
typedef bool (*bool_void_fn)(void);
typedef void (*void_int_fn)(int);
typedef int (*int_void_fn)(void);
typedef void (*void_int_int_fn)(int, int);
typedef void (*void_u32_fn)(uint32_t);
typedef float (*float_void_fn)(void);

static bool_packet_fn fn_safety_rx_hook = NULL;
static bool_packet_fn fn_safety_tx_hook = NULL;
static int_int_int_fn fn_safety_fwd_hook = NULL;
static int_u16_u16_fn fn_set_safety_hooks = NULL;
static void_bool_fn fn_set_controls_allowed = NULL;
static bool_void_fn fn_get_controls_allowed = NULL;
static bool_void_fn fn_get_longitudinal_allowed = NULL;
static void_int_fn fn_set_alternative_experience = NULL;
static int_void_fn fn_get_alternative_experience = NULL;
static void_bool_fn fn_set_relay_malfunction = NULL;
static bool_void_fn fn_get_relay_malfunction = NULL;
static bool_void_fn fn_get_gas_pressed_prev = NULL;
static void_bool_fn fn_set_gas_pressed_prev = NULL;
static bool_void_fn fn_get_brake_pressed_prev = NULL;
static bool_void_fn fn_get_regen_braking_prev = NULL;
static bool_void_fn fn_get_steering_disengage_prev = NULL;
static bool_void_fn fn_get_acc_main_on = NULL;
static float_void_fn fn_get_vehicle_speed_min = NULL;
static float_void_fn fn_get_vehicle_speed_max = NULL;
static int_void_fn fn_get_current_safety_mode = NULL;
static int_void_fn fn_get_current_safety_param = NULL;
static void_int_int_fn fn_set_torque_meas = NULL;
static int_void_fn fn_get_torque_meas_min = NULL;
static int_void_fn fn_get_torque_meas_max = NULL;
static void_int_int_fn fn_set_torque_driver = NULL;
static int_void_fn fn_get_torque_driver_min = NULL;
static int_void_fn fn_get_torque_driver_max = NULL;
static void_int_fn fn_set_desired_torque_last = NULL;
static void_int_fn fn_set_rt_torque_last = NULL;
static void_int_fn fn_set_desired_angle_last = NULL;
static int_void_fn fn_get_desired_angle_last = NULL;
static void_int_int_fn fn_set_angle_meas = NULL;
static int_void_fn fn_get_angle_meas_min = NULL;
static int_void_fn fn_get_angle_meas_max = NULL;
static bool_void_fn fn_get_cruise_engaged_prev = NULL;
static void_bool_fn fn_set_cruise_engaged_prev = NULL;
static bool_void_fn fn_get_vehicle_moving = NULL;
static void_u32_fn fn_set_timer = NULL;
static void (*fn_safety_tick_current_safety_config)(void) = NULL;
static bool_void_fn fn_safety_config_valid = NULL;
static void (*fn_init_tests)(void) = NULL;
static void_bool_fn fn_set_honda_fwd_brake = NULL;
static bool_void_fn fn_get_honda_fwd_brake = NULL;
static void_bool_fn fn_set_honda_alt_brake_msg = NULL;
static void_bool_fn fn_set_honda_bosch_long = NULL;
static int_void_fn fn_get_honda_hw = NULL;
static void_int_fn fn_mutation_set_active_mutant = NULL;
static int_void_fn fn_mutation_get_active_mutant = NULL;

static int require_loaded(void) {
  if (g_handle == NULL) {
    PyErr_SetString(PyExc_RuntimeError, "libsafety is not loaded");
    return 0;
  }
  return 1;
}

static int require_symbol(const void *fn, const char *name) {
  if (fn == NULL) {
    PyErr_Format(PyExc_RuntimeError, "missing symbol: %s", name);
    return 0;
  }
  return 1;
}

static void *load_symbol(const char *name, int required) {
  void *sym = dlsym(g_handle, name);
  if ((sym == NULL) && required) {
    PyErr_Format(PyExc_RuntimeError, "failed to load symbol %s: %s", name, dlerror());
  }
  return sym;
}

static int load_symbols(void) {
  fn_safety_rx_hook = (bool_packet_fn)load_symbol("safety_rx_hook", 1);
  fn_safety_tx_hook = (bool_packet_fn)load_symbol("safety_tx_hook", 1);
  fn_safety_fwd_hook = (int_int_int_fn)load_symbol("safety_fwd_hook", 1);
  fn_set_safety_hooks = (int_u16_u16_fn)load_symbol("set_safety_hooks", 1);
  fn_set_controls_allowed = (void_bool_fn)load_symbol("set_controls_allowed", 1);
  fn_get_controls_allowed = (bool_void_fn)load_symbol("get_controls_allowed", 1);
  fn_get_longitudinal_allowed = (bool_void_fn)load_symbol("get_longitudinal_allowed", 1);
  fn_set_alternative_experience = (void_int_fn)load_symbol("set_alternative_experience", 1);
  fn_get_alternative_experience = (int_void_fn)load_symbol("get_alternative_experience", 1);
  fn_set_relay_malfunction = (void_bool_fn)load_symbol("set_relay_malfunction", 1);
  fn_get_relay_malfunction = (bool_void_fn)load_symbol("get_relay_malfunction", 1);
  fn_get_gas_pressed_prev = (bool_void_fn)load_symbol("get_gas_pressed_prev", 1);
  fn_set_gas_pressed_prev = (void_bool_fn)load_symbol("set_gas_pressed_prev", 1);
  fn_get_brake_pressed_prev = (bool_void_fn)load_symbol("get_brake_pressed_prev", 1);
  fn_get_regen_braking_prev = (bool_void_fn)load_symbol("get_regen_braking_prev", 1);
  fn_get_steering_disengage_prev = (bool_void_fn)load_symbol("get_steering_disengage_prev", 1);
  fn_get_acc_main_on = (bool_void_fn)load_symbol("get_acc_main_on", 1);
  fn_get_vehicle_speed_min = (float_void_fn)load_symbol("get_vehicle_speed_min", 1);
  fn_get_vehicle_speed_max = (float_void_fn)load_symbol("get_vehicle_speed_max", 1);
  fn_get_current_safety_mode = (int_void_fn)load_symbol("get_current_safety_mode", 1);
  fn_get_current_safety_param = (int_void_fn)load_symbol("get_current_safety_param", 1);
  fn_set_torque_meas = (void_int_int_fn)load_symbol("set_torque_meas", 1);
  fn_get_torque_meas_min = (int_void_fn)load_symbol("get_torque_meas_min", 1);
  fn_get_torque_meas_max = (int_void_fn)load_symbol("get_torque_meas_max", 1);
  fn_set_torque_driver = (void_int_int_fn)load_symbol("set_torque_driver", 1);
  fn_get_torque_driver_min = (int_void_fn)load_symbol("get_torque_driver_min", 1);
  fn_get_torque_driver_max = (int_void_fn)load_symbol("get_torque_driver_max", 1);
  fn_set_desired_torque_last = (void_int_fn)load_symbol("set_desired_torque_last", 1);
  fn_set_rt_torque_last = (void_int_fn)load_symbol("set_rt_torque_last", 1);
  fn_set_desired_angle_last = (void_int_fn)load_symbol("set_desired_angle_last", 1);
  fn_get_desired_angle_last = (int_void_fn)load_symbol("get_desired_angle_last", 1);
  fn_set_angle_meas = (void_int_int_fn)load_symbol("set_angle_meas", 1);
  fn_get_angle_meas_min = (int_void_fn)load_symbol("get_angle_meas_min", 1);
  fn_get_angle_meas_max = (int_void_fn)load_symbol("get_angle_meas_max", 1);
  fn_get_cruise_engaged_prev = (bool_void_fn)load_symbol("get_cruise_engaged_prev", 1);
  fn_set_cruise_engaged_prev = (void_bool_fn)load_symbol("set_cruise_engaged_prev", 1);
  fn_get_vehicle_moving = (bool_void_fn)load_symbol("get_vehicle_moving", 1);
  fn_set_timer = (void_u32_fn)load_symbol("set_timer", 1);
  fn_safety_tick_current_safety_config = (void (*)(void))load_symbol("safety_tick_current_safety_config", 1);
  fn_safety_config_valid = (bool_void_fn)load_symbol("safety_config_valid", 1);
  fn_init_tests = (void (*)(void))load_symbol("init_tests", 1);
  fn_set_honda_fwd_brake = (void_bool_fn)load_symbol("set_honda_fwd_brake", 1);
  fn_get_honda_fwd_brake = (bool_void_fn)load_symbol("get_honda_fwd_brake", 1);
  fn_set_honda_alt_brake_msg = (void_bool_fn)load_symbol("set_honda_alt_brake_msg", 1);
  fn_set_honda_bosch_long = (void_bool_fn)load_symbol("set_honda_bosch_long", 1);
  fn_get_honda_hw = (int_void_fn)load_symbol("get_honda_hw", 1);
  fn_mutation_set_active_mutant = (void_int_fn)load_symbol("mutation_set_active_mutant", 0);
  fn_mutation_get_active_mutant = (int_void_fn)load_symbol("mutation_get_active_mutant", 0);

  return !PyErr_Occurred();
}

static CANPacketObject *packet_from_object(PyObject *obj) {
  if (!PyObject_TypeCheck(obj, &CANPacketType)) {
    PyErr_SetString(PyExc_TypeError, "expected CANPacket");
    return NULL;
  }
  return (CANPacketObject *)obj;
}

static PyObject *CANPacket_new(PyTypeObject *type, PyObject *args, PyObject *kwargs) {
  CANPacketObject *self = (CANPacketObject *)type->tp_alloc(type, 0);
  if (self != NULL) {
    memset(&self->packet, 0, sizeof(self->packet));
  }
  return (PyObject *)self;
}

static PyObject *CANPacket_subscript(PyObject *self, PyObject *item) {
  long index = PyLong_AsLong(item);
  if ((index == -1) && PyErr_Occurred()) return NULL;
  if (index != 0) {
    PyErr_SetString(PyExc_IndexError, "CANPacket only supports index 0");
    return NULL;
  }
  Py_INCREF(self);
  return self;
}

static PyMappingMethods CANPacket_mapping = {
  .mp_length = 0,
  .mp_subscript = CANPacket_subscript,
  .mp_ass_subscript = 0,
};

static PyObject *get_data_view(CANPacketObject *self, void *closure) {
  (void)closure;
  return PyMemoryView_FromMemory((char *)self->packet.data, CANPACKET_DATA_SIZE_MAX, PyBUF_WRITE);
}

static PyObject *get_uint_attr(unsigned int value) {
  return PyLong_FromUnsignedLong(value);
}

static int parse_small_uint(PyObject *value, unsigned int max_value, const char *name, unsigned int *out) {
  unsigned long v;
  if (value == NULL) {
    PyErr_Format(PyExc_AttributeError, "cannot delete %s", name);
    return -1;
  }
  v = PyLong_AsUnsignedLong(value);
  if ((v == (unsigned long)-1) && PyErr_Occurred()) return -1;
  if (v > max_value) {
    PyErr_Format(PyExc_ValueError, "%s out of range", name);
    return -1;
  }
  *out = (unsigned int)v;
  return 0;
}

static PyObject *packet_get_fd(CANPacketObject *self, void *closure) { (void)closure; return get_uint_attr(self->packet.fd); }
static int packet_set_fd(CANPacketObject *self, PyObject *value, void *closure) {
  unsigned int v;
  (void)closure;
  if (parse_small_uint(value, 1U, "fd", &v) < 0) return -1;
  self->packet.fd = v;
  return 0;
}
static PyObject *packet_get_bus(CANPacketObject *self, void *closure) { (void)closure; return get_uint_attr(self->packet.bus); }
static int packet_set_bus(CANPacketObject *self, PyObject *value, void *closure) {
  unsigned int v;
  (void)closure;
  if (parse_small_uint(value, 7U, "bus", &v) < 0) return -1;
  self->packet.bus = v;
  return 0;
}
static PyObject *packet_get_dlc(CANPacketObject *self, void *closure) { (void)closure; return get_uint_attr(self->packet.data_len_code); }
static int packet_set_dlc(CANPacketObject *self, PyObject *value, void *closure) {
  unsigned int v;
  (void)closure;
  if (parse_small_uint(value, 15U, "data_len_code", &v) < 0) return -1;
  self->packet.data_len_code = v;
  return 0;
}
static PyObject *packet_get_rejected(CANPacketObject *self, void *closure) { (void)closure; return get_uint_attr(self->packet.rejected); }
static int packet_set_rejected(CANPacketObject *self, PyObject *value, void *closure) {
  unsigned int v;
  (void)closure;
  if (parse_small_uint(value, 1U, "rejected", &v) < 0) return -1;
  self->packet.rejected = v;
  return 0;
}
static PyObject *packet_get_returned(CANPacketObject *self, void *closure) { (void)closure; return get_uint_attr(self->packet.returned); }
static int packet_set_returned(CANPacketObject *self, PyObject *value, void *closure) {
  unsigned int v;
  (void)closure;
  if (parse_small_uint(value, 1U, "returned", &v) < 0) return -1;
  self->packet.returned = v;
  return 0;
}
static PyObject *packet_get_extended(CANPacketObject *self, void *closure) { (void)closure; return get_uint_attr(self->packet.extended); }
static int packet_set_extended(CANPacketObject *self, PyObject *value, void *closure) {
  unsigned int v;
  (void)closure;
  if (parse_small_uint(value, 1U, "extended", &v) < 0) return -1;
  self->packet.extended = v;
  return 0;
}
static PyObject *packet_get_addr(CANPacketObject *self, void *closure) { (void)closure; return PyLong_FromUnsignedLong(self->packet.addr); }
static int packet_set_addr(CANPacketObject *self, PyObject *value, void *closure) {
  unsigned long v;
  (void)closure;
  if (value == NULL) {
    PyErr_SetString(PyExc_AttributeError, "cannot delete addr");
    return -1;
  }
  v = PyLong_AsUnsignedLong(value);
  if ((v == (unsigned long)-1) && PyErr_Occurred()) return -1;
  if (v > 0x1FFFFFFFU) {
    PyErr_SetString(PyExc_ValueError, "addr out of range");
    return -1;
  }
  self->packet.addr = (unsigned int)v;
  return 0;
}

static PyGetSetDef CANPacket_getset[] = {
  {"fd", (getter)packet_get_fd, (setter)packet_set_fd, NULL, NULL},
  {"bus", (getter)packet_get_bus, (setter)packet_set_bus, NULL, NULL},
  {"data_len_code", (getter)packet_get_dlc, (setter)packet_set_dlc, NULL, NULL},
  {"rejected", (getter)packet_get_rejected, (setter)packet_set_rejected, NULL, NULL},
  {"returned", (getter)packet_get_returned, (setter)packet_set_returned, NULL, NULL},
  {"extended", (getter)packet_get_extended, (setter)packet_set_extended, NULL, NULL},
  {"addr", (getter)packet_get_addr, (setter)packet_set_addr, NULL, NULL},
  {"data", (getter)get_data_view, NULL, NULL, NULL},
  {NULL, NULL, NULL, NULL, NULL},
};

#define LIBSAFETY_BOOL_PACKET_WRAPPER(py_name, fn_ptr) \
static PyObject *py_name(LibSafetyObject *self, PyObject *arg) { \
  CANPacketObject *packet; \
  (void)self; \
  if (!require_loaded() || !require_symbol((const void *)fn_ptr, #fn_ptr)) return NULL; \
  packet = packet_from_object(arg); \
  if (packet == NULL) return NULL; \
  if (fn_ptr(&packet->packet)) Py_RETURN_TRUE; \
  Py_RETURN_FALSE; \
}

#define LIBSAFETY_BOOL_VOID_WRAPPER(py_name, fn_ptr) \
static PyObject *py_name(LibSafetyObject *self, PyObject *Py_UNUSED(ignored)) { \
  (void)self; \
  if (!require_loaded() || !require_symbol((const void *)fn_ptr, #fn_ptr)) return NULL; \
  if (fn_ptr()) Py_RETURN_TRUE; \
  Py_RETURN_FALSE; \
}

#define LIBSAFETY_INT_VOID_WRAPPER(py_name, fn_ptr) \
static PyObject *py_name(LibSafetyObject *self, PyObject *Py_UNUSED(ignored)) { \
  (void)self; \
  if (!require_loaded() || !require_symbol((const void *)fn_ptr, #fn_ptr)) return NULL; \
  return PyLong_FromLong(fn_ptr()); \
}

#define LIBSAFETY_FLOAT_VOID_WRAPPER(py_name, fn_ptr) \
static PyObject *py_name(LibSafetyObject *self, PyObject *Py_UNUSED(ignored)) { \
  (void)self; \
  if (!require_loaded() || !require_symbol((const void *)fn_ptr, #fn_ptr)) return NULL; \
  return PyFloat_FromDouble((double)fn_ptr()); \
}

#define LIBSAFETY_VOID_BOOL_WRAPPER(py_name, fn_ptr) \
static PyObject *py_name(LibSafetyObject *self, PyObject *arg) { \
  int truthy; \
  (void)self; \
  if (!require_loaded() || !require_symbol((const void *)fn_ptr, #fn_ptr)) return NULL; \
  truthy = PyObject_IsTrue(arg); \
  if (truthy < 0) return NULL; \
  fn_ptr(truthy != 0); \
  Py_RETURN_NONE; \
}

#define LIBSAFETY_VOID_INT_WRAPPER(py_name, fn_ptr) \
static PyObject *py_name(LibSafetyObject *self, PyObject *arg) { \
  long value; \
  (void)self; \
  if (!require_loaded() || !require_symbol((const void *)fn_ptr, #fn_ptr)) return NULL; \
  value = PyLong_AsLong(arg); \
  if ((value == -1) && PyErr_Occurred()) return NULL; \
  fn_ptr((int)value); \
  Py_RETURN_NONE; \
}

LIBSAFETY_BOOL_PACKET_WRAPPER(LibSafety_safety_rx_hook, fn_safety_rx_hook)
LIBSAFETY_BOOL_PACKET_WRAPPER(LibSafety_safety_tx_hook, fn_safety_tx_hook)
LIBSAFETY_BOOL_VOID_WRAPPER(LibSafety_get_controls_allowed, fn_get_controls_allowed)
LIBSAFETY_BOOL_VOID_WRAPPER(LibSafety_get_longitudinal_allowed, fn_get_longitudinal_allowed)
LIBSAFETY_BOOL_VOID_WRAPPER(LibSafety_get_relay_malfunction, fn_get_relay_malfunction)
LIBSAFETY_BOOL_VOID_WRAPPER(LibSafety_get_gas_pressed_prev, fn_get_gas_pressed_prev)
LIBSAFETY_BOOL_VOID_WRAPPER(LibSafety_get_brake_pressed_prev, fn_get_brake_pressed_prev)
LIBSAFETY_BOOL_VOID_WRAPPER(LibSafety_get_regen_braking_prev, fn_get_regen_braking_prev)
LIBSAFETY_BOOL_VOID_WRAPPER(LibSafety_get_steering_disengage_prev, fn_get_steering_disengage_prev)
LIBSAFETY_BOOL_VOID_WRAPPER(LibSafety_get_acc_main_on, fn_get_acc_main_on)
LIBSAFETY_BOOL_VOID_WRAPPER(LibSafety_get_cruise_engaged_prev, fn_get_cruise_engaged_prev)
LIBSAFETY_BOOL_VOID_WRAPPER(LibSafety_get_vehicle_moving, fn_get_vehicle_moving)
LIBSAFETY_BOOL_VOID_WRAPPER(LibSafety_safety_config_valid, fn_safety_config_valid)
LIBSAFETY_BOOL_VOID_WRAPPER(LibSafety_get_honda_fwd_brake, fn_get_honda_fwd_brake)
LIBSAFETY_INT_VOID_WRAPPER(LibSafety_get_alternative_experience, fn_get_alternative_experience)
LIBSAFETY_INT_VOID_WRAPPER(LibSafety_get_current_safety_mode, fn_get_current_safety_mode)
LIBSAFETY_INT_VOID_WRAPPER(LibSafety_get_current_safety_param, fn_get_current_safety_param)
LIBSAFETY_INT_VOID_WRAPPER(LibSafety_get_torque_meas_min, fn_get_torque_meas_min)
LIBSAFETY_INT_VOID_WRAPPER(LibSafety_get_torque_meas_max, fn_get_torque_meas_max)
LIBSAFETY_INT_VOID_WRAPPER(LibSafety_get_torque_driver_min, fn_get_torque_driver_min)
LIBSAFETY_INT_VOID_WRAPPER(LibSafety_get_torque_driver_max, fn_get_torque_driver_max)
LIBSAFETY_INT_VOID_WRAPPER(LibSafety_get_desired_angle_last, fn_get_desired_angle_last)
LIBSAFETY_INT_VOID_WRAPPER(LibSafety_get_angle_meas_min, fn_get_angle_meas_min)
LIBSAFETY_INT_VOID_WRAPPER(LibSafety_get_angle_meas_max, fn_get_angle_meas_max)
LIBSAFETY_INT_VOID_WRAPPER(LibSafety_get_honda_hw, fn_get_honda_hw)
LIBSAFETY_INT_VOID_WRAPPER(LibSafety_mutation_get_active_mutant, fn_mutation_get_active_mutant)
LIBSAFETY_FLOAT_VOID_WRAPPER(LibSafety_get_vehicle_speed_min, fn_get_vehicle_speed_min)
LIBSAFETY_FLOAT_VOID_WRAPPER(LibSafety_get_vehicle_speed_max, fn_get_vehicle_speed_max)
LIBSAFETY_VOID_BOOL_WRAPPER(LibSafety_set_controls_allowed, fn_set_controls_allowed)
LIBSAFETY_VOID_BOOL_WRAPPER(LibSafety_set_relay_malfunction, fn_set_relay_malfunction)
LIBSAFETY_VOID_BOOL_WRAPPER(LibSafety_set_gas_pressed_prev, fn_set_gas_pressed_prev)
LIBSAFETY_VOID_BOOL_WRAPPER(LibSafety_set_cruise_engaged_prev, fn_set_cruise_engaged_prev)
LIBSAFETY_VOID_BOOL_WRAPPER(LibSafety_set_honda_fwd_brake, fn_set_honda_fwd_brake)
LIBSAFETY_VOID_BOOL_WRAPPER(LibSafety_set_honda_alt_brake_msg, fn_set_honda_alt_brake_msg)
LIBSAFETY_VOID_BOOL_WRAPPER(LibSafety_set_honda_bosch_long, fn_set_honda_bosch_long)
LIBSAFETY_VOID_INT_WRAPPER(LibSafety_set_alternative_experience, fn_set_alternative_experience)
LIBSAFETY_VOID_INT_WRAPPER(LibSafety_set_desired_torque_last, fn_set_desired_torque_last)
LIBSAFETY_VOID_INT_WRAPPER(LibSafety_set_rt_torque_last, fn_set_rt_torque_last)
LIBSAFETY_VOID_INT_WRAPPER(LibSafety_set_desired_angle_last, fn_set_desired_angle_last)
LIBSAFETY_VOID_INT_WRAPPER(LibSafety_mutation_set_active_mutant, fn_mutation_set_active_mutant)

static PyObject *LibSafety_safety_fwd_hook(LibSafetyObject *self, PyObject *args) {
  int bus_num, addr;
  (void)self;
  if (!require_loaded() || !require_symbol((const void *)fn_safety_fwd_hook, "safety_fwd_hook")) return NULL;
  if (!PyArg_ParseTuple(args, "ii", &bus_num, &addr)) return NULL;
  return PyLong_FromLong(fn_safety_fwd_hook(bus_num, addr));
}

static PyObject *LibSafety_set_safety_hooks(LibSafetyObject *self, PyObject *args) {
  unsigned int mode, param;
  (void)self;
  if (!require_loaded() || !require_symbol((const void *)fn_set_safety_hooks, "set_safety_hooks")) return NULL;
  if (!PyArg_ParseTuple(args, "II", &mode, &param)) return NULL;
  return PyLong_FromLong(fn_set_safety_hooks((uint16_t)mode, (uint16_t)param));
}

static PyObject *LibSafety_set_torque_meas(LibSafetyObject *self, PyObject *args) {
  int min_value, max_value;
  (void)self;
  if (!require_loaded() || !require_symbol((const void *)fn_set_torque_meas, "set_torque_meas")) return NULL;
  if (!PyArg_ParseTuple(args, "ii", &min_value, &max_value)) return NULL;
  fn_set_torque_meas(min_value, max_value);
  Py_RETURN_NONE;
}

static PyObject *LibSafety_set_torque_driver(LibSafetyObject *self, PyObject *args) {
  int min_value, max_value;
  (void)self;
  if (!require_loaded() || !require_symbol((const void *)fn_set_torque_driver, "set_torque_driver")) return NULL;
  if (!PyArg_ParseTuple(args, "ii", &min_value, &max_value)) return NULL;
  fn_set_torque_driver(min_value, max_value);
  Py_RETURN_NONE;
}

static PyObject *LibSafety_set_angle_meas(LibSafetyObject *self, PyObject *args) {
  int min_value, max_value;
  (void)self;
  if (!require_loaded() || !require_symbol((const void *)fn_set_angle_meas, "set_angle_meas")) return NULL;
  if (!PyArg_ParseTuple(args, "ii", &min_value, &max_value)) return NULL;
  fn_set_angle_meas(min_value, max_value);
  Py_RETURN_NONE;
}

static PyObject *LibSafety_set_timer(LibSafetyObject *self, PyObject *arg) {
  unsigned long value;
  (void)self;
  if (!require_loaded() || !require_symbol((const void *)fn_set_timer, "set_timer")) return NULL;
  value = PyLong_AsUnsignedLong(arg);
  if ((value == (unsigned long)-1) && PyErr_Occurred()) return NULL;
  fn_set_timer((uint32_t)value);
  Py_RETURN_NONE;
}

static PyObject *LibSafety_safety_tick_current_safety_config(LibSafetyObject *self, PyObject *Py_UNUSED(ignored)) {
  (void)self;
  if (!require_loaded() || !require_symbol((const void *)fn_safety_tick_current_safety_config, "safety_tick_current_safety_config")) return NULL;
  fn_safety_tick_current_safety_config();
  Py_RETURN_NONE;
}

static PyObject *LibSafety_init_tests(LibSafetyObject *self, PyObject *Py_UNUSED(ignored)) {
  (void)self;
  if (!require_loaded() || !require_symbol((const void *)fn_init_tests, "init_tests")) return NULL;
  fn_init_tests();
  Py_RETURN_NONE;
}

static PyMethodDef LibSafety_methods[] = {
  {"safety_rx_hook", (PyCFunction)LibSafety_safety_rx_hook, METH_O, NULL},
  {"safety_tx_hook", (PyCFunction)LibSafety_safety_tx_hook, METH_O, NULL},
  {"safety_fwd_hook", (PyCFunction)LibSafety_safety_fwd_hook, METH_VARARGS, NULL},
  {"set_safety_hooks", (PyCFunction)LibSafety_set_safety_hooks, METH_VARARGS, NULL},
  {"set_controls_allowed", (PyCFunction)LibSafety_set_controls_allowed, METH_O, NULL},
  {"get_controls_allowed", (PyCFunction)LibSafety_get_controls_allowed, METH_NOARGS, NULL},
  {"get_longitudinal_allowed", (PyCFunction)LibSafety_get_longitudinal_allowed, METH_NOARGS, NULL},
  {"set_alternative_experience", (PyCFunction)LibSafety_set_alternative_experience, METH_O, NULL},
  {"get_alternative_experience", (PyCFunction)LibSafety_get_alternative_experience, METH_NOARGS, NULL},
  {"set_relay_malfunction", (PyCFunction)LibSafety_set_relay_malfunction, METH_O, NULL},
  {"get_relay_malfunction", (PyCFunction)LibSafety_get_relay_malfunction, METH_NOARGS, NULL},
  {"get_gas_pressed_prev", (PyCFunction)LibSafety_get_gas_pressed_prev, METH_NOARGS, NULL},
  {"set_gas_pressed_prev", (PyCFunction)LibSafety_set_gas_pressed_prev, METH_O, NULL},
  {"get_brake_pressed_prev", (PyCFunction)LibSafety_get_brake_pressed_prev, METH_NOARGS, NULL},
  {"get_regen_braking_prev", (PyCFunction)LibSafety_get_regen_braking_prev, METH_NOARGS, NULL},
  {"get_steering_disengage_prev", (PyCFunction)LibSafety_get_steering_disengage_prev, METH_NOARGS, NULL},
  {"get_acc_main_on", (PyCFunction)LibSafety_get_acc_main_on, METH_NOARGS, NULL},
  {"get_vehicle_speed_min", (PyCFunction)LibSafety_get_vehicle_speed_min, METH_NOARGS, NULL},
  {"get_vehicle_speed_max", (PyCFunction)LibSafety_get_vehicle_speed_max, METH_NOARGS, NULL},
  {"get_current_safety_mode", (PyCFunction)LibSafety_get_current_safety_mode, METH_NOARGS, NULL},
  {"get_current_safety_param", (PyCFunction)LibSafety_get_current_safety_param, METH_NOARGS, NULL},
  {"set_torque_meas", (PyCFunction)LibSafety_set_torque_meas, METH_VARARGS, NULL},
  {"get_torque_meas_min", (PyCFunction)LibSafety_get_torque_meas_min, METH_NOARGS, NULL},
  {"get_torque_meas_max", (PyCFunction)LibSafety_get_torque_meas_max, METH_NOARGS, NULL},
  {"set_torque_driver", (PyCFunction)LibSafety_set_torque_driver, METH_VARARGS, NULL},
  {"get_torque_driver_min", (PyCFunction)LibSafety_get_torque_driver_min, METH_NOARGS, NULL},
  {"get_torque_driver_max", (PyCFunction)LibSafety_get_torque_driver_max, METH_NOARGS, NULL},
  {"set_desired_torque_last", (PyCFunction)LibSafety_set_desired_torque_last, METH_O, NULL},
  {"set_rt_torque_last", (PyCFunction)LibSafety_set_rt_torque_last, METH_O, NULL},
  {"set_desired_angle_last", (PyCFunction)LibSafety_set_desired_angle_last, METH_O, NULL},
  {"get_desired_angle_last", (PyCFunction)LibSafety_get_desired_angle_last, METH_NOARGS, NULL},
  {"set_angle_meas", (PyCFunction)LibSafety_set_angle_meas, METH_VARARGS, NULL},
  {"get_angle_meas_min", (PyCFunction)LibSafety_get_angle_meas_min, METH_NOARGS, NULL},
  {"get_angle_meas_max", (PyCFunction)LibSafety_get_angle_meas_max, METH_NOARGS, NULL},
  {"get_cruise_engaged_prev", (PyCFunction)LibSafety_get_cruise_engaged_prev, METH_NOARGS, NULL},
  {"set_cruise_engaged_prev", (PyCFunction)LibSafety_set_cruise_engaged_prev, METH_O, NULL},
  {"get_vehicle_moving", (PyCFunction)LibSafety_get_vehicle_moving, METH_NOARGS, NULL},
  {"set_timer", (PyCFunction)LibSafety_set_timer, METH_O, NULL},
  {"safety_tick_current_safety_config", (PyCFunction)LibSafety_safety_tick_current_safety_config, METH_NOARGS, NULL},
  {"safety_config_valid", (PyCFunction)LibSafety_safety_config_valid, METH_NOARGS, NULL},
  {"init_tests", (PyCFunction)LibSafety_init_tests, METH_NOARGS, NULL},
  {"set_honda_fwd_brake", (PyCFunction)LibSafety_set_honda_fwd_brake, METH_O, NULL},
  {"get_honda_fwd_brake", (PyCFunction)LibSafety_get_honda_fwd_brake, METH_NOARGS, NULL},
  {"set_honda_alt_brake_msg", (PyCFunction)LibSafety_set_honda_alt_brake_msg, METH_O, NULL},
  {"set_honda_bosch_long", (PyCFunction)LibSafety_set_honda_bosch_long, METH_O, NULL},
  {"get_honda_hw", (PyCFunction)LibSafety_get_honda_hw, METH_NOARGS, NULL},
  {"mutation_set_active_mutant", (PyCFunction)LibSafety_mutation_set_active_mutant, METH_O, NULL},
  {"mutation_get_active_mutant", (PyCFunction)LibSafety_mutation_get_active_mutant, METH_NOARGS, NULL},
  {NULL, NULL, 0, NULL},
};

static PyObject *module_load(PyObject *module, PyObject *arg) {
  const char *path;
  (void)module;
  if (!PyUnicode_Check(arg)) {
    PyErr_SetString(PyExc_TypeError, "path must be a string");
    return NULL;
  }
  path = PyUnicode_AsUTF8(arg);
  if (path == NULL) return NULL;

  if (g_handle != NULL) {
    dlclose(g_handle);
    g_handle = NULL;
  }

  g_handle = dlopen(path, RTLD_NOW | RTLD_LOCAL);
  if (g_handle == NULL) {
    PyErr_Format(PyExc_RuntimeError, "dlopen failed: %s", dlerror());
    return NULL;
  }

  if (!load_symbols()) {
    dlclose(g_handle);
    g_handle = NULL;
    return NULL;
  }

  Py_INCREF((PyObject *)g_libsafety);
  return (PyObject *)g_libsafety;
}

static PyObject *module_make_CANPacket(PyObject *module, PyObject *args) {
  unsigned long addr;
  unsigned long bus;
  unsigned long dlc;
  const unsigned char *data;
  Py_ssize_t data_len;
  CANPacketObject *packet;
  (void)module;

  if (!PyArg_ParseTuple(args, "kkky#", &addr, &bus, &dlc, &data, &data_len)) return NULL;
  if (data_len < 0 || data_len > CANPACKET_DATA_SIZE_MAX) {
    PyErr_SetString(PyExc_ValueError, "invalid CAN packet length");
    return NULL;
  }

  packet = (CANPacketObject *)CANPacket_new(&CANPacketType, NULL, NULL);
  if (packet == NULL) return NULL;

  packet->packet.extended = addr >= 0x800U;
  packet->packet.addr = (unsigned int)addr;
  packet->packet.data_len_code = (unsigned int)dlc;
  packet->packet.bus = (unsigned char)bus;
  memcpy(packet->packet.data, data, (size_t)data_len);

  return (PyObject *)packet;
}

static PyMethodDef module_methods[] = {
  {"load", (PyCFunction)module_load, METH_O, NULL},
  {"make_CANPacket", (PyCFunction)module_make_CANPacket, METH_VARARGS, NULL},
  {NULL, NULL, 0, NULL},
};

static struct PyModuleDef moduledef = {
  PyModuleDef_HEAD_INIT,
  .m_name = "_libsafety_c_ext",
  .m_doc = NULL,
  .m_size = -1,
  .m_methods = module_methods,
};

PyMODINIT_FUNC PyInit__libsafety_c_ext(void) {
  PyObject *module;

  CANPacketType = (PyTypeObject){
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "_libsafety_c_ext.CANPacket",
    .tp_basicsize = sizeof(CANPacketObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_new = CANPacket_new,
    .tp_as_mapping = &CANPacket_mapping,
    .tp_getset = CANPacket_getset,
  };

  LibSafetyType = (PyTypeObject){
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "_libsafety_c_ext.LibSafety",
    .tp_basicsize = sizeof(LibSafetyObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_methods = LibSafety_methods,
  };

  if (PyType_Ready(&CANPacketType) < 0) return NULL;
  if (PyType_Ready(&LibSafetyType) < 0) return NULL;

  module = PyModule_Create(&moduledef);
  if (module == NULL) return NULL;

  Py_INCREF(&CANPacketType);
  if (PyModule_AddObject(module, "CANPacket", (PyObject *)&CANPacketType) < 0) return NULL;

  Py_INCREF(&LibSafetyType);
  if (PyModule_AddObject(module, "LibSafety", (PyObject *)&LibSafetyType) < 0) return NULL;

  g_libsafety = PyObject_New(LibSafetyObject, &LibSafetyType);
  if (g_libsafety == NULL) return NULL;
  if (PyModule_AddObject(module, "libsafety", (PyObject *)g_libsafety) < 0) return NULL;

  return module;
}
