// C ABI around rig::Core so the Python digital twin runs the real firmware logic.
#include "../sma_rig/rig_core.h"

#define API extern "C" __declspec(dllexport)

API void* rig_new(uint32_t now_ms) {
  rig::Core* c = new rig::Core();
  c->begin(now_ms);
  return c;
}
API void rig_free(void* c) { delete static_cast<rig::Core*>(c); }
API void rig_command(void* c, const char* line) { static_cast<rig::Core*>(c)->command(line); }
API void rig_update(void* c, const rig::Inputs* in, rig::Outputs* out) { static_cast<rig::Core*>(c)->update(*in, *out); }
API int rig_telemetry(void* c, char* buf, int len) { return static_cast<rig::Core*>(c)->telemetry(buf, (size_t)len) ? 1 : 0; }
API int rig_message(void* c, char* buf, int len) { return static_cast<rig::Core*>(c)->pending_message(buf, (size_t)len) ? 1 : 0; }
API int rig_sizeof_inputs() { return (int)sizeof(rig::Inputs); }
API int rig_sizeof_outputs() { return (int)sizeof(rig::Outputs); }
