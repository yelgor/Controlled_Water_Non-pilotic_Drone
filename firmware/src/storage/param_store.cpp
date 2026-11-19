#include "param_store.h"

#include <Preferences.h>

#include "param_table.h"

namespace storage {

namespace {
constexpr const char* kNs = "drone";
constexpr const char* kKeyVer = "ver";
constexpr const char* kKeyData = "params";
}  // namespace

bool loadParams(core::Params& out) {
    Preferences prefs;
    if (!prefs.begin(kNs, true)) return false;
    const uint32_t ver = prefs.getUInt(kKeyVer, 0);
    core::Params p;
    const size_t n = prefs.getBytes(kKeyData, &p, sizeof(p));
    prefs.end();
    if (ver != core::kParamsVersion || n != sizeof(p) || !tuning::paramsValid(p)) return false;
    out = p;
    return true;
}

bool saveParams(const core::Params& p) {
    if (!tuning::paramsValid(p)) return false;
    Preferences prefs;
    if (!prefs.begin(kNs, false)) return false;
    const bool ok = prefs.putBytes(kKeyData, &p, sizeof(p)) == sizeof(p) &&
                    prefs.putUInt(kKeyVer, core::kParamsVersion) > 0;
    prefs.end();
    return ok;
}

}  // namespace storage
