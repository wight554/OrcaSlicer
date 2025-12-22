#ifndef slic3r_PresetRenameHistory_hpp_
#define slic3r_PresetRenameHistory_hpp_

#include <string>
#include <vector>
#include <optional>

#include <boost/filesystem/path.hpp>

#include "Preset.hpp"

namespace Slic3r {

struct RenameHistoryEntry {
    Preset::Type type { Preset::TYPE_INVALID };
    std::string  old_name;
    std::string  new_name;
    long long    timestamp { 0 };
};

class PresetRenameHistory
{
public:
    static PresetRenameHistory& instance();

    void add_entry(Preset::Type type, const std::string& old_name, const std::string& new_name);

    // Returns the latest name for a preset if it was renamed, otherwise std::nullopt.
    std::optional<std::string> resolve(Preset::Type type, const std::string& name) const;

    const std::vector<RenameHistoryEntry>& entries() const { return m_entries; }

private:
    PresetRenameHistory();

    void load();
    void save() const;

    static std::string type_to_string(Preset::Type type);
    static Preset::Type type_from_string(const std::string& type);

    std::vector<RenameHistoryEntry> m_entries;
    boost::filesystem::path         m_path;
};

} // namespace Slic3r

#endif // slic3r_PresetRenameHistory_hpp_
