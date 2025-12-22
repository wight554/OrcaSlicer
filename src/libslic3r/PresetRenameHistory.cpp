#include "PresetRenameHistory.hpp"

#include <algorithm>
#include <chrono>
#include <unordered_set>

#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>

#include <nlohmann/json.hpp>

#include "Utils.hpp"

namespace Slic3r {

namespace {
constexpr size_t kMaxResolveDepth = 32;
}

PresetRenameHistory& PresetRenameHistory::instance()
{
    static PresetRenameHistory history;
    return history;
}

PresetRenameHistory::PresetRenameHistory()
{
    boost::filesystem::path dir = boost::filesystem::path(data_dir()) / PRESET_USER_DIR;
    m_path = dir / "rename_history.json";
    load();
}

void PresetRenameHistory::load()
{
    m_entries.clear();
    if (m_path.empty())
        return;
    boost::filesystem::create_directories(m_path.parent_path());
    if (!boost::filesystem::exists(m_path))
        return;

    try {
        boost::nowide::ifstream ifs(m_path.string());
        if (!ifs.good())
            return;
        nlohmann::json json_data;
        ifs >> json_data;
        const auto &entries = json_data.contains("entries") ? json_data["entries"] : nlohmann::json::array();
        for (const auto &item : entries) {
            if (!item.is_object())
                continue;
            RenameHistoryEntry entry;
            entry.type      = type_from_string(item.value("type", ""));
            entry.old_name  = item.value("old", std::string());
            entry.new_name  = item.value("new", std::string());
            entry.timestamp = item.value("timestamp", 0ll);
            if (entry.type == Preset::TYPE_INVALID || entry.old_name.empty() || entry.new_name.empty())
                continue;
            m_entries.emplace_back(std::move(entry));
        }
    } catch (...) {
        // ignore malformed files
    }
}

void PresetRenameHistory::save() const
{
    try {
        nlohmann::json json_data;
        json_data["entries"] = nlohmann::json::array();
        for (const auto &entry : m_entries) {
            json_data["entries"].push_back({
                {"type", type_to_string(entry.type)},
                {"old", entry.old_name},
                {"new", entry.new_name},
                {"timestamp", entry.timestamp}
            });
        }
        boost::filesystem::create_directories(m_path.parent_path());
        boost::nowide::ofstream ofs(m_path.string(), std::ios::out | std::ios::trunc);
        ofs << json_data.dump(2);
    } catch (...) {
        // ignore failure
    }
}

void PresetRenameHistory::add_entry(Preset::Type type, const std::string& old_name, const std::string& new_name)
{
    if (type == Preset::TYPE_INVALID || old_name.empty() || new_name.empty() || old_name == new_name)
        return;

    RenameHistoryEntry entry;
    entry.type      = type;
    entry.old_name  = old_name;
    entry.new_name  = new_name;
    entry.timestamp = static_cast<long long>(std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    m_entries.emplace_back(std::move(entry));
    save();
}

std::optional<std::string> PresetRenameHistory::resolve(Preset::Type type, const std::string& name) const
{
    if (type == Preset::TYPE_INVALID || name.empty())
        return std::nullopt;

    std::string current = name;
    bool        changed = false;
    std::unordered_set<std::string> visited;
    visited.insert(current);

    for (size_t depth = 0; depth < kMaxResolveDepth; ++depth) {
        auto it = std::find_if(m_entries.rbegin(), m_entries.rend(), [&](const RenameHistoryEntry &entry) {
            return entry.type == type && entry.old_name == current;
        });
        if (it == m_entries.rend())
            break;
        current = it->new_name;
        if (!visited.insert(current).second)
            break;
        changed = true;
    }

    if (changed && current != name)
        return current;
    return std::nullopt;
}

std::string PresetRenameHistory::type_to_string(Preset::Type type)
{
    switch (type) {
    case Preset::TYPE_PRINTER:  return "printer";
    case Preset::TYPE_FILAMENT: return "filament";
    default:                    return "unknown";
    }
}

Preset::Type PresetRenameHistory::type_from_string(const std::string& type)
{
    if (type == "printer")
        return Preset::TYPE_PRINTER;
    if (type == "filament")
        return Preset::TYPE_FILAMENT;
    return Preset::TYPE_INVALID;
}

} // namespace Slic3r
