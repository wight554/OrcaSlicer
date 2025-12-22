#ifndef slic3r_GUI_RenamedProfilesDialog_hpp_
#define slic3r_GUI_RenamedProfilesDialog_hpp_

#include <vector>
#include <string>

#include "wxExtensions.hpp"
#include "Widgets/DialogButtons.hpp"
#include "Widgets/CheckBox.hpp"

#include "libslic3r/Preset.hpp"

namespace Slic3r {
namespace GUI {

struct RenameUpdateOption {
    Preset::Type type { Preset::TYPE_INVALID };
    std::string  old_name;
    std::string  new_name;
};

class RenamedProfilesDialog : public DPIDialog
{
public:
    RenamedProfilesDialog(wxWindow *parent, const std::vector<RenameUpdateOption> &options);

    std::vector<RenameUpdateOption> selection() const;

protected:
    void on_dpi_changed(const wxRect &suggested_rect) override;
    void on_sys_color_changed() override {}

private:
    wxString describe_option(const RenameUpdateOption &option) const;

    std::vector<RenameUpdateOption> m_options;
    std::vector<::CheckBox*>        m_checkboxes;
    wxPanel *                       m_list_panel { nullptr };
    DialogButtons *                 m_buttons { nullptr };
};

} // namespace GUI
} // namespace Slic3r

#endif // slic3r_GUI_RenamedProfilesDialog_hpp_
